#include "solver.hpp"

#include <yaml-cpp/yaml.h>

#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
  // ============================================================================
  // 坐标解算：把"图像里的 4 个像素点"变成"世界坐标系下的位置和朝向"。
  //
  // 先记住四个坐标系（变量名里的 2 都是 "to"）：
  //   armor  装甲板自身坐标系。原点在板中心，板面就是 x=0 平面（见下面物点）
  //   camera 相机坐标系，原点在光心
  //   gimbal 云台坐标系，原点在云台旋转中心
  //   world  世界坐标系。原点仍在云台，但**姿态被重力和绝对 yaw 对齐**：
  //          车怎么转，world 的朝向都不变（跟着车平移，但不跟着车旋转）。
  //          正因如此，EKF 才能在 world 系里用"匀速直线运动"这么简单的模型描述敌方车。
  //
  // 变换链条：armor --PnP--> camera --标定外参--> gimbal --云台姿态--> world
  // 前两跳靠标定出来的固定参数，最后一跳靠电控实时回传的四元数。
  // ============================================================================

  // 装甲板真实物理尺寸，单位【米】（56e-3 即 56mm）。
  // 这三个数是 PnP 的"尺子"——它靠"实物多大 + 图上多大"反推距离，
  // 所以这里错多少，解出的距离就整体错多少，后面弹道和预测全跟着偏。
  constexpr double LIGHTBAR_LENGTH = 56e-3;     // m
  constexpr double BIG_ARMOR_WIDTH = 230e-3;    // m
  constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // m

  // PnP 的 3D 物点（armor 系）。x 全为 0 —— 四点共面，这是能用 SOLVEPNP_IPPE 的前提。
  // 顺序必须与 Armor::points 的像素点顺序严格对应：{左灯条上, 右灯条上, 右灯条下, 左灯条下}。
  // 对照下面 y/z 的符号可验证：y 正方向指向左侧，z 正方向朝上。
  const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
    {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
  const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
    {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
    {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
    {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

  Solver::Solver(const std::string & config_path) : R_gimbal2world_(Eigen::Matrix3d::Identity())
  {
    auto yaml = YAML::LoadFile(config_path);

    // ★下面这些全是**标定工具生成的**，不是手调参数：
    //   R_camera2gimbal / t_camera2gimbal <- 手眼标定 calibration/calibrate_handeye.cpp
    //   camera_matrix / distort_coeffs    <- 相机内参标定 calibration/calibrate_camera.cpp
    // 手改任何一个都会毁掉 PnP，而且症状是"距离一直不对"这种难查的偏差，不是崩溃。
    // 现在没相机没车，这几项属于"别碰"清单。
    auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
    auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
    auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
    R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
    R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
    t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

    // 相机内参：camera_matrix 是 3x3 的 fx/fy/cx/cy（焦距与光心，单位像素）；
    // distort_coeffs 是 5 个畸变系数（镜头把直线拍弯了，PnP 前要把畸变考虑进去）。
    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
    Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
    cv::eigen2cv(camera_matrix, camera_matrix_);
    cv::eigen2cv(distort_coeffs, distort_coeffs_);
  }

  Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

  // 喂入云台当前姿态，建立 gimbal -> world 的旋转。每帧调一次，且必须在 solve() 之前。
  //
  // 【四元数是什么】陀螺仪告诉你"现在朝哪、歪多少"。这件事可以用欧拉角(yaw/pitch/roll)表示，
  // 但欧拉角有万向锁、插值不连续的毛病，所以工程上普遍用四个数 (w,x,y,z) 的四元数。
  // 你在电控侧从 IMU 拿到的就是它。这里不需要懂它的数学，只要知道：
  // 四元数 ≈ 一个"朝向"；toRotationMatrix() 把它变成 3x3 旋转矩阵；
  // 而"旋转矩阵乘一个点"就等于把这个点从一个坐标系换算到另一个坐标系。
  //
  // 下面那行是相似变换：R_gimbal2imubody 描述陀螺仪装在云台上的安装偏差（IMU 轴与云台轴不重合）。
  // 先转到 IMU 系、应用姿态、再转回云台系，把安装偏差抵消掉，得到纯粹的"云台相对世界"的旋转。
  void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
  {
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
  }

  // 单块装甲板的完整解算：像素 -> 世界坐标 + 世界朝向。识别出的每块板都要过一遍。
  //
  // 【PnP 是什么】Perspective-n-Point。已知 n 个点在物体上的真实三维位置（这里 n=4，
  // 就是上面那两组物点），又知道它们落在图像上的像素位置（armor.points），
  // 就能反解出"物体相对相机的位置和朝向"。
  // 直观理解：同一块板离得越远在图上越小；侧过去时四个角会变成梯形——
  // 从这个形变里就能反推距离和角度。
  // 电控类比：类似用编码器读数反推机构位置，只不过这里的"传感器"是一张图。
  //solvePnP（获得姿态）
  void Solver::solve(Armor & armor) const
  {
    // 大板小板物点尺寸不同，选错则整个距离都错（type 由 detector 的 get_type 判定）
    const auto & object_points = (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    // rvec: 旋转（Rodrigues 紧凑形式，3 个数）；tvec: 平移，即位置，单位【米】。
    // SOLVEPNP_IPPE 是专给"4 个共面点"的解法，比通用迭代快且稳；
    // 代价是对这种长宽比的扁平靶，它给出的 yaw 有歧义 —— 所以后面还要 optimize_yaw 重求。
    cv::Vec3d rvec, tvec;
    cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    // 位置的两跳：camera -> gimbal（标定外参，含平移）-> world（云台姿态，只旋转）。
    // world 这一跳没有平移项，因为 world 的原点就定在云台上。
    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

    // 朝向的同样两跳。Rodrigues 把 rvec 展开成 3x3 旋转矩阵，连乘完成坐标系接力；
    // eulers(...,2,1,0) 再把矩阵转成 yaw-pitch-roll，单位【弧度】。
    // 其中 ypr_in_world[0]（yaw）是反小陀螺最关键的量：它表示这块板正面朝向哪。
    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

    // 顺带算一份球坐标 (yaw, pitch, distance)：EKF 的观测量用的就是这种形式。
    // 因为相机"测角准、测距糙"，用球坐标才能在噪声矩阵 R 里分别给它们不同的信任度。
    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

    // 平衡不做yaw优化，因为pitch假设不成立
    auto is_balance = (armor.type == ArmorType::big) && (armor.name == ArmorName::three || armor.name == ArmorName::four || armor.name == ArmorName::five);
    if (is_balance) return;

    optimize_yaw(armor);
  }

  void Solver::omn_dig_yaw_solve(Armor & armor, Eigen::Vector3d R_camera2biggimbal_ypr, Eigen::Vector3d t_camera2biggimbal) const
  {
    cv::Vec3d rvec, tvec;

    const auto & object_points = (armor.type == auto_aim::ArmorType::big) ? auto_aim::BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);

    // R_camera2biggimbal_ypr = Eigen::Vector3d(0,0, 105.0);
    Eigen::Matrix3d R_camera2biggimbal = tools::rotation_matrix(R_camera2biggimbal_ypr);
    // Eigen::Vector3d t_camera2biggimbal = Eigen::Vector3d(0.0, 0.0, 0.0);
    armor.xyz_in_gimbal = R_camera2biggimbal * xyz_in_camera + t_camera2biggimbal;
    // armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

    // cv::Mat rmat;
    // cv::Rodrigues(rvec, rmat);
    // Eigen::Matrix3d R_armor2camera;
    // cv::cv2eigen(rmat, R_armor2camera);
    // Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    // // Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    // armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
    // armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

    // armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  }

  std::vector<cv::Point2f> Solver::reproject_armor(const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
  {
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d & t_armor2world = xyz_in_world;
    Eigen::Matrix3d R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
    Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, rvec);
    cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // reproject
    std::vector<cv::Point2f> image_points;
    const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
    return image_points;
  }

  double Solver::oupost_reprojection_error(Armor armor, const double & pitch)
  {
    // solve
    const auto & object_points = (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

    cv::Vec3d rvec, tvec;
    cv::solvePnP(object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    Eigen::Vector3d xyz_in_camera;
    cv::cv2eigen(tvec, xyz_in_camera);
    armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
    armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

    cv::Mat rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_armor2camera;
    cv::cv2eigen(rmat, R_armor2camera);
    Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
    Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
    armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
    armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

    auto yaw = armor.ypr_in_world[0];
    auto xyz_in_world = armor.xyz_in_world;

    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);

    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
    // clang-format on

    // get R_armor2camera t_armor2camera
    const Eigen::Vector3d & t_armor2world = xyz_in_world;
    Eigen::Matrix3d _R_armor2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
    Eigen::Vector3d t_armor2camera = R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

    // get rvec tvec
    cv::Vec3d _rvec;
    cv::Mat R_armor2camera_cv;
    cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
    cv::Rodrigues(R_armor2camera_cv, _rvec);
    cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

    // reproject
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

    auto error = 0.0;
    for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
    return error;
  }

  // 用暴力搜索重新求 yaw，覆盖 PnP 给出的那个。
  //
  // 【为什么不信 PnP 的 yaw】装甲板是又扁又小的平面靶（小板宽 135mm、灯条高 56mm）。
  // 在这种长宽比下，"板转了一点角度"和"板稍微远近了一点"造成的四角位移非常相似，
  // 于是 IPPE 解出的 yaw 会在两个相近解之间反复跳，抖得厉害。
  // 距离和 pitch 受影响不大，照用；只有 yaw 需要重求。
  //
  // 【怎么重求】既然正着解不稳，就反过来穷举：假设 yaw 取某个值，把 3D 板按此假设投影回图像，
  // 看投出来的四点与实际检测到的四点差多远（重投影误差），取误差最小的那个假设。
  // 这就是"生成-检验"，比解析解稳得多。
  //
  // 搜索范围 140 度、步长 1 度、共 140 次（SEARCH_RANGE 同时充当范围和循环次数）；
  // 以云台当前 yaw 为中心左右各 70 度——能被看到的板，正面不会偏离视线太多。
  // 代价是每块板要做 140 次重投影，这也是 solve 的主要耗时来源。
  // 原始 PnP 的 yaw 不会丢，存进 armor.yaw_raw 供对比（曲线里的 armor_yaw_raw 就是它）。
  void Solver::optimize_yaw(Armor & armor) const
  {
    Eigen::Vector3d gimbal_ypr = tools::eulers(R_gimbal2world_, 2, 1, 0);

    constexpr double SEARCH_RANGE = 140;  // degree
    auto yaw0 = tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);

    auto min_error = 1e10;
    auto best_yaw = armor.ypr_in_world[0];

    for (int i = 0; i < SEARCH_RANGE; i++)
    {
      double yaw = tools::limit_rad(yaw0 + i * CV_PI / 180.0);
      auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

      if (error < min_error)
      {
        min_error = error;
        best_yaw = yaw;
      }
    }

    armor.yaw_raw = armor.ypr_in_world[0];
    armor.ypr_in_world[0] = best_yaw;
  }

  double Solver::SJTU_cost(const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts, const double & inclined) const
  {
    std::size_t size = cv_refs.size();
    std::vector<Eigen::Vector2d> refs;
    std::vector<Eigen::Vector2d> pts;
    for (std::size_t i = 0u; i < size; ++i)
    {
      refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
      pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
    }
    double cost = 0.;
    for (std::size_t i = 0u; i < size; ++i)
    {
      std::size_t p = (i + 1u) % size;
      // i - p 构成线段。过程：先移动起点，再补长度，再旋转
      Eigen::Vector2d ref_d = refs[p] - refs[i];  // 标准
      Eigen::Vector2d pt_d = pts[p] - pts[i];
      // 长度差代价 + 起点差代价(1 / 2)（0 度左右应该抛弃)
      double pixel_dis =  // dis 是指方差平面内到原点的距离
        (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) + std::fabs(ref_d.norm() - pt_d.norm())) / ref_d.norm();
      double angular_dis = ref_d.norm() * tools::get_abs_angle(ref_d, pt_d) / ref_d.norm();
      // 平方可能是为了配合 sin 和 cos
      // 弧度差代价（0 度左右占比应该大）
      double cost_i = tools::square(pixel_dis * std::sin(inclined)) + tools::square(angular_dis * std::cos(inclined)) * 2.0;  // DETECTOR_ERROR_PIXEL_BY_SLOPE
      // 重投影像素误差越大，越相信斜率
      cost += std::sqrt(cost_i);
    }
    return cost;
  }

  double Solver::armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const
  {
    auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
    auto error = 0.0;
    for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
    // auto error = SJTU_cost(image_points, armor.points, inclined);

    return error;
  }

  // 世界坐标到像素坐标的转换
  std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints)
  {
    Eigen::Matrix3d R_world2camera = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
    Eigen::Vector3d t_world2camera = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

    cv::Mat rvec;
    cv::Mat tvec;
    cv::eigen2cv(R_world2camera, rvec);
    cv::eigen2cv(t_world2camera, tvec);

    std::vector<cv::Point3f> valid_world_points;
    for (const auto & world_point : worldPoints)
    {
      Eigen::Vector3d world_point_eigen(world_point.x, world_point.y, world_point.z);
      Eigen::Vector3d camera_point = R_world2camera * world_point_eigen + t_world2camera;

      if (camera_point.z() > 0)
      {
        valid_world_points.push_back(world_point);
      }
    }
    // 如果没有有效点，返回空vector
    if (valid_world_points.empty())
    {
      return std::vector<cv::Point2f>();
    }
    std::vector<cv::Point2f> pixelPoints;
    cv::projectPoints(valid_world_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixelPoints);
    return pixelPoints;
  }
}  // namespace auto_aim