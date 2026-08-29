#include "detector.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace auto_aim
{
  // ============================================================================
  // 传统CV 装甲板识别（不用神经网络）。
  //
  // ★这是**上场实际跑的识别路径**。
  //   上场主程序 src/rb_auto_aim_debug.cpp:158 调的是 detector.detect(img)；
  //   它虽然在 :49 构造了 YOLO、加载并编译了 OpenVINO 模型，却没有任何一次 yolo.detect 调用
  //   （全仓库唯一的 yolo.detect 在 tests/auto_aim_test.cpp，即离线那条路）。
  //   所以：在离线视频里调好的识别参数，不一定是车上生效的那条路，这点务必分清。
  //   另外 YOLO 那条路也会回头用传统CV 精修角点：yolos/yolov5.cpp:200 在 use_traditional_
  //   为真时调下面 detect(Armor&, const cv::Mat&) 那个重载，而 demo.yaml 和 sb_long.yaml
  //   都写着 use_traditional: true。所以本文件的代码两条路都会碰到。
  //
  // 整体流程：
  //   灰度化 -> 二值化（只留亮的）-> 找轮廓 -> 套最小外接旋转矩形当灯条候选 -> 几何筛选
  //   -> 同色灯条两两配对成板 -> 几何筛选 -> 认数字 -> 共用灯条去重
  // 每一步的阈值基本都来自 yaml，构造函数读的就是它们。
  // ============================================================================
  Detector::Detector(const std::string & config_path, bool debug) : classifier_(config_path), debug_(debug)
  {
    auto yaml = YAML::LoadFile(config_path);

    threshold_ = yaml["threshold"].as<double>();
    max_angle_error_ = yaml["max_angle_error"].as<double>() / 57.3;  // degree to rad
    min_lightbar_ratio_ = yaml["min_lightbar_ratio"].as<double>();
    max_lightbar_ratio_ = yaml["max_lightbar_ratio"].as<double>();
    min_lightbar_length_ = yaml["min_lightbar_length"].as<double>();
    min_armor_ratio_ = yaml["min_armor_ratio"].as<double>();
    max_armor_ratio_ = yaml["max_armor_ratio"].as<double>();
    max_side_ratio_ = yaml["max_side_ratio"].as<double>();
    min_confidence_ = yaml["min_confidence"].as<double>();
    max_rectangular_error_ = yaml["max_rectangular_error"].as<double>() / 57.3;  // degree to rad

    save_path_ = "patterns";
    std::filesystem::create_directory(save_path_);
  }

  std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int frame_count)
  {
    // 【第 1 步】转灰度：二值化只关心"亮不亮"，不关心颜色，所以先降成单通道。
    // 颜色留到后面 get_color 时再从原始彩色图上取。
    // 彩色图转灰度图
    cv::Mat gray_img;
    cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

    // 【第 2 步】二值化：亮度 > threshold 的像素变白(255)，其余变黑(0)。
    // ★yaml 参数 threshold —— 调识别时第一个该动的旋钮。
    //   调太高：灯条变细甚至断开，远处的板直接找不到；
    //   调太低：地面反光、白墙、灯带全变白块，灯条候选爆炸，误配对随之增多。
    // 曝光 exposure_us 与它耦合，但离线视频是录好的，改曝光无效，所以离线只能调 threshold。
    // 进行二值化
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
    // ★这个窗口没有 debug_ 开关，是**无条件弹出**的（本函数末尾的 show_result 才受 debug_ 控制）。
    // 调 threshold 时正好盯着它看二值化效果。
    cv::imshow("binary_img", binary_img);

    // 【第 3 步】找轮廓：把白色区域的外边界提取成点集。
    // RETR_EXTERNAL 只取最外层（灯条内部的空洞不关心）；
    // CHAIN_APPROX_NONE 保留轮廓上每个点、不做折线近似，后面算最小外接矩形才准。
    // 获取轮廓点
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    // 【第 4 步】每个轮廓套一个最小外接旋转矩形当"灯条候选"，再几何筛选。
    // check_geometry(Lightbar) 用到三个 yaml 参数：
    //   min_lightbar_ratio / max_lightbar_ratio  长宽比（灯条细长）
    //   min_lightbar_length                     最小长度（像素），滤掉远处噪点亮斑
    //   max_angle_error                         允许偏离竖直多少（yaml 写度，读入时 /57.3）
    // 注意顺序：先筛掉不合格的，才去 get_color——取颜色要遍历轮廓内像素，比较贵。
    // 获取灯条
    std::size_t lightbar_id = 0;
    std::list<Lightbar> lightbars;
    for (const auto & contour : contours)
    {
      auto rotated_rect = cv::minAreaRect(contour);
      auto lightbar = Lightbar(rotated_rect, lightbar_id);

      if (!check_geometry(lightbar)) continue;

      lightbar.color = get_color(bgr_img, contour);
      lightbars.emplace_back(lightbar);
      lightbar_id += 1;
    }

    // 按 x 从左到右排序，于是下面配对时 left 只需往右找 right，不会重复配对。
    // 将灯条从左到右排序
    lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

    // 【第 5 步】两两配对成装甲板。注意是 O(n^2) 全组合（left 与其右侧每一根都试一遍），
    // 所以第 2 步阈值放太松、灯条候选一多，这里的开销和误配对会一起涨。
    // 内部顺序是"先几何筛，再认数字"：分类器最贵，能便宜滤掉的先滤掉。
    // 获取装甲板
    std::list<Armor> armors;
    for (auto left = lightbars.begin(); left != lightbars.end(); left++)
    {
      for (auto right = std::next(left); right != lightbars.end(); right++)
      {
        // 只有同色灯条才可能属于同一块板
        if (left->color != right->color) continue;

        // 【第 6 步】几何筛选装甲板。check_geometry(Armor) 用到的 yaml 参数：
        //   min_armor_ratio / max_armor_ratio  两灯条中点连线 / 长灯条长度之比
        //   max_side_ratio                    长短灯条长度比，正对时≈1
        //   max_rectangular_error             灯条与连线夹角偏离 90° 多少（yaml 写度，读入时 /57.3）
        auto armor = Armor(*left, *right);
        if (!check_geometry(armor)) continue;

        // 【第 7 步】认数字：抠出两灯条之间的贴图送分类器。
        // check_name 滤掉 not_armor 以及置信度低于 yaml 参数 min_confidence 的结果。
        armor.pattern = get_pattern(bgr_img, armor);
        classifier_.classify(armor);
        if (!check_name(armor)) continue;

        // 判大板/小板。注意 get_type 里的 ratio 阈值 3.0 / 2.5 是**写死在代码里的，不在 yaml**。
        // 结果决定 PnP 用哪组 3D 物点（solver.cpp:16-25）。
        armor.type = get_type(armor);
        // if (!check_type(armor)) continue;

        armor.center_norm = get_center_norm(bgr_img, armor.center);
        armors.emplace_back(armor);
      }
    }

    // 【第 8 步】去重。因为上面是全组合配对，同一根灯条可能同时进了两块"板"，必须挑一个留下。
    // 下面按两种撞法分别处理，取舍规则不同。
    // 检查装甲板是否存在共用灯条的情况
    for (auto armor1 = armors.begin(); armor1 != armors.end(); armor1++)
    {
      for (auto armor2 = std::next(armor1); armor2 != armors.end(); armor2++)
      {
        if (
          armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id && armor1->right.id != armor2->left.id &&
          armor1->right.id != armor2->right.id)
        {
          continue;
        }

        // 情况一：同侧撞（共用左灯条，或共用右灯条）——"套娃"式重叠，一块板把另一块包在里面。
        // 保留贴图面积小的：面积大的那块通常是把相邻两块板的外侧灯条错配成了一块超宽假板。
        // 装甲板重叠, 保留roi小的
        if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id)
        {
          auto area1 = armor1->pattern.cols * armor1->pattern.rows;
          auto area2 = armor2->pattern.cols * armor2->pattern.rows;
          if (area1 < area2)
            armor2->duplicated = true;
          else
            armor1->duplicated = true;
        }

        // 情况二：首尾相接（A 的右灯条就是 B 的左灯条）——两块板并排挨着，中间那根被两边都算上。
        // 这种没法靠面积区分，改用数字分类置信度：认得更准的那块留下。
        // 装甲板相连，保留置信度大的
        if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id)
        {
          if (armor1->confidence < armor2->confidence)
            armor1->duplicated = true;
          else
            armor2->duplicated = true;
        }
      }
    }

    armors.remove_if([&](const Armor & a) { return a.duplicated; });

    if (debug_) show_result(binary_img, bgr_img, lightbars, armors, frame_count);

    return armors;
  }

  bool Detector::detect(Armor & armor, const cv::Mat & bgr_img)
  {
    // 取得四个角点
    auto tl = armor.points[0];
    auto tr = armor.points[1];
    auto br = armor.points[2];
    auto bl = armor.points[3];
    // 计算向量和调整后的点
    auto lt2b = bl - tl;
    auto rt2b = br - tr;
    auto tl1 = (tl + bl) / 2 - lt2b;
    auto bl1 = (tl + bl) / 2 + lt2b;
    auto br1 = (tr + br) / 2 + rt2b;
    auto tr1 = (tr + br) / 2 - rt2b;
    auto tl2tr = tr1 - tl1;
    auto bl2br = br1 - bl1;
    auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
    auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
    auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
    auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;
    // 构造新的四个角点
    std::vector<cv::Point> points = {tl2, tr2, br2, bl2};
    auto armor_rotaterect = cv::minAreaRect(points);
    cv::Rect boundingBox = armor_rotaterect.boundingRect();
    // 检查boundingBox是否超出图像边界
    if (boundingBox.x < 0 || boundingBox.y < 0 || boundingBox.x + boundingBox.width > bgr_img.cols || boundingBox.y + boundingBox.height > bgr_img.rows)
    {
      return false;
    }

    // 在图像上裁剪出这个矩形区域（ROI）
    cv::Mat armor_roi = bgr_img(boundingBox);
    if (armor_roi.empty())
    {
      return false;
    }

    // 彩色图转灰度图
    cv::Mat gray_img;
    cv::cvtColor(armor_roi, gray_img, cv::COLOR_BGR2GRAY);
    // 进行二值化
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
    // cv::imshow("binary_img", binary_img);
    // 获取轮廓点
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    // 获取灯条
    std::size_t lightbar_id = 0;
    std::list<Lightbar> lightbars;
    for (const auto & contour : contours)
    {
      auto rotated_rect = cv::minAreaRect(contour);
      auto lightbar = Lightbar(rotated_rect, lightbar_id);

      if (!check_geometry(lightbar)) continue;

      lightbar.color = get_color(bgr_img, contour);
      // lightbar_points_corrector(lightbar, gray_img); //关闭PCA
      lightbars.emplace_back(lightbar);
      lightbar_id += 1;
    }

    if (lightbars.size() < 2) return false;

    // 将灯条从左到右排序
    lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

    // 计算与 tl_roi, bl_roi 和 br_roi, tr_roi 距离最近的灯条
    Lightbar * closest_left_lightbar = nullptr;
    Lightbar * closest_right_lightbar = nullptr;
    float min_distance_tl_bl = std::numeric_limits<float>::max();
    float min_distance_br_tr = std::numeric_limits<float>::max();
    for (auto & lightbar : lightbars)
    {
      float distance_tl_bl = cv::norm(tl - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y))) +
                             cv::norm(bl - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y)));
      if (distance_tl_bl < min_distance_tl_bl)
      {
        min_distance_tl_bl = distance_tl_bl;
        closest_left_lightbar = &lightbar;
      }
      float distance_br_tr = cv::norm(br - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y))) +
                             cv::norm(tr - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y)));
      if (distance_br_tr < min_distance_br_tr)
      {
        min_distance_br_tr = distance_br_tr;
        closest_right_lightbar = &lightbar;
      }
    }

    // tools::logger()->debug(
    // "min_distance_br_tr + min_distance_tl_bl is {}", min_distance_br_tr + min_distance_tl_bl);
    // std::vector<cv::Point2f> points2f{
    //   closest_left_lightbar->top, closest_left_lightbar->bottom, closest_right_lightbar->bottom,
    //   closest_right_lightbar->top};
    // tools::draw_points(armor_roi, points2f, {0, 0, 255}, 2);
    // cv::imshow("armor_roi", armor_roi);

    if (closest_left_lightbar && closest_right_lightbar && min_distance_br_tr + min_distance_tl_bl < 15)
    {
      // 将四个点从armor_roi坐标系转换到原始图像坐标系
      armor.points[0] = closest_left_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
      armor.points[1] = closest_right_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
      armor.points[2] = closest_right_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
      armor.points[3] = closest_left_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
      return true;
    }

    return false;
  }

  bool Detector::check_geometry(const Lightbar & lightbar) const
  {
    auto angle_ok = lightbar.angle_error < max_angle_error_;
    auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
    auto length_ok = lightbar.length > min_lightbar_length_;
    return angle_ok && ratio_ok && length_ok;
  }

  bool Detector::check_geometry(const Armor & armor) const
  {
    auto ratio_ok = armor.ratio > min_armor_ratio_ && armor.ratio < max_armor_ratio_;
    auto side_ratio_ok = armor.side_ratio < max_side_ratio_;
    auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_;
    return ratio_ok && side_ratio_ok && rectangular_error_ok;
  }

  bool Detector::check_name(const Armor & armor) const
  {
    auto name_ok = armor.name != ArmorName::not_armor;
    auto confidence_ok = armor.confidence > min_confidence_;

    // 保存不确定的图案，用于分类器的迭代
    if (name_ok && !confidence_ok) save(armor);

    // 出现 5号 则显示 debug 信息。但不过滤。
    if (armor.name == ArmorName::five) tools::logger()->debug("See pattern 5");

    return name_ok && confidence_ok;
  }

  bool Detector::check_type(const Armor & armor) const
  {
    auto name_ok = armor.type == ArmorType::small ? (armor.name != ArmorName::one || armor.name == ArmorName::base)
                                                  : (armor.name == ArmorName::one && armor.name != ArmorName::base);

    // 保存异常的图案，用于分类器的迭代
    if (!name_ok)
    {
      tools::logger()->debug("see strange armor: {} {}", ARMOR_TYPES[armor.type], ARMOR_NAMES[armor.name]);
      save(armor);
    }

    return name_ok;
  }

  Color Detector::get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const
  {
    int red_sum = 0, blue_sum = 0;

    for (const auto & point : contour)
    {
      red_sum += bgr_img.at<cv::Vec3b>(point)[2];
      blue_sum += bgr_img.at<cv::Vec3b>(point)[0];
    }

    return blue_sum > red_sum ? Color::blue : Color::red;
  }

  cv::Mat Detector::get_pattern(const cv::Mat & bgr_img, const Armor & armor) const
  {
    // 延长灯条获得装甲板角点
    // 1.125 = 0.5 * armor_height / lightbar_length = 0.5 * 126mm / 56mm
    auto tl = armor.left.center - armor.left.top2bottom * 1.125;
    auto bl = armor.left.center + armor.left.top2bottom * 1.125;
    auto tr = armor.right.center - armor.right.top2bottom * 1.125;
    auto br = armor.right.center + armor.right.top2bottom * 1.125;

    auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);
    auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);
    auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
    auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);
    auto roi_tl = cv::Point(roi_left, roi_top);
    auto roi_br = cv::Point(roi_right, roi_bottom);
    auto roi = cv::Rect(roi_tl, roi_br);

    return bgr_img(roi);
  }

  ArmorType Detector::get_type(const Armor & armor)
  {
    /// 优先根据当前armor.ratio判断
    /// TODO: 25赛季是否还需要根据比例判断大小装甲？能否根据图案直接判断？

    if (armor.ratio > 3.0)
    {
      // tools::logger()->debug(
      //   "[Detector] get armor type by ratio: BIG {} {:.2f}", ARMOR_NAMES[armor.name], armor.ratio);
      return ArmorType::big;
    }

    if (armor.ratio < 2.5)
    {
      // tools::logger()->debug(
      //   "[Detector] get armor type by ratio: SMALL {} {:.2f}", ARMOR_NAMES[armor.name], armor.ratio);
      return ArmorType::small;
    }

    // tools::logger()->debug("[Detector] get armor type by name: {}", ARMOR_NAMES[armor.name]);

    // 英雄、
    if (armor.name == ArmorName::one)
    {
      return ArmorType::big;
    }
    // 基地只能是小装甲板
    if (armor.name == ArmorName::base)
    {
      return ArmorType::small;
    }

    // 其他所有（工程、哨兵、前哨站、步兵）都是小装甲板
    /// TODO: 基地顶装甲是小装甲板
    return ArmorType::small;
  }

  cv::Point2f Detector::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
  {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return {center.x / w, center.y / h};
  }

  void Detector::save(const Armor & armor) const
  {
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    auto img_path = fmt::format("{}/{}_{}.jpg", save_path_, armor.name, file_name);
    cv::imwrite(img_path, armor.pattern);
  }

  void Detector::show_result(
    const cv::Mat & binary_img, const cv::Mat & bgr_img, const std::list<Lightbar> & lightbars, const std::list<Armor> & armors, int frame_count) const
  {
    auto detection = bgr_img.clone();
    tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

    for (const auto & lightbar : lightbars)
    {
      auto info = fmt::format("{:.1f} {:.1f} {:.1f} {}", lightbar.angle_error * 57.3, lightbar.ratio, lightbar.length, COLORS[lightbar.color]);
      tools::draw_text(detection, info, lightbar.top, {0, 255, 255});
      tools::draw_points(detection, lightbar.points, {0, 255, 255}, 3);
    }

    for (const auto & armor : armors)
    {
      auto info = fmt::format(
        "{:.2f} {:.2f} {:.1f} {:.2f} {} {}", armor.ratio, armor.side_ratio, armor.rectangular_error * 57.3, armor.confidence, ARMOR_NAMES[armor.name],
        ARMOR_TYPES[armor.type]);
      tools::draw_points(detection, armor.points, {0, 255, 0});
      tools::draw_text(detection, info, armor.left.bottom, {0, 255, 0});
    }

    cv::Mat binary_img2;
    cv::resize(binary_img, binary_img2, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::resize(detection, detection, {}, 0.5, 0.5);     // 显示时缩小图片尺寸

    // cv::imshow("threshold", binary_img2);
    cv::imshow("detection", detection);
  }

  void Detector::lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const
  {
    // 配置参数
    constexpr float MAX_BRIGHTNESS = 25;  // 归一化最大亮度值
    constexpr float ROI_SCALE = 0.07;     // ROI扩展比例
    constexpr float SEARCH_START = 0.4;   // 搜索起始位置比例（原0.8/2）
    constexpr float SEARCH_END = 0.6;     // 搜索结束位置比例（原1.2/2）

    // 扩展并裁剪ROI
    cv::Rect roi_box = lightbar.rotated_rect.boundingRect();
    roi_box.x -= roi_box.width * ROI_SCALE;
    roi_box.y -= roi_box.height * ROI_SCALE;
    roi_box.width += 2 * roi_box.width * ROI_SCALE;
    roi_box.height += 2 * roi_box.height * ROI_SCALE;

    // 边界约束
    roi_box &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);

    // 归一化ROI
    cv::Mat roi = gray_img(roi_box);
    const float mean_val = cv::mean(roi)[0];
    roi.convertTo(roi, CV_32F);
    cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

    // 计算质心
    const cv::Moments moments = cv::moments(roi);
    const cv::Point2f centroid(moments.m10 / moments.m00 + roi_box.x, moments.m01 / moments.m00 + roi_box.y);

    // 生成稀疏点云（优化性能）
    std::vector<cv::Point2f> points;
    for (int i = 0; i < roi.rows; ++i)
    {
      for (int j = 0; j < roi.cols; ++j)
      {
        const float weight = roi.at<float>(i, j);
        if (weight > 1e-3)
        {                             // 忽略极小值提升性能
          points.emplace_back(j, i);  // 坐标相对于ROI区域
        }
      }
    }

    // PCA计算对称轴方向
    cv::PCA pca(cv::Mat(points).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::Point2f axis(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));
    axis /= cv::norm(axis);
    if (axis.y > 0) axis = -axis;  // 统一方向

    const auto find_corner = [&](int direction) -> cv::Point2f {
      const float dx = axis.x * direction;
      const float dy = axis.y * direction;
      const float search_length = lightbar.length * (SEARCH_END - SEARCH_START);

      std::vector<cv::Point2f> candidates;

      // 横向采样多个候选线
      const int half_width = (lightbar.width - 2) / 2;
      for (int i_offset = -half_width; i_offset <= half_width; ++i_offset)
      {
        // 计算搜索起点
        cv::Point2f start_point(centroid.x + lightbar.length * SEARCH_START * dx + i_offset, centroid.y + lightbar.length * SEARCH_START * dy);

        // 沿轴搜索亮度跳变点
        cv::Point2f corner = start_point;
        float max_diff = 0;
        bool found = false;

        for (float step = 0; step < search_length; ++step)
        {
          const cv::Point2f cur_point(start_point.x + dx * step, start_point.y + dy * step);

          // 边界检查
          if (cur_point.x < 0 || cur_point.x >= gray_img.cols || cur_point.y < 0 || cur_point.y >= gray_img.rows)
          {
            break;
          }

          // 计算亮度差（使用双线性插值提升精度）
          const auto prev_val = gray_img.at<uchar>(cv::Point2i(cur_point - cv::Point2f(dx, dy)));
          const auto cur_val = gray_img.at<uchar>(cv::Point2i(cur_point));
          const float diff = prev_val - cur_val;

          if (diff > max_diff && prev_val > mean_val)
          {
            max_diff = diff;
            corner = cur_point - cv::Point2f(dx, dy);  // 跳变发生在上一位置
            found = true;
          }
        }

        if (found)
        {
          candidates.push_back(corner);
        }
      }

      // 返回候选点均值
      return candidates.empty() ? cv::Point2f(-1, -1)
                                : std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0)) / static_cast<float>(candidates.size());
    };

    // 并行检测顶部和底部
    lightbar.top = find_corner(1);
    lightbar.bottom = find_corner(-1);
  }

}  // namespace auto_aim