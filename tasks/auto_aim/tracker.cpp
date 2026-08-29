#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <numeric>
#include <tuple>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
  Tracker::Tracker(const std::string & config_path, Solver * solver)
  : solver_{solver}, detect_count_(0), temp_lost_count_(0), state_{"lost"}, last_timestamp_(std::chrono::steady_clock::now())
  {
    auto yaml = YAML::LoadFile(config_path);
    enemy_color_str_ = yaml["enemy_color"].as<std::string>();
    enemy_color_ = (enemy_color_str_ == "red") ? Color::red : Color::blue;
    min_detect_count_ = yaml["min_detect_count"].as<int>();
    max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
    outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
    normal_temp_lost_count_ = max_temp_lost_count_;

    last_cam_is_short = true;
  }

  std::string Tracker::state() const { return state_; }

  // ============================================================================
  // 【三个 track 方法的区别 —— 本文件最容易看晕的地方】
  //
  // 三者主体流程几乎一样（滤颜色 -> 排序 -> set/update -> 状态机 -> 健康检查），
  // 但适用场合不同，**只有一个能离线用**：
  //
  //   sb_track  (:49)  哨兵专用。**唯一真正按 ArmorPriority 排序**的（:90）。
  //                     开头就解引用 gimbal_（:55-60），离线不可用。
  //   track     (:126)  上场步兵用。多一段"按右键切换/锁定目标"的逻辑。
  //                     开头同样要 gimbal_（:130-135），离线不可用。
  //   test_track(:232)  离线测试用。★**完全不碰 gimbal_**，所以笔记本上跑的就是它；
  //                     代价是拿不到下位机状态，enemy_color 只能用 yaml 里写死的值。
  //
  // 另外两处差异容易踩坑：
  //   1. 按优先级排序那行只有 sb_track 是活的，track(:165-166) 和 test_track(:264-265) 都被注释掉了。
  //      加上 Armor::priority 从未被赋值（见 armor.hpp），所以"优先级"目前形同虚设。
  //   2. enemy_color 为 "auto" 时的红蓝映射，sb_track(:61) 与 track(:136) **是相反的**：
  //      一处 (g.enemy_color==0)?blue:red，另一处 ?red:blue，其中至少有一个是错的。
  //      但这要接 C 板才能验证，离线看不出来，故只记录、不动手改。
  // ============================================================================
  std::list<Target> Tracker::sb_track(std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool cam_is_short, bool use_enemy_color)
  {
    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;

    // TODO
    if (gimbal_ == nullptr)
    {
      tools::logger()->error("[Tracker] gimbal_不能为空指针，请先调用set_gimbal()设置云台指针");
      return {};
    }
    io::GimbalState g = gimbal_->state();
    if (enemy_color_str_ == "auto") enemy_color_ = (g.enemy_color == 0) ? Color::blue : Color::red;

    target_.cam_is_short = cam_is_short;

    // 时间间隔过长，说明可能发生了相机离线
    if (state_ != "lost" && dt > 0.1)
    {
      tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
      state_ = "lost";
    }
    // 过滤掉非我方装甲板
    armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

    // 过滤前哨站顶部装甲板
    // armors.remove_if([this](const auto_aim::Armor & a) {
    //   return a.name == ArmorName::outpost &&
    //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
    //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
    // });

    // 优先选择靠近图像中心的装甲板
    armors.sort([](const Armor & a, const Armor & b) {
      cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
      auto distance_1 = cv::norm(a.center - img_center);
      auto distance_2 = cv::norm(b.center - img_center);
      return distance_1 < distance_2;
    });

    // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
    armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

    bool found;
    if (state_ == "lost")
    {
      found = set_target(armors, t);
    }

    else
    {
      found = update_target(armors, t);
    }

    state_machine(found);

    // 发散检测
    if (state_ != "lost" && target_.diverged())
    {
      tools::logger()->debug("[Tracker] Target diverged!");
      state_ = "lost";
      return {};
    }

    if (std::accumulate(target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >= (0.4 * target_.ekf().window_size))
    {
      tools::logger()->debug("[Target] Bad Converge Found!");
      state_ = "lost";
      return {};
    }

    if (state_ == "lost") return {};

    std::list<Target> targets = {target_};
    return targets;
  }

  std::list<Target> Tracker::track(std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool cam_is_short, bool use_enemy_color)
  {
    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;
    if (gimbal_ == nullptr)
    {
      tools::logger()->error("[Tracker] gimbal_不能为空指针，请先调用set_gimbal()设置云台指针");
      return {};
    }
    io::GimbalState g = gimbal_->state();
    if (enemy_color_str_ == "auto") enemy_color_ = (g.enemy_color == 0) ? Color::red : Color::blue;

    target_.cam_is_short = cam_is_short;

    // 时间间隔过长，说明可能发生了相机离线
    if (state_ != "lost" && dt > 0.1)
    {
      tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
      state_ = "lost";
    }
    // 过滤掉非我方装甲板
    armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

    // 过滤前哨站顶部装甲板
    // armors.remove_if([this](const auto_aim::Armor & a) {
    //   return a.name == ArmorName::outpost &&
    //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
    //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
    // });

    // 优先选择靠近图像中心的装甲板
    armors.sort([](const Armor & a, const Armor & b) {
      cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
      auto distance_1 = cv::norm(a.center - img_center);
      auto distance_2 = cv::norm(b.center - img_center);
      return distance_1 < distance_2;
    });

    // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
    // armors.sort(
    //   [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

    bool found = 0;

    static uint8_t last_mode = g.mode;
    bool mode_switch_0to1 = (last_mode == 0 && g.mode == 1);
    //按下右键时，mouse为1则跟随上一次的目标，不按则瞄准最近的装甲板
    if (!mode_switch_0to1)
    {
      if (state_ == "lost")
      {
        found = set_target(armors, t);
        // tools::logger()->debug("按下右键，只选择正在跟踪的装甲板，跳过其他兵种，直至丢跟踪，初始化跟踪类型为 {}", ARMOR_NAMES[armors.front().name]);
      }
      else
      {
        found = update_target(armors, t);
      }
    }
    else
    {
      if (state_ == "lost")
      {
        found = set_target(armors, t);
      }
      else
      {
        if (target_.name == armors.front().name && target_.armor_type == armors.front().type)
        {
          found = update_target(armors, t);
        }
        else
        {
          found = set_target(armors, t);
          state_ = "detecting";
          detect_count_ = 1;
          // tools::logger()->debug("不按右键，默认选中离图像中心最近的兵种，切换至： {}, 跟踪器重置, 置信度:{:.3f}", ARMOR_NAMES[armors.front().name],armors.front().confidence );
        }
      }
    }
    last_mode = g.mode;
    // found = set_target(armors, t);

    state_machine(found);

    // 发散检测
    if (state_ != "lost" && target_.diverged())
    {
      // tools::logger()->debug("[Tracker] Target diverged!");
      state_ = "lost";
      return {};
    }

    if (std::accumulate(target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >= (0.4 * target_.ekf().window_size))
    {
      tools::logger()->debug("[Target] Bad Converge Found!");
      state_ = "lost";
      return {};
    }

    if (state_ == "lost") return {};

    std::list<Target> targets = {target_};
    return targets;
  }

  std::list<Target> Tracker::test_track(std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool cam_is_short, bool use_enemy_color)
  {
    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;

    target_.cam_is_short = cam_is_short;

    // 时间间隔过长，说明可能发生了相机离线
    if (state_ != "lost" && dt > 0.1)
    {
      tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
      state_ = "lost";
    }
    // 过滤掉非我方装甲板
    armors.remove_if([&](const auto_aim::Armor & a) { return a.color != enemy_color_; });

    // 过滤前哨站顶部装甲板
    // armors.remove_if([this](const auto_aim::Armor & a) {
    //   return a.name == ArmorName::outpost &&
    //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
    //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
    // });

    // 优先选择靠近图像中心的装甲板
    armors.sort([](const Armor & a, const Armor & b) {
      cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO
      auto distance_1 = cv::norm(a.center - img_center);
      auto distance_2 = cv::norm(b.center - img_center);
      return distance_1 < distance_2;
    });

    // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
    // armors.sort(
    //   [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

    bool found = 0;

    if (state_ == "lost")
    {
      found = set_target(armors, t);
      // tools::logger()->debug("按下右键，只选择正在跟踪的装甲板，跳过其他兵种，直至丢跟踪，初始化跟踪类型为 {}", ARMOR_NAMES[armors.front().name]);
    }
    else
    {
      found = update_target(armors, t);
    }

    // found = set_target(armors, t);

    state_machine(found);

    // 发散检测
    if (state_ != "lost" && target_.diverged())
    {
      // tools::logger()->debug("[Tracker] Target diverged!");
      state_ = "lost";
      return {};
    }

    if (std::accumulate(target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >= (0.4 * target_.ekf().window_size))
    {
      tools::logger()->debug("[Target] Bad Converge Found!");
      state_ = "lost";
      return {};
    }

    if (state_ == "lost") return {};

    std::list<Target> targets = {target_};
    return targets;
  }

  // ============================================================================
  // 跟踪状态机：决定"这一帧到底算不算跟上了目标"。
  // 这是全仓库最像电控代码的一段——一个四态机，和你写过的按键消抖/状态切换一个套路。
  //
  //   lost       完全没目标。见到一块板就进 detecting。
  //   detecting  疑似发现，还在攒证据。连续 found 累计到 min_detect_count 才升级 tracking；
  //              中间断一帧就立刻掉回 lost（要求连续，不容忍抖动）。
  //   tracking   稳定跟踪中。只有 tracking 和 temp_lost 会把 Target 交给上层去打。
  //   temp_lost  短暂丢失：被己方车挡住、目标转到背面、识别抖了一帧，都属于这类。
  //              这期间 EKF 继续靠预测往前推，不喂新观测；
  //              连续丢超过 max_temp_lost_count 帧才判定真丢，回到 lost。
  //
  // 两个 yaml 参数的取舍（离线就能调，改完直接看效果）：
  //   min_detect_count     调大：不容易被误识别骗到，但发现目标慢半拍；
  //                        调小：反应快，但可能对着反光的白墙就开始"跟踪"。
  //   max_temp_lost_count  调大：目标转背面/被短暂遮挡时不丢跟踪，小陀螺跟得更连贯，
  //                        但目标真跑了以后还会对着空气预测一阵；
  //                        调小：丢得快，容易在小陀螺时反复重新初始化，EKF 一直收敛不了。
  // 前哨站单独用一套更大的 max_temp_lost_count（:364-369），因为它转速固定且会长时间只露侧面。
  // ============================================================================
  void Tracker::state_machine(bool found)
  {
    if (state_ == "lost")
    {
      if (!found) return;

      state_ = "detecting";
      detect_count_ = 1;
    }

    else if (state_ == "detecting")
    {
      if (found)
      {
        detect_count_++;
        if (detect_count_ >= min_detect_count_) state_ = "tracking";
      }
      else
      {
        detect_count_ = 0;
        state_ = "lost";
      }
    }

    else if (state_ == "tracking")
    {
      if (found) return;

      temp_lost_count_ = 1;
      state_ = "temp_lost";
    }

    else if (state_ == "temp_lost")
    {
      if (found)
      {
        state_ = "tracking";
      }
      else
      {
        temp_lost_count_++;
        if (target_.name == ArmorName::outpost)
          //前哨站的temp_lost_count需要设置的大一些
          max_temp_lost_count_ = outpost_max_temp_lost_count_;
        else
          max_temp_lost_count_ = normal_temp_lost_count_;

        if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
      }
    }
  }

  // 新建跟踪目标（从 lost 起手时调用）。取排序后的第一块板（离图像中心最近的那块），
  // 解算它，然后按兵种构造 Target。
  //
  // 【构造参数的含义 —— 理解各兵种差异的关键】Target(armor, t, r, armor_num, P0_dig)：
  //   r         旋转中心到装甲板的初始半径【米】。普通车 0.2、前哨站 0.2765、基地 0.3205。
  //             这是"车有多大"的先验，给错了 EKF 要花很多帧才修正回来。
  //   armor_num 该目标有几块装甲板：普通车 4、平衡步兵 2、前哨站/基地 3。
  //             它决定 h_armor_xyz 里 id*2π/armor_num 的角度间隔，是几何模型的骨架。
  //   P0_dig    EKF 初始协方差的对角线，即"一开始各维有多不确定"，11 维、顺序同状态向量。
  //             注意 vyaw 那一维给到 100（很不确定）——刚看到车时完全不知道它转不转；
  //             而前哨站/基地把 r 给 1e-4、r_/z_ 给 0，意思是"这几维我很确定，别乱改"，
  //             因为前哨站尺寸是固定已知的，不需要滤波器去估。
  // ★这些 P0 值写死在代码里、不在 yaml，改了必须重新编译。
  bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
  {
    if (armors.empty()) return false;

    auto & armor = armors.front();
    solver_->solve(armor);

    // 根据兵种优化初始化参数
    auto is_balance = (armor.type == ArmorType::big) && (armor.name == ArmorName::three || armor.name == ArmorName::four || armor.name == ArmorName::five);

    if (is_balance)
    {
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
      target_ = Target(armor, t, 0.2, 2, P0_dig);
    }

    else if (armor.name == ArmorName::outpost)
    {
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
      target_ = Target(armor, t, 0.2765, 3, P0_dig);
    }

    else if (armor.name == ArmorName::base)
    {
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
      target_ = Target(armor, t, 0.3205, 3, P0_dig);
    }

    else
    {
      Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
      target_ = Target(armor, t, 0.2, 4, P0_dig);
    }

    return true;
  }
  // 更新已有目标（跟踪中每帧调用）。这就是卡尔曼滤波"预测-更新"的完整一轮：
  //   1. predict(t) 先按运动模型把状态推进到当前时刻（无论这帧有没有看到板，都要推）；
  //   2. 在本帧识别结果里找一块"与正在跟的目标同兵种、同板型"的板，找到就 update 喂进 EKF。
  //
  // 匹配只看 name + type，不看位置——位置的合理性交由 Target::update 内部的马氏距离卡方门限
  // 把关（见 target.cpp），是双层筛选。
  // 因为 armors 已按"离图像中心近"排序，所以第一个匹配上的就是视野最正、畸变最小的那块，
  // 拿到即 break。返回 false（没找到）会让状态机走向 temp_lost。
  bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
  {
    target_.predict(t);

    bool found = false;

    // 由于 armors 在 track/sb_track 中已经按距离图像中心的远近排序
    // 遍历找到的第一个匹配目标的装甲板，即为视野中最居中、畸变最小的装甲板
    for (auto & armor : armors)
    {
      if (armor.name == target_.name && armor.type == target_.armor_type)
      {
        solver_->solve(armor);
        target_.update(armor);
        found = true;
        break;  // 找到最优匹配后立即退出
      }
    }

    return found;
  }

}  // namespace auto_aim