#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "io/gimbal/gimbal.hpp"
#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver * solver);

  std::string state() const;

  std::list<Target> sb_track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool cam_is_short = true,
    bool use_enemy_color = true);

  std::list<Target> track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, 
    bool cam_is_short = true,
    bool use_enemy_color = true);

  std::list<Target> test_track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t, 
    bool cam_is_short = true,
    bool use_enemy_color = true);


  inline void setSolver(Solver * solver__){this->solver_ = solver__; }
  void set_gimbal(io::Gimbal* gimbal) { gimbal_ = gimbal; }
  inline size_t get_update_count(){return this->target_.update_count_;}
private:
  Solver * solver_;
  io::Gimbal* gimbal_ = nullptr; // 新增一个云台指针，默认为空
  Color enemy_color_;
  std::string enemy_color_str_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  bool cam_is_switch = false, last_cam_is_short = true;

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP