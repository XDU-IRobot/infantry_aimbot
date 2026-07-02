#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "detection/armor_solver.hpp"
#include "estimation/target.hpp"
#include "params.hpp"
#include "typedefs.hpp"

namespace ia {
namespace estimation {

/// @brief 目标追踪状态机：lost → detecting → tracking → temp_lost → lost
class Tracker {
 public:
  Tracker(const RosParams& config, detection::ArmorSolver& solver);

  std::string State() const;

  /// @brief 主追踪入口：过滤→排序→状态机→发散/收敛检测→返回目标列表
  std::list<Target> Track(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);

 private:
  detection::ArmorSolver& solver_;
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int detect_count_{0};
  int temp_lost_count_{0};
  std::string state_{"lost"};
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;

  void StateMachine(bool found);
  bool SetTarget(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);
  bool UpdateTarget(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);
};

}  // namespace estimation
}  // namespace ia
