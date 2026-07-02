#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "estimation/target.hpp"
#include "params.hpp"
#include "typedefs.hpp"

namespace ia {
namespace decision {

struct AimPoint {
  bool valid{false};
  Eigen::Vector4d xyza{Eigen::Vector4d::Zero()};
};

class Aimer {
 public:
  explicit Aimer(const RosParams& config);

  AimPoint debug_aim_point;

  /// @brief 瞄准主入口：预测→选点→迭代弹道→输出指令
  Command Aim(const std::list<estimation::Target>& targets,
              std::chrono::steady_clock::time_point timestamp,
              double bullet_speed, bool to_now = true);

  /// @brief 开火决策
  bool Shoot(const Command& command,
             const std::list<estimation::Target>& targets,
             double gimbal_yaw);

 private:
  double yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  double first_tolerance_;
  double second_tolerance_;
  double judge_distance_;
  bool auto_fire_;
  double lock_id_{-1};
  Command last_command_{false, false, 0, 0};

  AimPoint ChooseAimPoint(const estimation::Target& target);
};

}  // namespace decision
}  // namespace ia
