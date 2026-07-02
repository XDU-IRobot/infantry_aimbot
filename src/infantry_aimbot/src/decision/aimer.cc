#include "decision/aimer.hpp"

#include <cmath>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace ia {
namespace decision {

Aimer::Aimer(const RosParams& config)
    : lock_id_{-1}, last_command_{false, false, 0, 0} {
  yaw_offset_ = config.aimer.yaw_offset / 57.3;
  pitch_offset_ = config.aimer.pitch_offset / 57.3;
  comming_angle_ = config.aimer.comming_angle / 57.3;
  leaving_angle_ = config.aimer.leaving_angle / 57.3;
  high_speed_delay_time_ = config.aimer.high_speed_delay_time;
  low_speed_delay_time_ = config.aimer.low_speed_delay_time;
  decision_speed_ = config.aimer.decision_speed;
  first_tolerance_ = config.shooter.first_tolerance / 57.3;
  second_tolerance_ = config.shooter.second_tolerance / 57.3;
  judge_distance_ = config.shooter.judge_distance;
  auto_fire_ = config.shooter.auto_fire;
}

Command Aimer::Aim(const std::list<estimation::Target>& targets,
                    std::chrono::steady_clock::time_point timestamp,
                    double bullet_speed, bool to_now) {
  if (targets.empty()) return {false, false, 0, 0};

  auto target = targets.front();

  double delay_time = std::abs(target.EkfX()[7]) > decision_speed_
                          ? high_speed_delay_time_
                          : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = 23;

  auto future = timestamp;
  if (to_now) {
    double dt =
        tools::DeltaTime(std::chrono::steady_clock::now(), timestamp) +
        delay_time;
    future += std::chrono::microseconds(static_cast<int>(dt * 1e6));
    target.Predict(future);
  } else {
    double dt = 0.005 + delay_time;  // detector-aimer耗时0.005s
    future += std::chrono::microseconds(static_cast<int>(dt * 1e6));
    target.Predict(future);
  }

  auto aim_point0 = ChooseAimPoint(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  double d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间（最多10次，收敛条件：相邻两次fly_time差 < 0.001）
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<estimation::Target> iteration_target(10, target);

  for (int iter = 0; iter < 10; ++iter) {
    auto predict_time =
        future +
        std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].Predict(predict_time);

    auto aim_point = ChooseAimPoint(iteration_target[iter]);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    if (current_traj.unsolvable) {
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  if (!converged && std::abs(current_traj.fly_time - prev_fly_time) >= 0.01) {
    return {false, false, 0, 0};
  }

  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);
  return {true, false, yaw, pitch};
}

AimPoint Aimer::ChooseAimPoint(const estimation::Target& target) {
  Eigen::VectorXd ekf_x = target.EkfX();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.ArmorXyzaList();
  int armor_num = static_cast<int>(armor_xyza_list.size());

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    double delta_angle =
        tools::LimitRad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺：在可射击范围内选择装甲板
  if (std::abs(target.EkfX()[8]) <= 2 &&
      target.name != ArmorName::kOutpost) {
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }

    if (id_list.empty()) return {false, armor_xyza_list[0]};

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) <
                    std::abs(delta_angle_list[id1]))
                       ? id0
                       : id1;
      return {true, armor_xyza_list[static_cast<int>(lock_id_)]};
    }

    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  // 小陀螺模式：根据旋转方向选择coming/leaving侧的装甲板
  double coming_angle, leaving_angle;
  if (target.name == ArmorName::kOutpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle)
      return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle)
      return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

bool Aimer::Shoot(const Command& command,
                   const std::list<estimation::Target>& targets,
                   double gimbal_yaw) {
  if (!command.control || targets.empty() || !auto_fire_) return false;

  double target_x = targets.front().EkfX()[0];
  double target_y = targets.front().EkfX()[2];
  double tolerance =
      std::sqrt(tools::Square(target_x) + tools::Square(target_y)) >
              judge_distance_
          ? second_tolerance_
          : first_tolerance_;

  if (std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&
      std::abs(gimbal_yaw - last_command_.yaw) < tolerance &&
      debug_aim_point.valid) {
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace decision
}  // namespace ia
