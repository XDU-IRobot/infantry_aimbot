#include "estimation/tracker.hpp"

#include <numeric>

#include "tools/math_tools.hpp"

namespace ia {
namespace estimation {

Tracker::Tracker(const RosParams& config, detection::ArmorSolver& solver)
    : solver_{solver}, last_timestamp_(std::chrono::steady_clock::now()) {
  enemy_color_ = config.detector.enemy_color == 0 ? Color::RED : Color::BLUE;
  min_detect_count_ = config.tracker.min_detect_count;
  max_temp_lost_count_ = config.tracker.max_temp_lost_count;
  outpost_max_temp_lost_count_ = config.tracker.outpost_max_temp_lost_count;
}

std::string Tracker::State() const { return state_; }

std::list<Target> Tracker::Track(std::list<Armor>& armors, std::chrono::steady_clock::time_point t) {
  double dt = tools::DeltaTime(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，可能相机离线
  if (state_ != "lost" && dt > 0.1) {
    state_ = "lost";
  }

  // 过滤非敌方装甲板
  armors.remove_if([&](const Armor& a) { return a.left_light.color != enemy_color_; });

  // 按图像中心距离排序
  armors.sort([](const Armor& a, const Armor& b) {
    auto distance_a = a.distance_to_image_center;
    auto distance_b = b.distance_to_image_center;
    return distance_a < distance_b;
  });

  // 按优先级排序（数字越小优先级越高）
  armors.sort(
      [](const Armor& a, const Armor& b) { return static_cast<int>(a.priority) < static_cast<int>(b.priority); });

  bool found;
  if (state_ == "lost") {
    found = SetTarget(armors, t);
  } else {
    found = UpdateTarget(armors, t);
  }

  StateMachine(found);

  // 发散检测
  if (state_ != "lost" && target_.Diverged()) {
    state_ = "lost";
    return {};
  }

  // NIS收敛检测
  if (state_ != "lost" && !target_.Ekf().recent_nis_failures.empty()) {
    int failures =
        std::accumulate(target_.Ekf().recent_nis_failures.begin(), target_.Ekf().recent_nis_failures.end(), 0);
    if (failures >= static_cast<int>(0.4 * target_.Ekf().window_size)) {
      state_ = "lost";
      return {};
    }
  }

  if (state_ == "lost") return {};
  return {target_};
}

void Tracker::StateMachine(bool found) {
  if (state_ == "lost") {
    if (!found) return;
    state_ = "detecting";
    detect_count_ = 1;
  } else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  } else if (state_ == "tracking") {
    if (found) return;
    temp_lost_count_ = 1;
    state_ = "temp_lost";
  } else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      int max_count = (target_.name == ArmorName::kOutpost) ? outpost_max_temp_lost_count_ : max_temp_lost_count_;
      if (temp_lost_count_ > max_count) state_ = "lost";
    }
  }
}

bool Tracker::SetTarget(std::list<Armor>& armors, std::chrono::steady_clock::time_point t) {
  if (armors.empty()) return false;

  auto& armor = armors.front();
  solver_.Solve(armor);

  // 根据兵种优化初始化参数
  bool is_balance =
      (armor.kind == ArmorKind::kBig) &&
      (armor.name == ArmorName::kThree || armor.name == ArmorName::kFour || armor.name == ArmorName::kFive);

  if (is_balance) {
    Eigen::VectorXd P0_diag{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_diag);
  } else if (armor.name == ArmorName::kOutpost) {
    Eigen::VectorXd P0_diag{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_diag);
  } else if (armor.name == ArmorName::kBase) {
    Eigen::VectorXd P0_diag{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_diag);
  } else {
    Eigen::VectorXd P0_diag{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_diag);
  }

  return true;
}

bool Tracker::UpdateTarget(std::list<Armor>& armors, std::chrono::steady_clock::time_point t) {
  target_.Predict(t);

  int found_count = 0;
  for (const auto& armor : armors) {
    if (armor.name != target_.name || armor.kind != target_.armor_kind) continue;
    found_count++;
  }

  if (found_count == 0) return false;

  for (auto& armor : armors) {
    if (armor.name != target_.name || armor.kind != target_.armor_kind) continue;

    solver_.Solve(armor);
    target_.Update(armor);
  }

  return true;
}

}  // namespace estimation
}  // namespace ia
