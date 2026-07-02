#pragma once

namespace ia {
namespace tools {

struct Trajectory {
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  /// @brief 不考虑空气阻力的弹道解算
  /// @param v0 子弹初速度大小，单位：m/s
  /// @param d 目标水平距离，单位：m
  /// @param h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);
};

}  // namespace tools
}  // namespace ia
