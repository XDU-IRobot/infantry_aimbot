
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "typedefs.hpp"

namespace ia {
namespace detection {
inline void Print(const geometry_msgs::msg::Pose &pose) {
  std::printf("pos: %f %f %f | orit: %f %f %f %f\n", pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

inline auto DrawArmorMarker(const Armor &armor) -> sp<visualization_msgs::msg::MarkerArray> {
  auto ret = std::make_shared<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose = armor.pose;

  marker.scale.x = 0.012f;
  marker.scale.y = 0.1f;
  marker.scale.z = 0.05f;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;

  ret->markers.push_back(marker);
  return ret;
}
inline auto DrawArmorMarker(const std::vector<Armor> &armors) -> sp<visualization_msgs::msg::MarkerArray> {
  auto ret = std::make_shared<visualization_msgs::msg::MarkerArray>();
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.scale.x = 0.012f;
  marker.scale.y = 0.1f;
  marker.scale.z = 0.05f;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.8f;

  ret->markers.reserve(armors.size());
  for (const auto &armor : armors) {
    marker.pose = armor.pose;
    ret->markers.push_back(marker);
  }

  return ret;
}
}  // namespace detection
}  // namespace ia