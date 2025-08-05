
// std
#include <chrono>

// ros
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// third-party
#include <opencv2/opencv.hpp>

// project
#include "params.hpp"
#include "ros/node.hpp"
#include "ros/params_manager.hpp"
#include "ros/publisher_pool.hpp"
#include "detector/traditional_detector.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ros_node = ia::ros::NodeSingleton::GetInstance().node();
  auto logger = ros_node->get_logger();
  ia::ros::PublisherPool<sensor_msgs::msg::CompressedImage> image_publisher_pool{ros_node};
  ia::ros::ParamsManager<Params> params_manager(ros_node);
  const auto &params = params_manager.data();
  params_manager.Init();
  if (params.debug) {
    logger.set_level(rclcpp::Logger::Level::Debug);
  } else {
    image_publisher_pool.Bypass(true);
    logger.set_level(rclcpp::Logger::Level::Info);
  }

  ia::detector::TraditionalDetector det{
      {params.bin_threshold, params.enemy_color},
      {params.angle_to_vertigal_max, params.height_width_min_ratio, params.size_area_min_ratio,
       params.is_corner_correct},
      {params.lights_angle_max_diff, params.lights_length_max_ratio, params.lights_y_max_ratio,
       params.width_height_min_ratio, params.width_height_max_ratio, params.max_angle, params.inside_thresh}};

  const auto package_path = ament_index_cpp::get_package_share_directory("infantry_aimbot");
  cv::VideoCapture cap(package_path + "/assets/test1.mp4");

  cv_bridge::CvImage frame;
  frame.encoding = sensor_msgs::image_encodings::BGR8;
  while (cap.isOpened() && rclcpp::ok()) {
    cap >> frame.image;
    if (frame.image.empty()) {
      break;
    }

    const auto start_time = std::chrono::high_resolution_clock::now();
    auto armors = det.DetectArmors(frame.image);
    RCLCPP_DEBUG(
        ros_node->get_logger(), "Detection took %ld us",
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time)
            .count());

    if (armors) {
      ia::detector::TraditionalDetector::DrawResult(frame.image, *armors.value());
    }

    image_publisher_pool.Publish("debug_image", frame.toCompressedImageMsg());
  }
  rclcpp::shutdown();
  return 0;
}

#include <backward.hpp>
backward::SignalHandling _;  ///< 程序崩掉时打印stack trace