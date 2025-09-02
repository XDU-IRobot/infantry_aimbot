
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
#include "detection/pipeline.hpp"
#include "detection/visualization.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ros_node = ia::ros::NodeSingleton::GetInstance().node();
  auto logger = ros_node->get_logger();
  ia::ros::PublisherPool<sensor_msgs::msg::CompressedImage> image_publisher_pool{ros_node};
  ia::ros::PublisherPool<visualization_msgs::msg::MarkerArray> marker_array_pub_pool{ros_node};
  ia::ros::ParamsManager<RosParams> ros_params_manager(ros_node);
  const auto &config = ros_params_manager.data();
  ros_params_manager.Init();
  if (config.debug) {
    logger.set_level(rclcpp::Logger::Level::Debug);
  } else {
    image_publisher_pool.Bypass(true);
    logger.set_level(rclcpp::Logger::Level::Info);
  }

  ia::detection::DetectionPipeline detection_pipeline{config};

  const auto package_path = ament_index_cpp::get_package_share_directory("infantry_aimbot");
  cv::VideoCapture cap(package_path + "/assets/test3.mp4");

  cv_bridge::CvImage frame;
  frame.encoding = sensor_msgs::image_encodings::BGR8;
  while (cap.isOpened() && rclcpp::ok()) {
    cap >> frame.image;
    if (frame.image.empty()) {
      break;
    }

    const auto start_time = std::chrono::high_resolution_clock::now();
    const auto armors = detection_pipeline.ProcessImage(frame.image);
    RCLCPP_DEBUG(
        ros_node->get_logger(), "Detection took %ld us",
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time)
            .count());

    if (armors && config.debug) {
      ia::detection::DrawArmor(frame.image, *armors.value());
      image_publisher_pool.Publish("debug_image", frame.toCompressedImageMsg());
      marker_array_pub_pool.Publish("armors", ia::detection::DrawArmorToRviz(*armors.value()));
    }
  }
  rclcpp::shutdown();
  return 0;
}

#include <backward.hpp>
backward::SignalHandling _;  ///< 程序崩掉时打印stack trace