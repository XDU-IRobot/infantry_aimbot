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
#include "daheng_cam/daheng.hpp"
#include "estimation/armor_matcher.hpp"
#include "estimation/ekf_car_tracker.hpp"
#include "estimation/data_buffer.hpp"

auto GetCamera(const RosParams &ros_params) {
  auto cam = std::make_shared<camera::DahengCam>();
  cam->set_parameter(camera::CamParamType::Width, ros_params.camera_settings.width);
  cam->set_parameter(camera::CamParamType::Height, ros_params.camera_settings.height);
  cam->set_parameter(camera::CamParamType::Exposure, ros_params.camera_settings.exposure);
  cam->set_parameter(camera::CamParamType::AutoExposure, ros_params.camera_settings.auto_exposure);
  cam->set_parameter(camera::CamParamType::Gain, ros_params.camera_settings.gain);
  cam->set_parameter(camera::CamParamType::Gamma, ros_params.camera_settings.gain);
  cam->set_parameter(camera::CamParamType::Fps, ros_params.camera_settings.fps);
  cam->set_parameter(camera::CamParamType::AutoWhiteBalance, ros_params.camera_settings.auto_white_balance);
  cam->set_parameter(camera::CamParamType::RGain, ros_params.camera_settings.rgain);
  cam->set_parameter(camera::CamParamType::GGain, ros_params.camera_settings.ggain);
  cam->set_parameter(camera::CamParamType::BGain, ros_params.camera_settings.bgain);
  return cam;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto ros_node = ia::ros::NodeSingleton::GetInstance().node();
  auto logger = ros_node->get_logger();
  ia::ros::PublisherPool<sensor_msgs::msg::Image> image_publisher_pool{ros_node};
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
  cv::VideoCapture cap(package_path + "/assets/test.mp4");
  auto cam = GetCamera(config);
  cam->open();

  cv_bridge::CvImage frame;
  frame.encoding = sensor_msgs::image_encodings::BGR8;
  while (cap.isOpened() && rclcpp::ok()) {
    // cap >> frame.image;
    cam->grab_image(frame.image);
    if (frame.image.empty()) {
      break;
    }

    const auto start_time = std::chrono::high_resolution_clock::now();
    const auto armors = detection_pipeline.ProcessImage(frame.image);
    RCLCPP_DEBUG(
        ros_node->get_logger(), "Detection took %ld us",
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time)
            .count());

    if (config.debug) {
      if (armors) {
        ia::detection::DrawArmor(frame.image, *armors.value());

        marker_array_pub_pool.Publish("armors", ia::detection::DrawArmorToRviz(*armors.value()));
      }
      // 选择其中一种方式进行调试可视化
      image_publisher_pool.Publish("debug_image", frame.toImageMsg());
      // cv::imshow("debug", frame.image);
      // cv::waitKey(1);
    }
  }
  rclcpp::shutdown();
  return 0;
}

#include <backward.hpp>
backward::SignalHandling _;  ///< 程序崩掉时打印stack trace