/// @file main.cc
/// @brief 步兵自瞄主程序 — 大恒相机 → 检测 → 坐标变换 → 追踪 → 瞄准 → ROS可视化

#include <chrono>
#include <mutex>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "daheng_cam/daheng.hpp"
#ifdef HAS_HIKROBOT_CAM
#include "hikrobot_cam/hikrobot_cam.hpp"
#endif
#include "decision/aimer.hpp"
#include "detection/armor_solver.hpp"
#include "detection/pipeline.hpp"
#include "detection/visualization.hpp"
#include "estimation/tracker.hpp"
#include "params.hpp"
#include "ros/node.hpp"
#include "ros/params_manager.hpp"
#include "ros/publisher_pool.hpp"
#include "tools/math_tools.hpp"

namespace {

ia::ArmorName NumIdToArmorName(int num_id) {
  switch (num_id) {
    case 0:
    case 6:
      return ia::ArmorName::kSentry;
    case 1:
      return ia::ArmorName::kOne;
    case 2:
      return ia::ArmorName::kTwo;
    case 3:
      return ia::ArmorName::kThree;
    case 4:
      return ia::ArmorName::kFour;
    case 5:
      return ia::ArmorName::kFive;
    case 7:
      return ia::ArmorName::kOutpost;
    case 8:
      return ia::ArmorName::kBase;
    default:
      return ia::ArmorName::kNotArmor;
  }
}

ia::ArmorPriority ArmorNameToPriority(ia::ArmorName name) {
  switch (name) {
    case ia::ArmorName::kOne:
      return ia::ArmorPriority::kFirst;
    case ia::ArmorName::kTwo:
      return ia::ArmorPriority::kSecond;
    case ia::ArmorName::kThree:
      return ia::ArmorPriority::kThird;
    case ia::ArmorName::kFour:
      return ia::ArmorPriority::kFourth;
    default:
      return ia::ArmorPriority::kFifth;
  }
}

ia::ArmorKind TypeToArmorKind(ia::Armor::Type type) {
  return (type == ia::Armor::BIG || type == ia::Armor::GRAY_BIG_ARMOR)
             ? ia::ArmorKind::kBig
             : ia::ArmorKind::kSmall;
}

/// 初始化大恒相机
std::shared_ptr<camera::DahengCam> InitCamera(const RosParams& config) {
  auto cam = std::make_shared<camera::DahengCam>();
  const auto& s = config.camera_settings;

  cam->set_parameter(camera::CamParamType::Width, s.width);
  cam->set_parameter(camera::CamParamType::Height, s.height);
  cam->set_parameter(camera::CamParamType::Exposure, s.exposure);
  cam->set_parameter(camera::CamParamType::AutoExposure, s.auto_exposure);
  cam->set_parameter(camera::CamParamType::Gain, s.gain);
  cam->set_parameter(camera::CamParamType::Fps, s.fps);
  cam->set_parameter(camera::CamParamType::AutoWhiteBalance,
                     s.auto_white_balance);
  cam->set_parameter(camera::CamParamType::RGain, s.rgain);
  cam->set_parameter(camera::CamParamType::GGain, s.ggain);
  cam->set_parameter(camera::CamParamType::BGain, s.bgain);
  return cam;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto ros_node = ia::ros::NodeSingleton::GetInstance().node();
  auto logger = ros_node->get_logger();

  // ---- ROS2 发布器 ----
  ia::ros::PublisherPool<sensor_msgs::msg::CompressedImage>
      image_publisher_pool{ros_node};
  ia::ros::PublisherPool<visualization_msgs::msg::MarkerArray>
      marker_array_pub_pool{ros_node};
  ia::ros::ParamsManager<RosParams> ros_params_manager(ros_node);

  const auto& config = ros_params_manager.data();
  ros_params_manager.Init();

  if (config.debug) {
    logger.set_level(rclcpp::Logger::Level::Debug);
  } else {
    image_publisher_pool.Bypass(true);
    logger.set_level(rclcpp::Logger::Level::Info);
  }

  // ---- IMU 四元数（可选：有IMU topic才启用追踪/瞄准后端） ----
  std::mutex imu_mutex;
  Eigen::Quaterniond latest_imu_quaternion = Eigen::Quaterniond::Identity();
  bool imu_received = false;
  bool imu_enabled = !config.aimer.imu_topic.empty();

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  if (imu_enabled) {
    imu_sub = ros_node->create_subscription<sensor_msgs::msg::Imu>(
        config.aimer.imu_topic, rclcpp::SensorDataQoS(),
        [&](sensor_msgs::msg::Imu::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(imu_mutex);
          latest_imu_quaternion =
              Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                 msg->orientation.y, msg->orientation.z);
          imu_received = true;
        });
    RCLCPP_INFO(logger, "IMU enabled: topic=%s",
                config.aimer.imu_topic.c_str());
  } else {
    RCLCPP_INFO(logger, "IMU disabled (imu_topic empty) — 仅检测+可视化");
  }

  // ---- 后端模块 ----
  ia::detection::DetectionPipeline detection_pipeline{config};
  ia::detection::ArmorSolver armor_solver(config);
  ia::estimation::Tracker tracker(config, armor_solver);
  ia::decision::Aimer aimer(config);

  // ---- 相机初始化 ----
  const auto& cam_type = config.camera_settings.camera_type;
  RCLCPP_INFO(logger, "Camera type: %s", cam_type.c_str());

#ifdef HAS_HIKROBOT_CAM
  std::shared_ptr<camera::HikrobotCam> hik_cam;
#endif
  std::shared_ptr<camera::DahengCam> dh_cam;
  bool cam_ok = false;

  if (cam_type == "hikrobot") {
#ifdef HAS_HIKROBOT_CAM
    hik_cam = std::make_shared<camera::HikrobotCam>();
    cam_ok = hik_cam->open();
    if (!cam_ok)
      RCLCPP_FATAL(logger, "Hikrobot open failed: %s",
                   hik_cam->error_message().c_str());
#else
    RCLCPP_FATAL(logger, "Hikrobot not supported (MVS SDK missing)");
#endif
  } else {
    dh_cam = InitCamera(config);
    cam_ok = dh_cam->open();
    if (!cam_ok)
      RCLCPP_FATAL(logger, "Daheng open failed: %s",
                   dh_cam->error_message().c_str());
  }

  if (!cam_ok) {
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(logger, "Camera opened: %dx%d @ %d fps",
              config.camera_settings.width, config.camera_settings.height,
              config.camera_settings.fps);

  double bullet_speed = config.shooter.bullet_speed;
  RCLCPP_INFO(logger, "Bullet speed: %.1f m/s, auto_fire: %s",
              bullet_speed, config.shooter.auto_fire ? "true" : "false");

  // ---- 主循环 ----
  cv_bridge::CvImage frame;
  frame.encoding = sensor_msgs::image_encodings::BGR8;

  int frame_idx = 0;
  while (rclcpp::ok()) {
    frame_idx++;
    bool grabbed = false;
#ifdef HAS_HIKROBOT_CAM
    if (hik_cam) grabbed = hik_cam->grab_image(frame.image);
    else
#endif
        if (dh_cam)
      grabbed = dh_cam->grab_image(frame.image);

    if (!grabbed || frame.image.empty()) {
      RCLCPP_INFO_THROTTLE(logger, *ros_node->get_clock(), 2000,
                           "Waiting for frame... (%d)", frame_idx);
      continue;
    }

    auto timestamp = std::chrono::steady_clock::now();

    // 1. 检测
    const auto detect_start = std::chrono::high_resolution_clock::now();
    const auto armors_result = detection_pipeline.ProcessImage(frame.image);
    const auto detect_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - detect_start)
            .count();

    // 只要有结果就画图+显示（不管检没检测到装甲板）
    if (config.debug) {
      if (armors_result) {
        ia::detection::DrawArmor(frame.image, *armors_result.value());
        marker_array_pub_pool.Publish(
            "armors", ia::detection::DrawArmorToRviz(*armors_result.value()));
      }
      image_publisher_pool.Publish("debug_image",
                                   frame.toCompressedImageMsg());

      static bool has_display = std::getenv("DISPLAY") != nullptr;
      if (has_display) {
        cv::Mat display;
        cv::resize(frame.image, display, {}, 0.5, 0.5);
        cv::imshow("Infantry Aimbot", display);
        if (cv::waitKey(1) == 27) break;
      }
    }

    RCLCPP_INFO_THROTTLE(logger, *ros_node->get_clock(), 1000,
                         "Frame %d | detect:%ldus armors:%zu",
                         frame_idx, detect_us,
                         armors_result ? armors_result.value()->size() : 0);

    if (!armors_result) continue;

    // 2. IMU → 世界系旋转（无IMU则跳过追踪/瞄准）
    bool have_imu = false;
    {
      std::lock_guard<std::mutex> lock(imu_mutex);
      if (imu_received) {
        armor_solver.SetRGimbal2World(latest_imu_quaternion);
        have_imu = true;
      }
    }

    if (have_imu) {
      double gimbal_yaw =
          ia::tools::Eulers(armor_solver.RGimbal2World(), 2, 1, 0)[0];

      // 3. 坐标变换 + 分类信息
      std::list<ia::Armor> armor_list;
      for (auto& armor : *armors_result.value()) {
        armor.name = NumIdToArmorName(armor.num_id);
        armor.kind = TypeToArmorKind(armor.type);
        armor.priority = ArmorNameToPriority(armor.name);
        armor_solver.Solve(armor);
        armor_list.push_back(armor);
      }

      // 4. 追踪
      auto targets = tracker.Track(armor_list, timestamp);

      // 5. 瞄准
      ia::Command command =
          aimer.Aim(targets, timestamp, bullet_speed);

      // 6. 开火判断
      command.shoot = aimer.Shoot(command, targets, gimbal_yaw);

      // ROS 日志
      if (config.debug && !targets.empty()) {
        auto ekf_x = targets.front().EkfX();
        RCLCPP_INFO_THROTTLE(
            logger, *ros_node->get_clock(), 1000,
            "[%s] detect:%ldus pos=(%.2f,%.2f) omega=%.2f "
            "cmd:(%.3f,%.3f) fire=%d",
            tracker.State().c_str(), detect_us,
            ekf_x[0], ekf_x[2], ekf_x[7],
            command.yaw, command.pitch, command.shoot);
      }
    }

  }

#ifdef HAS_HIKROBOT_CAM
  if (hik_cam) hik_cam->close();
#endif
  if (dh_cam) dh_cam->close();
  rclcpp::shutdown();
  return 0;
}

#include <backward.hpp>
backward::SignalHandling _;  ///< 程序崩掉时打印stack trace
