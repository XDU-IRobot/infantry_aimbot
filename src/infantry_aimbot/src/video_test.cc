/// @file video_test.cc
/// @brief 离线视频测试程序（参照 sp_vision_25 auto_aim_test 模式）
/// 用法:
///   ./video_test -c=configs/demo.yaml -s=0 -e=0 assets/demo/demo
///   ./video_test @input_path [options...]

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include "decision/aimer.hpp"
#include "detection/armor_solver.hpp"
#include "detection/pipeline.hpp"
#include "estimation/tracker.hpp"
#include "params.hpp"
#include "tools/math_tools.hpp"

namespace {

const std::string kKeys =
    "{help h usage ? |              | 输出命令行参数说明 }"
    "{config-path c  | configs/demo.yaml | yaml配置文件路径}"
    "{start-index s  | 0            | 视频起始帧下标 }"
    "{end-index e    | 0            | 视频结束帧下标 }"
    "{@input-path    | demo         | avi和txt文件的基础路径}";

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
  return (type == ia::Armor::BIG || type == ia::Armor::GRAY_BIG_ARMOR) ? ia::ArmorKind::kBig : ia::ArmorKind::kSmall;
}

void DrawText(cv::Mat& img, const std::string& text, cv::Point pos, cv::Scalar color = {0, 255, 0}) {
  cv::putText(img, text, pos, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
}

void DrawCross(cv::Mat& img, cv::Point2f center, int size, cv::Scalar color) {
  int cx = static_cast<int>(center.x);
  int cy = static_cast<int>(center.y);
  cv::line(img, {cx - size, cy}, {cx + size, cy}, color, 2);
  cv::line(img, {cx, cy - size}, {cx, cy + size}, color, 2);
}

/// 从YAML加载检测器参数（映射 sp_vision_25 → infantry_aimbot）
void LoadDetectorParams(const YAML::Node& yaml, RosParams& config) {
  if (yaml["threshold"]) config.detector.bin_threshold = yaml["threshold"].as<double>();
  if (yaml["enemy_color"]) {
    std::string ec = yaml["enemy_color"].as<std::string>();
    config.detector.enemy_color = (ec == "red") ? 0 : 1;
  }
  // sp_vision_25 参数 → infantry_aimbot 参数
  if (yaml["max_angle_error"]) config.detector.angle_to_vertical_max = yaml["max_angle_error"].as<double>();
  if (yaml["min_lightbar_ratio"]) config.detector.height_width_min_ratio = yaml["min_lightbar_ratio"].as<double>();
  if (yaml["min_armor_ratio"]) config.detector.width_height_min_ratio = yaml["min_armor_ratio"].as<double>();
  if (yaml["max_armor_ratio"]) config.detector.width_height_max_ratio = yaml["max_armor_ratio"].as<double>();
  if (yaml["max_side_ratio"]) config.detector.lights_length_max_ratio = yaml["max_side_ratio"].as<double>();
}

/// 从YAML加载解算器参数
void LoadSolverParams(const YAML::Node& yaml, RosParams& config) {
  if (yaml["camera_matrix"]) {
    config.camera_info.camera_matrix = yaml["camera_matrix"].as<std::vector<double>>();
  }
  if (yaml["distort_coeffs"]) {
    config.camera_info.distortion_coefficients = yaml["distort_coeffs"].as<std::vector<double>>();
  }
  if (yaml["R_gimbal2imubody"]) {
    config.solver.R_gimbal2imubody = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  }
  if (yaml["R_camera2gimbal"]) {
    config.solver.R_camera2gimbal = yaml["R_camera2gimbal"].as<std::vector<double>>();
  }
  if (yaml["t_camera2gimbal"]) {
    config.solver.t_camera2gimbal = yaml["t_camera2gimbal"].as<std::vector<double>>();
  }
}

/// 从YAML加载追踪/瞄准/射击参数
void LoadBackendParams(const YAML::Node& yaml, RosParams& config) {
  if (yaml["min_detect_count"]) config.tracker.min_detect_count = yaml["min_detect_count"].as<int>();
  if (yaml["max_temp_lost_count"]) config.tracker.max_temp_lost_count = yaml["max_temp_lost_count"].as<int>();
  if (yaml["outpost_max_temp_lost_count"])
    config.tracker.outpost_max_temp_lost_count = yaml["outpost_max_temp_lost_count"].as<int>();

  if (yaml["yaw_offset"]) config.aimer.yaw_offset = yaml["yaw_offset"].as<double>();
  if (yaml["pitch_offset"]) config.aimer.pitch_offset = yaml["pitch_offset"].as<double>();
  if (yaml["comming_angle"]) config.aimer.comming_angle = yaml["comming_angle"].as<double>();
  if (yaml["leaving_angle"]) config.aimer.leaving_angle = yaml["leaving_angle"].as<double>();
  if (yaml["decision_speed"]) config.aimer.decision_speed = yaml["decision_speed"].as<double>();
  if (yaml["high_speed_delay_time"]) config.aimer.high_speed_delay_time = yaml["high_speed_delay_time"].as<double>();
  if (yaml["low_speed_delay_time"]) config.aimer.low_speed_delay_time = yaml["low_speed_delay_time"].as<double>();

  if (yaml["first_tolerance"]) config.shooter.first_tolerance = yaml["first_tolerance"].as<double>();
  if (yaml["second_tolerance"]) config.shooter.second_tolerance = yaml["second_tolerance"].as<double>();
  if (yaml["judge_distance"]) config.shooter.judge_distance = yaml["judge_distance"].as<double>();
  if (yaml["auto_fire"]) config.shooter.auto_fire = yaml["auto_fire"].as<bool>();
  // 子弹初速（sp_vision_25 从CAN获取，YAML中可选，默认27m/s）
  if (yaml["bullet_speed"])
    config.shooter.bullet_speed = yaml["bullet_speed"].as<double>();
  else
    config.shooter.bullet_speed = 27.0;
}

}  // namespace

int main(int argc, char* argv[]) {
  // 解析命令行参数
  cv::CommandLineParser cli(argc, argv, kKeys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  std::string input_path = cli.get<std::string>(0);
  std::string config_path = cli.get<std::string>("config-path");
  int start_index = cli.get<int>("start-index");
  int end_index = cli.get<int>("end-index");

  // 加载YAML配置
  RosParams config{};
  config.debug = true;
  config.number_classifier.model_path = "";

  // 默认值
  config.camera_info.camera_matrix = {1569.5, 0, 655.8, 0, 1569.9, 532.0, 0, 0, 1};
  config.camera_info.distortion_coefficients = {0, 0, 0, 0, 0};
  config.solver.R_gimbal2imubody = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  config.solver.R_camera2gimbal = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  config.solver.t_camera2gimbal = {0, 0, 0};
  config.tracker.min_detect_count = 5;
  config.tracker.max_temp_lost_count = 15;
  config.tracker.outpost_max_temp_lost_count = 75;
  config.detector.enemy_color = 1;

  try {
    YAML::Node yaml = YAML::LoadFile(config_path);
    LoadDetectorParams(yaml, config);
    LoadSolverParams(yaml, config);
    LoadBackendParams(yaml, config);
    std::cout << "配置加载: " << config_path << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "配置加载失败: " << e.what() << "，使用默认值" << std::endl;
  }

  // 打开视频和IMU文件
  std::string video_path = input_path + ".avi";
  std::string text_path = input_path + ".txt";
  cv::VideoCapture video(video_path);
  if (!video.isOpened()) {
    std::cerr << "无法打开视频: " << video_path << std::endl;
    return 1;
  }
  std::ifstream text(text_path);
  if (!text.is_open()) {
    std::cerr << "无法打开IMU文件: " << text_path << std::endl;
    return 1;
  }

  // 初始化模块
  ia::detection::DetectionPipeline detection_pipeline(config);
  ia::detection::ArmorSolver armor_solver(config);
  ia::estimation::Tracker tracker(config, armor_solver);
  ia::decision::Aimer aimer(config);

  double bullet_speed = config.shooter.bullet_speed;

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();
  ia::Command last_command;

  // 跳到起始帧
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  std::cout << "=== 视频测试开始 ===" << std::endl;
  std::cout << "输入: " << input_path << std::endl;
  std::cout << "子弹初速: " << bullet_speed << " m/s" << std::endl;
  std::cout << "帧范围: " << start_index << " ~ " << (end_index > 0 ? end_index : -1) << std::endl;
  std::cout << "=== 检测器参数 ===" << std::endl;
  std::cout << "  bin_threshold: " << config.detector.bin_threshold << std::endl;
  std::cout << "  enemy_color: " << (config.detector.enemy_color ? "blue" : "red") << std::endl;
  std::cout << "  angle_to_vertical_max: " << config.detector.angle_to_vertical_max << std::endl;
  std::cout << "  height_width_min_ratio: " << config.detector.height_width_min_ratio << std::endl;
  std::cout << "  width_height_min_ratio: " << config.detector.width_height_min_ratio << std::endl;
  std::cout << "  width_height_max_ratio: " << config.detector.width_height_max_ratio << std::endl;
  std::cout << "  lights_length_max_ratio: " << config.detector.lights_length_max_ratio << std::endl;
  std::cout << "  按键: q=退出 空格=暂停 b=切换二值图 t/T=阈值+/- []=阈值±10" << std::endl;

  // 可调参数
  int bin_thresh = static_cast<int>(config.detector.bin_threshold);
  double gamma = 1.0;         // 模拟曝光: 1.0=原图, 0.5=减半, 0.3=更低
  bool color_filter = false;  // B-R色差滤波: 滤白光, 只保留蓝色灯条

  bool show_binary = false;
  bool paused = false;
  bool step_frame = false;
  int frame_delay = 30;  // ms, 可调速
  int frame_count = start_index;
  cv::Mat last_img;
  for (;;) {
    char diag[256];

    // 非暂停 或 单步 → 读新帧
    if (!paused || step_frame) {
      step_frame = false;
      frame_count++;
      if (end_index > 0 && frame_count > end_index) break;

      video.read(img);
      if (img.empty()) break;
      img.copyTo(last_img);
    } else {
      // 暂停：显示上一帧
      if (!last_img.empty()) last_img.copyTo(img);
    }

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(static_cast<int>(t * 1e6));

    /// === 诊断：生成二值图（模拟曝光调节） ===
    if (show_binary || frame_count == start_index) {
      std::vector<cv::Mat> bgr;
      cv::split(img, bgr);
      int ch = (config.detector.enemy_color == 0) ? 2 : 0;
      cv::Mat binary;
      // gamma 衰减模拟降低曝光: 乘系数 <1 压缩亮度
      cv::Mat darkened;
      bgr[ch].convertTo(darkened, CV_8U, gamma, 0);
      cv::threshold(darkened, binary, bin_thresh, 255, cv::THRESH_BINARY);

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(binary.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

      // 轮廓过多时跳过可视化防止卡死
      bool too_many = contours.size() > 5000;
      int valid_count = 0;
      if (!too_many) {
        for (const auto& c : contours) {
          if (cv::contourArea(c) >= 9) valid_count++;
        }
      }

      std::snprintf(diag, sizeof(diag), "Binary thresh=%d contours=%zu valid=%d %zux%zu%s", bin_thresh, contours.size(),
                    valid_count, img.cols, img.rows, too_many ? " TOO_MANY!" : "");
      DrawText(img, diag, {10, img.rows - 20}, {0, 255, 255});

      if (show_binary) {
        cv::Mat binary_color;
        cv::cvtColor(binary, binary_color, cv::COLOR_GRAY2BGR);
        if (!too_many) {
          // 绘制轮廓+分析每个轮廓的minAreaRect
          for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 9) continue;
            auto rrect = cv::minAreaRect(c);
            // width/height swap: 确保 height >= width
            float rw = rrect.size.width, rh = rrect.size.height;
            if (rw > rh) std::swap(rw, rh);
            double ratio = rh / rw;
            double rect_ratio = area / (rw * rh);  // 轮廓面积/外接矩形面积

            // 绿色=通过IsValidLight预检, 红色=不通过
            bool pass = (ratio >= config.detector.height_width_min_ratio);
            cv::Scalar color = pass ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);

            cv::drawContours(binary_color, std::vector<std::vector<cv::Point>>{c}, -1, color, 1);
            // 画minAreaRect
            cv::Point2f vertices[4];
            rrect.points(vertices);
            for (int k = 0; k < 4; k++) cv::line(binary_color, vertices[k], vertices[(k + 1) % 4], color, 1);

            // 标注 ratio
            char label[32];
            std::snprintf(label, sizeof(label), "%.1f", ratio);
            cv::putText(binary_color, label,
                        cv::Point(static_cast<int>(rrect.center.x), static_cast<int>(rrect.center.y)),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
          }
        }  // !too_many
        std::snprintf(diag, sizeof(diag), "Binary thresh=%d | T+/-5, []+/-10, r=ratio_min", bin_thresh);
        cv::putText(binary_color, diag, {10, 20}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 0}, 1);
        cv::resize(binary_color, binary_color, {}, 0.5, 0.5);
        cv::imshow("Binary Preview", binary_color);
      }
    }

    /// === 自瞄核心逻辑 ===

    armor_solver.SetRGimbal2World({w, x, y, z});

    // 应用实时调整的阈值
    detection_pipeline.SetBinThreshold(bin_thresh);

    // 模拟曝光调节 + 色差滤波
    cv::Mat proc_img;
    if (color_filter) {
      // B-R色差: 白光(R=G=B)→0, 蓝光(B>R)→正数
      std::vector<cv::Mat> bgr;
      cv::split(img, bgr);
      cv::Mat diff;
      cv::subtract(bgr[0], bgr[2], diff);                    // B - R
      cv::threshold(diff, diff, 0, 255, cv::THRESH_TOZERO);  // 截断负数
      if (std::abs(gamma - 1.0) > 0.01) {
        diff.convertTo(proc_img, CV_8U, gamma, 0);
      } else {
        proc_img = diff;
      }
      cv::cvtColor(proc_img, proc_img, cv::COLOR_GRAY2BGR);  // 转回BGR给pipeline
    } else if (std::abs(gamma - 1.0) > 0.01) {
      img.convertTo(proc_img, CV_8U, gamma, 0);
    } else {
      proc_img = img;
    }

    // 1. 检测
    auto yolo_start = std::chrono::steady_clock::now();
    auto armors_result = detection_pipeline.ProcessImage(proc_img);
    auto tracker_start = std::chrono::steady_clock::now();

    if (!armors_result) continue;
    auto& armors = *armors_result.value();

    // 2. 坐标变换
    std::list<ia::Armor> armor_list;
    for (auto& armor : armors) {
      armor.name = NumIdToArmorName(armor.num_id);
      armor.kind = TypeToArmorKind(armor.type);
      armor.priority = ArmorNameToPriority(armor.name);
      armor_solver.Solve(armor);
      armor_list.push_back(armor);
    }

    // 3. 追踪
    auto targets = tracker.Track(armor_list, timestamp);
    auto aimer_start = std::chrono::steady_clock::now();

    // 4. 瞄准
    auto command = aimer.Aim(targets, timestamp, bullet_speed, false);
    auto finish = std::chrono::steady_clock::now();

    // 5. 开火判断
    if (!targets.empty() && aimer.debug_aim_point.valid && std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
      command.shoot = true;
    if (command.control) last_command = command;

    /// === 调试输出 ===

    double yolo_ms = ia::tools::DeltaTime(tracker_start, yolo_start) * 1e3;
    double tracker_ms = ia::tools::DeltaTime(aimer_start, tracker_start) * 1e3;
    double aimer_ms = ia::tools::DeltaTime(finish, aimer_start) * 1e3;

    printf("[%d] detect:%.1fms tracker:%.1fms aimer:%.1fms | cmd:%.2f,%.2f shoot:%d\n", frame_count, yolo_ms,
           tracker_ms, aimer_ms, command.yaw * 57.3, command.pitch * 57.3, command.shoot);

    // 状态信息叠加
    char buf[256];
    std::snprintf(buf, sizeof(buf), "cmd: %s,%.2f,%.2f shoot:%d", command.control ? "T" : "F", command.yaw * 57.3,
                  command.pitch * 57.3, command.shoot);
    DrawText(img, buf, {10, 60}, {154, 50, 205});

    Eigen::Quaterniond gimbal_q(w, x, y, z);
    std::snprintf(buf, sizeof(buf), "gimbal yaw:%.2f",
                  (ia::tools::Eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]);
    DrawText(img, buf, {10, 90}, {255, 255, 255});

    // === JSON数据（兼容PlotJuggler） ===
    nlohmann::json data;
    data["armor_num"] = armors.size();
    if (!armors.empty()) {
      const auto& armor = armors.front();
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
    }

    data["gimbal_yaw"] = ia::tools::Eulers(gimbal_q.toRotationMatrix(), 2, 1, 0)[0] * 57.3;
    data["cmd_yaw"] = command.yaw * 57.3;
    data["shoot"] = command.shoot;

    if (!targets.empty()) {
      auto target = targets.front();
      auto ekf_x = target.EkfX();
      std::vector<Eigen::Vector4d> armor_xyza_list = target.ArmorXyzaList();

      // 重投影所有装甲板位置
      for (const auto& xyza : armor_xyza_list) {
        auto image_points = armor_solver.ReprojectArmor(xyza.head(3), xyza[3], target.armor_kind, target.name);
        for (const auto& p : image_points) DrawCross(img, p, 6, {0, 255, 0});
      }

      // 瞄准点重投影
      if (aimer.debug_aim_point.valid) {
        auto aim_xyza = aimer.debug_aim_point.xyza;
        auto image_points = armor_solver.ReprojectArmor(aim_xyza.head(3), aim_xyza[3], target.armor_kind, target.name);
        for (const auto& p : image_points) DrawCross(img, p, 8, {0, 0, 255});
      }

      // EKF状态数据
      data["x"] = ekf_x[0];
      data["vx"] = ekf_x[1];
      data["y"] = ekf_x[2];
      data["vy"] = ekf_x[3];
      data["z"] = ekf_x[4];
      data["vz"] = ekf_x[5];
      data["a"] = ekf_x[6] * 57.3;
      data["w"] = ekf_x[7];
      data["r"] = ekf_x[8];
      data["l"] = ekf_x[9];
      data["h"] = ekf_x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.Ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.Ekf().data.at("residual_pitch");
      data["residual_distance"] = target.Ekf().data.at("residual_distance");
      data["residual_angle"] = target.Ekf().data.at("residual_angle");
      data["nis"] = target.Ekf().data.at("nis");
      data["nees"] = target.Ekf().data.at("nees");
      data["nis_fail"] = target.Ekf().data.at("nis_fail");
      data["nees_fail"] = target.Ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.Ekf().data.at("recent_nis_failures");
    }

    // 输出JSON（每行一条，可重定向到文件供PlotJuggler读取）
    std::cout << data.dump() << std::endl;

    // 参数状态
    std::snprintf(diag, sizeof(diag), "thresh=%d gamma=%.1f ratio=%.1f ang=%.0f %s speed=%dms", bin_thresh, gamma,
                  config.detector.height_width_min_ratio, config.detector.angle_to_vertical_max,
                  color_filter ? "B-R" : "", frame_delay);
    DrawText(img, diag, {10, img.rows - 20}, {255, 200, 0});

    // 显示
    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("Infantry Aimbot - Video Test", img);
    int key = cv::waitKey(frame_delay);
    if (key == 'q' || key == 27) break;

    // === 播放控制 ===
    if (key == ' ') paused = !paused;
    if (key == '.' || key == 'n') {
      if (paused) step_frame = true;
    }
    if (key == 's') frame_delay = 100;  // 慢放
    if (key == 'S') frame_delay = 300;  // 更慢
    if (key == 'f') frame_delay = 10;   // 快放
    if (key == 'F') frame_delay = 1;    // 极速

    // === 检测器调参 ===
    if (key == 'b' || key == 'B') show_binary = !show_binary;
    if (key == 't') bin_thresh = std::max(10, bin_thresh - 5);
    if (key == 'T') bin_thresh = std::min(255, bin_thresh + 5);
    if (key == '[') bin_thresh = std::max(10, bin_thresh - 10);
    if (key == ']') bin_thresh = std::min(255, bin_thresh + 10);
    if (key == 'r') {
      config.detector.height_width_min_ratio = std::max(1.0, config.detector.height_width_min_ratio - 0.1);
      detection_pipeline.SetHeightWidthMinRatio(config.detector.height_width_min_ratio);
    }
    if (key == 'R') {
      config.detector.height_width_min_ratio = std::min(5.0, config.detector.height_width_min_ratio + 0.1);
      detection_pipeline.SetHeightWidthMinRatio(config.detector.height_width_min_ratio);
    }
    // a/A: 调整灯条最大倾斜角
    if (key == 'a') {
      config.detector.angle_to_vertical_max = std::max(5.0, config.detector.angle_to_vertical_max - 5.0);
      detection_pipeline.SetAngleToVerticalMax(config.detector.angle_to_vertical_max);
    }
    if (key == 'A') {
      config.detector.angle_to_vertical_max = std::min(90.0, config.detector.angle_to_vertical_max + 5.0);
      detection_pipeline.SetAngleToVerticalMax(config.detector.angle_to_vertical_max);
    }
    // g/G: 模拟曝光调节 (gamma系数)
    if (key == 'g') gamma = std::max(0.1, gamma - 0.1);
    if (key == 'G') gamma = std::min(1.0, gamma + 0.1);
    // c: 切换色差滤波 (B-R) — 滤白光，只保留蓝色灯条
    if (key == 'c') color_filter = !color_filter;
  }

  std::cout << "=== 完成，共 " << frame_count << " 帧 ===" << std::endl;
  return 0;
}
