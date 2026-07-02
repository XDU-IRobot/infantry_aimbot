
#pragma once

#include <vector>
#include <string>

struct RosParams {
  bool debug;
  struct {
    std::string camera_type;  // "daheng" 或 "hikrobot"
    int width;
    int height;
    int exposure;
    bool auto_exposure;
    int gain;
    int fps;
    bool auto_white_balance;
    double rgain;
    double bgain;
    double ggain;
    int time_offset;
    bool auto_exp_change;
    int max_exp;
    int min_exp;
  } camera_settings;
  struct {
    std::vector<double> camera_matrix;
    std::vector<double> distortion_coefficients;
  } camera_info;
  struct {
    double bin_threshold;
    int enemy_color;
    bool is_corner_correct;
    double angle_to_vertical_max;
    double height_width_min_ratio;
    double size_area_min_ratio;
    double lights_angle_max_diff;
    double lights_length_max_ratio;
    double lights_y_max_ratio;
    double width_height_min_ratio;
    double width_height_max_ratio;
    double max_angle;
    double inside_thresh;
  } detector;
  struct {
    std::string model_path;
  } number_classifier;

  // ---- 后端模块参数（移植自 sp_vision_25） ----
  struct {
    std::vector<double> R_gimbal2imubody;  // 9 elements, row-major 3x3
    std::vector<double> R_camera2gimbal;   // 9 elements
    std::vector<double> t_camera2gimbal;   // 3 elements [x, y, z]
  } solver;

  struct {
    int min_detect_count;               // 最小连续检测帧数, default 5
    int max_temp_lost_count;            // 最大临时丢失帧数, default 15
    int outpost_max_temp_lost_count;    // 前哨站最大临时丢失帧数, default 75
  } tracker;

  struct {
    std::string imu_topic;        // IMU话题名，空字符串=禁用IMU+后端
    double yaw_offset;            // yaw偏置 (degree)
    double pitch_offset;          // pitch偏置 (degree)
    double comming_angle;         // 来袭角 (degree)
    double leaving_angle;         // 离去角 (degree)
    double decision_speed;        // 决策速度阈值 (rad/s)
    double high_speed_delay_time; // 高速预测延迟 (s)
    double low_speed_delay_time;  // 低速预测延迟 (s)
  } aimer;

  struct {
    double first_tolerance;       // 近距离容差 (degree)
    double second_tolerance;      // 远距离容差 (degree)
    double judge_distance;        // 距离阈值 (m)
    bool auto_fire;               // 是否自动开火
    double bullet_speed;          // 子弹初速 (m/s), default 23.0
  } shooter;
};