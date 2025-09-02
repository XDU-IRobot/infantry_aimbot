
#pragma once

#include <vector>
#include <string>

struct RosParams {
  bool debug;
  struct {
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
};