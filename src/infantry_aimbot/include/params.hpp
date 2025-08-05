
#pragma once

struct Params {
  bool debug;

  double bin_threshold;
  int enemy_color;

  double angle_to_vertigal_max;
  double height_width_min_ratio;
  double size_area_min_ratio;
  bool is_corner_correct;

  double lights_angle_max_diff;
  double lights_length_max_ratio;
  double lights_y_max_ratio;
  double width_height_min_ratio;
  double width_height_max_ratio;
  double max_angle;
  double inside_thresh;
};