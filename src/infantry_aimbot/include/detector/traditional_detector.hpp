
#pragma once

#include "number_classifier.hpp"
#include "typedefs.hpp"

namespace ia {
namespace detector {

// 输入图像为BGR
class TraditionalDetector {
 public:
  struct ProcessParams {
    double bin_threshold;
    int enemy_color;
  };

  struct LightParams {
    double angle_to_vertigal_max;
    double height_width_min_ratio;
    double size_area_min_ratio;
    bool is_corner_correct;
  };

  struct ArmorParams {
    double lights_angle_max_diff;
    double lights_length_max_ratio;
    double lights_y_max_ratio;
    double width_height_min_ratio;
    double width_height_max_ratio;
    double max_angle;
    double inside_thresh;
  };

 public:
  TraditionalDetector(ProcessParams p_params, LightParams l_params, ArmorParams a_params);
  ~TraditionalDetector() = default;
  /**
   * @brief 识别图像中的装甲板
   * @param image
   * @return result_sp<std::vector<Armor>>
   */
  result_sp<std::vector<Armor>> DetectArmors(const cv::Mat &image);

  static void DrawResult(cv::Mat &image, const std::vector<Armor> &armors);

  /**
   * @brief 设置敌方颜色
   * @param[in] enemy_color 敌方颜色
   **/
  void setEnemyColor(Color enemy_color);

 private:
  result_sp<std::vector<LightBlob>> FindLights(const cv::Mat &image);
  bool IsValidLight(const LightBlob &light);
  result_sp<std::vector<Armor>> MatchLights(std::vector<LightBlob> &lights);
  /**
   * @brief 判断两个灯是否为装甲板
   * @param[in] light_1 灯1
   * @param[in] light_2 灯2
   * @return 0: 不是装甲板; 1: 小装甲板; 2: 大装甲板
   **/
  Armor::Type IsValidArmor(const LightBlob &light_1, const LightBlob &light_2);

  /**
   * @brief 寻找最大亮度变化点
   * @param[in] image 输入图像
   * @param[in] start 起始点
   * @param[in] end 终止点
   * @return 最大亮度变化点
   **/
  cv::Point findMaxBrightnessChange(const cv::Mat &image, cv::Point start, cv::Point end);

  ProcessParams process_params_;
  LightParams light_params_;
  ArmorParams armor_params_;
  Color enemy_color_;

 public:
  cv::Mat src_;
  cv::Mat debug_binary_;
};
}  // namespace detector
}  // namespace ia