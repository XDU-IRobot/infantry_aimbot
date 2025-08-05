
#pragma once

#include <opencv2/opencv.hpp>

#include "typedefs.hpp"

namespace ia {
namespace detector {

/**
 * @brief   数字分类器
 * @details 该分类器用于对图像中的数字进行分类，用于识别装甲板上的数字。
 * @warning 这个分类器是线程不安全的，不能在多线程环境中使用。如果需要并行优化，每个worker应该拥有自己的实例。
 */
class NumberClassifier {
 public:
  struct ClassifyResult {
    float confidence;
    int number;
  };

 public:
  /**
   * @param model_path   数字分类模型的路径
   */
  NumberClassifier(std::string model_path) { net_ = cv::dnn::readNetFromONNX(model_path); };

  /**
   * @brief 对图像中一个框出来的roi进行数字分类
   * @param image 图像
   * @param roi   四边形roi，点顺序为左下、左上、右上、右下
   * @return result_sp<ClassifyResult>  分类结果，包含置信度和数字
   */
  result_sp<ClassifyResult> Classify(const cv::Mat &image, const std::vector<cv::Point2f> &roi);

 private:
  const int kLightLength = 12;
  const int kWarpHeight = 28;       ///< 透视变换之后图像的高度
  const int kSmallArmorWidth = 32;  ///< 透视变换后图像的宽度
  // const int kLargeArmorWidth = 54;
  const cv::Size kROISize{20, 28};
  const cv::Size kInputSize{28, 28};

  sp<ClassifyResult> ret_buffer_{std::make_shared<ClassifyResult>()};
  cv::Mat roi_img_;
  cv::Mat net_blob_;
  cv::dnn::Net net_;
};

}  // namespace detector
}  // namespace ia