
#include "detection/number_classifier.hpp"

namespace ia {
namespace detection {
result_sp<NumberClassifier::ClassifyResult> NumberClassifier::Classify(const cv::Mat &img,
                                                                       const std::vector<cv::Point2f> &light_points) {
  const int top_light_y = (kWarpHeight - kLightLength) / 2 - 1;
  const int bottom_light_y = top_light_y + kLightLength;
  const int warp_width = kSmallArmorWidth;
  const cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
  };
  const auto rotation_matrix = cv::getPerspectiveTransform(light_points.data(), target_vertices);

  static cv::Mat warped;
  cv::warpPerspective(img, warped, rotation_matrix, cv::Size(warp_width, kWarpHeight));

  // Get ROI
  roi_img_ = warped(cv::Rect(cv::Point((warp_width - kROISize.width) / 2, 0), kROISize));

  // Binarize
  cv::cvtColor(roi_img_, roi_img_, cv::COLOR_RGB2GRAY);
  cv::threshold(roi_img_, roi_img_, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::resize(roi_img_, roi_img_, kInputSize);

  roi_img_ /= 255.0;

  // Create blob from image
  cv::dnn::blobFromImage(roi_img_, net_blob_);

  // Set the input blob for the neural network
  net_.setInput(net_blob_);

  // Forward pass the image blob through the model
  const cv::Mat outputs = net_.forward();

  // Decode the output
  double confidence;
  cv::Point class_id_point;
  minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);

  ret_buffer_->confidence = confidence;
  ret_buffer_->number = class_id_point.x + 1;

  return ret_buffer_;
}
}  // namespace detection
}  // namespace ia