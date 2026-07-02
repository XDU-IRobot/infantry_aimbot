#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>

#include "MvCameraControl.h"
#include "daheng_cam/typedefs.hpp"

namespace camera {

class HikrobotCam {
 public:
  explicit HikrobotCam();
  ~HikrobotCam();

  bool open();
  bool close();
  bool is_open();
  bool grab_image(cv::Mat& image);

  bool set_parameter(CamParamType type, int value);
  bool get_parameter(CamParamType type, int& value);
  std::string error_message() { return error_message_; }

 private:
  bool is_open_{false};
  void* handle_{nullptr};
  unsigned char* raw_buf_{nullptr};
  unsigned int raw_buf_size_{0};
  std::string error_message_;
  int width_{1440};
  int height_{1080};
  bool triggered_{false};
  std::unordered_map<CamParamType, int> params_;

  bool InitSdk();
  bool StartGrabbing();
  bool StopGrabbing();
  bool SetPixelFormat();
  bool SetTriggerMode(bool on);
};

}  // namespace camera
