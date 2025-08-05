
#ifndef DAHENG_HPP
#define DAHENG_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>

#include "typedefs.hpp"
#include "gxapi/DxImageProc.h"
#include "gxapi/GxIAPI.h"

namespace camera {
class DahengCam {
 public:
  explicit DahengCam(const std::string camera_sn = "");
  virtual ~DahengCam();

  // 打开设备
  bool open();

  // 关闭设备
  bool close();

  // 返回是否打开
  bool is_open();

  // 获取Mat图像
  bool grab_image(cv::Mat &image);

  // 设置参数
  bool set_parameter(CamParamType type, int value);

  // 得到参数
  bool get_parameter(CamParamType type, int &value);

  // 返回错误信息
  std::string error_message() { return ("Error: " + error_message_); }

  // 重设参数
  bool ResetParameters(const std::map<CamParamType, int> &param_map);

  // 改变曝光参数
  bool SetExposure(int exp);

 private:
  bool is_open_;
  std::string camera_sn_;          // 相机sn号
  GX_DEV_HANDLE device_;           // 设备权柄
  PGX_FRAME_BUFFER pFrameBuffer_;  // raw 图像的buffer
  uint8_t *rgbImagebuf_;           // rgb 图像的buffer
  std::string error_message_;      // 错误消息，对外传输

 public:
  std::unordered_map<CamParamType, int> params_;

 private:
  // 设置相机初始化参数
  bool SettingsInit();

  // 初始化相机sdk
  bool InitSdk();
};

}  // namespace camera

#endif  // DAHENG_HPP
