
#ifndef DAHENG_EXT_TRIG_HPP
#define DAHENG_EXT_TRIG_HPP

#include <functional>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>

#include "gxapi/DxImageProc.h"
#include "gxapi/GxIAPI.h"
#include "typedefs.hpp"

#define ACQ_TRANSFER_SIZE (64 * 1024)
#define ACQ_TRANSFER_NUMBER_URB 64
#define ACQ_BUFFER_NUM 3

namespace camera {
class DahengCamExtTrig {
 public:
  explicit DahengCamExtTrig(const std::string camera_sn = "");
  virtual ~DahengCamExtTrig();

  // 打开设备
  bool open();

  // 关闭设备
  bool close();

  // 返回是否打开
  bool is_open();

  void OnImageCallback(std::function<void(cv::Mat &)> callback);

  // 设置参数
  bool set_parameter(CamParamType type, int value);

  // 得到参数
  bool get_parameter(CamParamType type, int &value);

  // 返回错误信息
  std::string error_message() { return ("Error: " + error_message_); }

  // 重设参数
  bool ResetParameters(const std::map<CamParamType, int> &param_map);

  // 改变曝光参数
  void SetExposure(int exp);

 private:
  void GX_STDC OnCaptureFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrame);

 private:
  bool is_open_;
  cv::Mat image_buffer_{};
  std::function<void(cv::Mat &)> on_image_callback_{};
  GX_DEV_HANDLE device_;           // 设备权柄
  PGX_FRAME_BUFFER pFrameBuffer_;  // raw 图像的buffer
  uint8_t *rgbImagebuf_;           // rgb 图像的buffer
  std::string error_message_;      // 错误消息，对外传输
  std::string camera_sn_;          // 相机sn号

 public:
  std::unordered_map<CamParamType, int> params_;

 private:
  // 设置相机初始化参数
  bool SettingsInit();

  // 初始化相机sdk
  bool InitSdk();
};

}  // namespace camera

#endif  // DAHENG_EXT_TRIG_HPP