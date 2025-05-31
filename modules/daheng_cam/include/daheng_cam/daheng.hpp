
#ifndef DAHENG_HPP
#define DAHENG_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>

#include "camera_interface.hpp"
#include "gxapi/DxImageProc.h"
#include "gxapi/GxIAPI.h"

#define ACQ_TRANSFER_SIZE (64 * 1024)
#define ACQ_TRANSFER_NUMBER_URB 64
#define ACQ_BUFFER_NUM 3

namespace camera {
class DahengCam : public CamInterface {
 public:
  explicit DahengCam(const std::string camera_sn = "");
  virtual ~DahengCam() override;

  // 打开设备
  bool open() override;

  // 关闭设备
  bool close() override;

  // 返回是否打开
  bool is_open() override;

  // 获取Mat图像
  bool grab_image(cv::Mat &image) override;

  // 设置参数
  bool set_parameter(CamParamType type, int value) override;

  // 得到参数
  bool get_parameter(CamParamType type, int &value) override;

  // 返回错误信息
  std::string error_message() override { return ("Error: " + error_message_); }

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
