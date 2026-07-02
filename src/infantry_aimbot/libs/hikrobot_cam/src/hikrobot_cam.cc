#include "hikrobot_cam/hikrobot_cam.hpp"

#include <cstring>
#include <sstream>

namespace camera {

static int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth,
                   unsigned int nHeight) {
  if (!pRgbData) return MV_E_PARAMETER;
  for (unsigned int j = 0; j < nHeight; j++) {
    for (unsigned int i = 0; i < nWidth; i++) {
      unsigned char red = pRgbData[j * nWidth * 3 + i * 3 + 0];
      pRgbData[j * nWidth * 3 + i * 3 + 0] =
          pRgbData[j * nWidth * 3 + i * 3 + 2];
      pRgbData[j * nWidth * 3 + i * 3 + 2] = red;
    }
  }
  return MV_OK;
}

HikrobotCam::HikrobotCam() {}

HikrobotCam::~HikrobotCam() { close(); }

bool HikrobotCam::open() {
  if (is_open_) return true;

  int ret = MV_CC_Initialize();
  if (ret != MV_OK) {
    std::ostringstream ss;
    ss << "MV_CC_Initialize failed: 0x" << std::hex << ret;
    error_message_ = ss.str();
    return false;
  }

  // 枚举设备
  MV_CC_DEVICE_INFO_LIST dev_list;
  memset(&dev_list, 0, sizeof(dev_list));
  ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &dev_list);
  if (ret != MV_OK || dev_list.nDeviceNum == 0) {
    error_message_ = "No Hikrobot camera found";
    return false;
  }

  // 打开第一个设备
  ret = MV_CC_CreateHandle(&handle_, dev_list.pDeviceInfo[0]);
  if (ret != MV_OK) {
    error_message_ = "MV_CC_CreateHandle failed";
    return false;
  }

  ret = MV_CC_OpenDevice(handle_);
  if (ret != MV_OK) {
    error_message_ = "MV_CC_OpenDevice failed";
    MV_CC_DestroyHandle(handle_);
    handle_ = nullptr;
    return false;
  }

  // 设置连续采集模式
  MV_CC_SetEnumValue(handle_, "AcquisitionMode", 2);  // 2=Continuous
  MV_CC_SetEnumValue(handle_, "TriggerMode", 0);       // 0=Off

  // 应用宽高参数
  MV_CC_SetIntValue(handle_, "Width", width_);
  MV_CC_SetIntValue(handle_, "Height", height_);

  // 分配buf (RGB8: 3 bytes/pixel, Bayer: 1 byte/pixel. 取较大值)
  raw_buf_size_ = width_ * height_ * 3;
  raw_buf_ = new (std::nothrow) unsigned char[raw_buf_size_];
  if (!raw_buf_) {
    error_message_ = "Failed to alloc image buffer";
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    return false;
  }

  ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    error_message_ = "MV_CC_StartGrabbing failed";
    delete[] raw_buf_;
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
    return false;
  }

  is_open_ = true;
  error_message_.clear();
  return true;
}

bool HikrobotCam::close() {
  if (!is_open_) return true;
  MV_CC_StopGrabbing(handle_);
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  delete[] raw_buf_;
  raw_buf_ = nullptr;
  is_open_ = false;
  return true;
}

bool HikrobotCam::is_open() { return is_open_; }

bool HikrobotCam::grab_image(cv::Mat& image) {
  if (!is_open_) return false;

  MV_FRAME_OUT frame;
  memset(&frame, 0, sizeof(frame));

  int ret = MV_CC_GetImageBuffer(handle_, &frame, 200);
  if (ret != MV_OK) {
    // 超时静默跳过，其他错误记录
    return true;
  }

  // 转RGB → BGR
  MV_CC_PIXEL_CONVERT_PARAM convert;
  memset(&convert, 0, sizeof(convert));
  convert.nWidth = frame.stFrameInfo.nWidth;
  convert.nHeight = frame.stFrameInfo.nHeight;
  convert.pSrcData = frame.pBufAddr;
  convert.nSrcDataLen = frame.stFrameInfo.nFrameLen;
  convert.enSrcPixelType = frame.stFrameInfo.enPixelType;
  convert.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
  convert.pDstBuffer = raw_buf_;
  convert.nDstBufferSize = raw_buf_size_;

  ret = MV_CC_ConvertPixelType(handle_, &convert);
  if (ret == MV_OK) {
    RGB2BGR(raw_buf_, convert.nWidth, convert.nHeight);
    image = cv::Mat(convert.nHeight, convert.nWidth, CV_8UC3, raw_buf_)
                .clone();
  }

  MV_CC_FreeImageBuffer(handle_, &frame);
  return !image.empty();
}

bool HikrobotCam::SetTriggerMode(bool on) {
  return MV_CC_SetEnumValue(handle_, "TriggerMode",
                            on ? MV_TRIGGER_SOURCE_SOFTWARE : 0) == MV_OK;
}

bool HikrobotCam::set_parameter(CamParamType type, int value) {
  params_[type] = value;

  if (!is_open_ && type != CamParamType::Width &&
      type != CamParamType::Height) {
    return true;  // 未打开时缓存参数
  }

  switch (type) {
    case CamParamType::Width:
      width_ = value;
      break;
    case CamParamType::Height:
      height_ = value;
      break;
    case CamParamType::Exposure:
      MV_CC_SetFloatValue(handle_, "ExposureTime", value);
      break;
    case CamParamType::Gain:
      MV_CC_SetFloatValue(handle_, "Gain", value);
      break;
    case CamParamType::AutoExposure:
      MV_CC_SetEnumValue(handle_, "ExposureAuto",
                         value ? 2 : 0);  // 2=Continuous, 0=Off
      break;
    case CamParamType::AutoWhiteBalance:
      MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto",
                         value ? 1 : 0);  // 1=Continuous, 0=Off
      break;
    case CamParamType::Fps:
      MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
      MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", value);
      break;
    default:
      break;
  }
  return true;
}

bool HikrobotCam::get_parameter(CamParamType type, int& value) {
  auto it = params_.find(type);
  if (it != params_.end()) {
    value = it->second;
    return true;
  }
  return false;
}

}  // namespace camera
