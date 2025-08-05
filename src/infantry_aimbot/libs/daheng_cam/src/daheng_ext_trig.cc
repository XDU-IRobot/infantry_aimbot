
#include "daheng_ext_trig.hpp"

namespace camera {

GXCaptureCallBack StdFunctionToGxCallback(
    std::function<void GX_STDC(GX_FRAME_CALLBACK_PARAM *)> std_func_object) {
  static auto static_func_object = std::move(std_func_object);
  return [](GX_FRAME_CALLBACK_PARAM *arg) { static_func_object(arg); };
}

DahengCamExtTrig::DahengCamExtTrig(const std::string camera_sn)
    : camera_sn_(camera_sn), device_(nullptr), pFrameBuffer_(nullptr) {
  // default param
  params_[CamParamType::Height] = 720;
  params_[CamParamType::Width] = 1280;
  params_[CamParamType::AutoExposure] = 0;      // 自动曝光 ，0为否
  params_[CamParamType::Exposure] = 2000;       // 曝光时间 单位 us
  params_[CamParamType::AutoWhiteBalance] = 0;  // 自动白平衡 ，0为否
  params_[CamParamType::RGain] = 19;            // 红通道增益
  params_[CamParamType::GGain] = 10;            // 绿通道增益
  params_[CamParamType::BGain] = 19;            // 蓝通道增益
  params_[CamParamType::Gamma] = 9;             // 伽马增益

  /* TODO */
  params_[CamParamType::Fps] = 100;
  params_[CamParamType::Brightness] = 0;
  params_[CamParamType::WhiteBalance] = 0;
  params_[CamParamType::Gain] = 0;
  params_[CamParamType::Hue] = 0;
  params_[CamParamType::Contrast] = 0;
  params_[CamParamType::Saturation] = 0;
  this->is_open_ = false;
  this->error_message_ = "";
}

DahengCamExtTrig::~DahengCamExtTrig() {
  if (is_open_) {
    bool if_delete = this->close();
  }
}

bool DahengCamExtTrig::is_open() { return is_open_; }

void DahengCamExtTrig::OnImageCallback(
    std::function<void(cv::Mat &)> callback) {
  on_image_callback_ = callback;
}

bool DahengCamExtTrig::open() {
  if (is_open_) {
    return true;
  }
  // init the DahengCam sdk
  if (!this->InitSdk()) {
    return false;
  }
  // set the parameter
  if (!this->SettingsInit()) {
    return false;
  }
  rgbImagebuf_ = new uint8_t[params_[CamParamType::Height] *
                             params_[CamParamType::Width] * 3];

  GX_STATUS status;
  // status = GXStreamOn(device_);
  status = GXSendCommand(device_, GX_COMMAND_ACQUISITION_START);  ///< 开采
  if (status != GX_STATUS_SUCCESS) {
    bool if_close = this->close();
    this->error_message_ = "DahengCam open failed ";
    return false;
  }
  this->is_open_ = true;
  return true;
}

bool DahengCamExtTrig::close() {
  GX_STATUS status;
  status = GXSendCommand(device_, GX_COMMAND_ACQUISITION_STOP);  ///< 停采
  // status = GXStreamOff(device_);
  if (this->rgbImagebuf_ != NULL) {
    delete[] this->rgbImagebuf_;
    this->rgbImagebuf_ = NULL;
  }
  status = GXUnregisterCaptureCallback(device_);
  status = GXCloseDevice(device_);
  status = GXCloseLib();
  if (status != GX_STATUS_SUCCESS) {
    this->error_message_ = "DahengCam close failed ";
    return false;
  }
  this->is_open_ = false;
  return true;
}

bool DahengCamExtTrig::set_parameter(CamParamType type, int value) {
  if (params_.find(type) != params_.end()) {
    params_[type] = value;
    return true;
  } else {
    this->error_message_ = "DahengCam set_parameter failed";
    return false;
  }
}

bool DahengCamExtTrig::ResetParameters(
    const std::map<CamParamType, int> &param_map) {
  this->close();
  for (auto it = param_map.begin(); it != param_map.end(); it++)
    this->set_parameter(it->first, it->second);
  return this->open();
}

bool DahengCamExtTrig::get_parameter(CamParamType type, int &value) {
  if (params_.find(type) != params_.end()) {
    value = params_[type];
    return true;
  } else {
    this->error_message_ = "DahengCam get_Parameter failed";
    return false;
  }
}

bool DahengCamExtTrig::InitSdk() {
  GX_STATUS status;
  // init the lib of DahengCam
  status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    GXCloseLib();
    this->error_message_ = "DahengCam initsdk InitLib failed ";
    return false;
  }

  // get device lists
  uint32_t ui32DeviceNum = 0;
  status = GXUpdateDeviceList(&ui32DeviceNum, 1000);
  if (status != GX_STATUS_SUCCESS || ui32DeviceNum <= 0) {
    GXCloseLib();
    this->error_message_ = "DahengCam initsdk getDeviceList failed ";

    return false;
  }

  /*TODO*/
  // Open device by sn
  status = GXOpenDeviceByIndex(1, &device_);  //  open the  device

  if (status != GX_STATUS_SUCCESS) {
    GXCloseLib();
    this->error_message_ = "DahengCam initsdk OpentDevice failed ";

    return false;
  }

  size_t nSize = 0;
  // get  vendor name
  GXGetStringLength(device_, GX_STRING_DEVICE_VENDOR_NAME, &nSize);
  char *pszVendorName = new char[nSize];
  GXGetString(device_, GX_STRING_DEVICE_VENDOR_NAME, pszVendorName, &nSize);
  delete[] pszVendorName;
  pszVendorName = NULL;

  // get nodel_name
  GXGetStringLength(device_, GX_STRING_DEVICE_MODEL_NAME, &nSize);
  char *pszModelName = new char[nSize];
  GXGetString(device_, GX_STRING_DEVICE_MODEL_NAME, pszModelName, &nSize);
  delete[] pszModelName;
  pszModelName = NULL;

  // get  serial_number
  GXGetStringLength(device_, GX_STRING_DEVICE_SERIAL_NUMBER, &nSize);
  char *pszSerialNumber = new char[nSize];
  GXGetString(device_, GX_STRING_DEVICE_SERIAL_NUMBER, pszSerialNumber, &nSize);
  delete[] pszSerialNumber;
  pszSerialNumber = NULL;

  // get version
  GXGetStringLength(device_, GX_STRING_DEVICE_VERSION, &nSize);
  char *pszDeviceVersion = new char[nSize];
  GXGetString(device_, GX_STRING_DEVICE_VERSION, pszDeviceVersion, &nSize);
  delete[] pszDeviceVersion;
  pszDeviceVersion = NULL;

  // set the model and the buffer num
  status =
      GXSetEnum(device_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_SINGLE_FRAME);
  status = GXSetEnum(device_, GX_ENUM_TRIGGER_MODE,
                     GX_TRIGGER_MODE_ON);  ///< 外部触发模式
  status = GXSetEnum(device_, GX_ENUM_TRIGGER_ACTIVATION,
                     GX_TRIGGER_ACTIVATION_RISINGEDGE);  ///< 上升沿触发
  status = GXSetEnum(device_, GX_ENUM_TRIGGER_SOURCE,
                     GX_TRIGGER_SOURCE_LINE2);  ///< 设置触发开关为line2
  status =
      GXSetEnum(device_, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE2);
  status = GXSetEnum(device_, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT);
  status = GXRegisterCaptureCallback(
      device_, NULL,
      StdFunctionToGxCallback([this](GX_FRAME_CALLBACK_PARAM *arg) {
        this->OnCaptureFrameCallback(arg);
      }));

  status = GXSetAcqusitionBufferNumber(device_, ACQ_BUFFER_NUM);
  return true;
}

bool DahengCamExtTrig::SettingsInit() {
  GX_STATUS status;
  status = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
  status = GXSetInt(device_, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB,
                    ACQ_TRANSFER_NUMBER_URB);
  status = GXSetInt(device_, GX_INT_WIDTH, params_[CamParamType::Width]);
  status = GXSetInt(device_, GX_INT_HEIGHT, params_[CamParamType::Height]);
  status = GXSetInt(device_, GX_INT_OFFSET_X,
                    (1920 - params_[CamParamType::Width]) / 2);
  status = GXSetInt(device_, GX_INT_OFFSET_Y,
                    (1200 - params_[CamParamType::Height]) / 2);

  // set Exposure
  if (params_[CamParamType::AutoExposure]) {
    status =
        GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, 5000);
    status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, 20000);
  } else {
    status = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN,
                        params_[CamParamType::Exposure] - 200);
    status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX,
                        params_[CamParamType::Exposure] + 200);
    status = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME,
                        params_[CamParamType::Exposure]);
  }

  // set AutoWhiteBalance Gain
  if (params_[CamParamType::AutoWhiteBalance]) {
    status = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO,
                       GX_BALANCE_WHITE_AUTO_CONTINUOUS);
  } else {
    status = GXSetEnum(device_, GX_ENUM_BALANCE_WHITE_AUTO,
                       GX_BALANCE_WHITE_AUTO_OFF);
    status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_RED);
    status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO,
                        params_[CamParamType::RGain] / 10.0);
    status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_GREEN);
    status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO,
                        params_[CamParamType::GGain] / 10.0);
    status = GXSetEnum(device_, GX_ENUM_BALANCE_RATIO_SELECTOR,
                       GX_BALANCE_RATIO_SELECTOR_BLUE);
    status = GXSetFloat(device_, GX_FLOAT_BALANCE_RATIO,
                        params_[CamParamType::BGain] / 10.0);
  }

  // Set Gamma
  status = GXSetEnum(device_, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
  status = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MIN, 0);
  status = GXSetFloat(device_, GX_FLOAT_AUTO_GAIN_MAX, 10);
  status = GXSetFloat(device_, GX_FLOAT_GAIN, params_[CamParamType::Gamma]);

  if (status != GX_STATUS_SUCCESS) {
    this->error_message_ = "DahengCam setInit failed ";
    return false;
  }
  return true;
}

void DahengCamExtTrig::SetExposure(int exposure) {
  GX_STATUS status;

  params_[CamParamType::Exposure] = exposure;
  status = GXSetEnum(device_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);

  status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN,
                      params_[CamParamType::Exposure] - 200);
  status = GXSetFloat(device_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX,
                      params_[CamParamType::Exposure] + 200);
  status = GXSetFloat(device_, GX_FLOAT_EXPOSURE_TIME,
                      params_[CamParamType::Exposure]);

  // status_ = GXStreamOn(device_);
}

void GX_STDC
DahengCamExtTrig::OnCaptureFrameCallback(GX_FRAME_CALLBACK_PARAM *pFrame) {
  if (pFrame->status == 0) {
    if (pFrame->nWidth != image_buffer_.cols ||
        pFrame->nHeight != image_buffer_.rows) {
      image_buffer_ =
          cv::Mat(pFrame->nHeight, pFrame->nWidth, CV_8UC3, cv::Scalar::all(0));
    }
    DxRaw8toRGB24((void *)pFrame->pImgBuf, image_buffer_.data, pFrame->nWidth,
                  pFrame->nHeight, RAW2RGB_NEIGHBOUR, BAYERRG, false);
    if (on_image_callback_) {
      on_image_callback_(image_buffer_);
    }
  } else {
    std::cerr << "[daheng] trigger callback failed\n";
  }
  return;
}

}  // namespace camera