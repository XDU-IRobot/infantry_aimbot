
#ifndef TYPEDEFS_HPP
#define TYPEDEFS_HPP

#include <opencv2/core.hpp>

namespace camera {

enum class CamParamType {
  Width,
  Height,
  AutoExposure,
  Exposure,
  Brightness,
  AutoWhiteBalance,
  WhiteBalance,
  Gain,
  RGain,
  GGain,
  BGain,
  Gamma,
  Contrast,
  Saturation,
  Hue,
  Fps
};

}  // namespace camera

#endif  // TYPEDEFS_HPP
