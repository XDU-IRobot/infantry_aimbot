
#include "detection/traditional_detector.hpp"

#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tbb/tbb.h>

#include "ros/node.hpp"

namespace ia {
namespace detection {
TraditionalDetector::TraditionalDetector(ProcessParams p_params, LightParams l_params, ArmorParams a_params)
    : process_params_(p_params),
      light_params_(l_params),
      armor_params_(a_params),
      enemy_color_((Color)p_params.enemy_color) {}

result_sp<std::vector<Armor>> TraditionalDetector::DetectArmors(const cv::Mat &img) {
  if (img.empty()) {
    return std::make_error_code(std::errc::invalid_argument);
  }

  src_ = img;
  const auto lights = FindLights(img);
  if (!lights) {
    return std::make_error_code(std::errc::no_message);
  }

  return MatchLights(*lights.value());
}

cv::Point TraditionalDetector::FindMaxBrightnessChange(const cv::Mat &img, cv::Point start, cv::Point end) {
  // Calculate the number of samples along the line
  int num_samples = std::max(abs(end.x - start.x), abs(end.y - start.y));

  // Variables to store the maximum brightness change and the corresponding point
  double max_brightness_change = 0;
  cv::Point max_change_point = start;

  // Sample points along the line
  for (int i = 0; i <= num_samples; ++i) {
    // Calculate the interpolation factor
    double alpha = static_cast<double>(i) / num_samples;

    // Interpolate the point along the line
    int x = static_cast<int>(start.x * (1 - alpha) + end.x * alpha);
    int y = static_cast<int>(start.y * (1 - alpha) + end.y * alpha);

    // Ensure the point is within the image bounds
    if (x >= 0 && x < img.cols && y >= 0 && y < img.rows) {
      // Get the brightness at the current point and the next point
      int current_brightness = img.at<uchar>(y, x);
      int next_brightness = 0;

      // Check if we can access the next point along the line
      if (i < num_samples) {
        int next_x =
            static_cast<int>(start.x * (1 - (alpha + 1.0 / num_samples)) + end.x * (alpha + 1.0 / num_samples));
        int next_y =
            static_cast<int>(start.y * (1 - (alpha + 1.0 / num_samples)) + end.y * (alpha + 1.0 / num_samples));

        if (next_x >= 0 && next_x < img.cols && next_y >= 0 && next_y < img.rows) {
          next_brightness = img.at<uchar>(next_y, next_x);
        }
      }

      // Calculate the brightness change
      double brightness_change = std::abs(current_brightness - next_brightness);

      // Update the maximum brightness change and the corresponding point if needed
      if (brightness_change > max_brightness_change) {
        max_brightness_change = brightness_change;
        max_change_point = cv::Point(x, y);
      }
    }
  }

  return max_change_point;
}

result_sp<tbb::concurrent_vector<LightBlob>> TraditionalDetector::FindLights(const cv::Mat &img) {
  auto lights = std::make_shared<tbb::concurrent_vector<LightBlob>>();
  lights->reserve(100);  ///< 预留空间，防止后面开始处理之后push_back重复allocate

  // split rgb
  static std::vector<cv::Mat> bgr_splitted(3);
  cv::split(img, bgr_splitted);
  // 根据敌人颜色选择红通道或者蓝通道
  const cv::Mat &target_channel = (enemy_color_ == Color::RED) ? bgr_splitted[2] : bgr_splitted[0];

  static auto binary = [] {
    cv_bridge::CvImage _;
    _.encoding = sensor_msgs::image_encodings::MONO8;
    return _;
  }();

  cv::threshold(target_channel, binary.image, process_params_.bin_threshold, 255, cv::THRESH_BINARY);
  // ros::NodeSingleton::GetInstance().Publish<sensor_msgs::msg::CompressedImage>("img_binary",
  //                                                                              binary.toCompressedImageMsg());

  /*-----------寻找并筛选灯条轮廓-----------*/
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary.image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  tbb::parallel_for(std::size_t(0), contours.size(), [&](size_t idx) {
    const auto &contour = contours[idx];
    if (cv::contourArea(contour) < 9) {
      return;  // 去掉太小的轮廓
    }

    const auto b_rect = cv::boundingRect(contour);
    const auto r_rect = cv::minAreaRect(contour);
    if (cv::contourArea(contour) / r_rect.size.area() < light_params_.size_area_min_ratio) {
      return;
    }
    LightBlob light(r_rect);

    if (IsValidLight(light)) {
      // 计算灯条颜色
      // int sum_r = 0, sum_b = 0;
      // for (const auto &point : contour) {
      //   sum_b += img.at<cv::Vec3b>(point.y, point.x)[0];
      //   sum_r += img.at<cv::Vec3b>(point.y, point.x)[2];
      // }
      // if (std::abs(sum_r - sum_b) / static_cast<int>(contour.size()) > 20) {
      //   light.color = sum_r > sum_b ? Color::RED : Color::BLUE;
      // }
      light.color = enemy_color_;

      // 拟合一条沿着灯条方向的直线，在这条直线上找最大亮度变化点，作为这个灯条的两个角点
      if (light_params_.is_corner_correct) {
        cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
        std::vector<cv::Point> mask_contour;
        for (const auto &p : contour) {
          mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
        }
        cv::fillPoly(mask, {mask_contour}, 255);
        std::vector<cv::Point> points;
        cv::findNonZero(mask, points);
        // points / rotated rect area
        cv::Vec4f return_param;  // 点斜式表示直线
        cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
        cv::Point2f top, bottom;
        double angle_k;
        if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
          top = cv::Point2f(b_rect.x + b_rect.width / 2.f, b_rect.y);
          bottom = cv::Point2f(b_rect.x + b_rect.width / 2.f, b_rect.y + b_rect.height);
          angle_k = 0;
        } else {
          auto k = return_param[1] / return_param[0];
          auto b = (return_param[3] + b_rect.y) - k * (return_param[2] + b_rect.x);
          top = cv::Point2f((b_rect.y - b) / k, b_rect.y);
          bottom = cv::Point2f((b_rect.y + b_rect.height - b) / k, b_rect.y + b_rect.height);
        }
        light.up = FindMaxBrightnessChange(target_channel(b_rect), top, (top + bottom) / 2);
        light.down = FindMaxBrightnessChange(target_channel(b_rect), bottom, (top + bottom) / 2);
      }

      // if (enemy_color_ == light.color) {
      lights->push_back(light);
      // }
    }
  });

  if (lights->size() < 2) {
    return std::make_error_code(std::errc::no_message);
  }
  return lights;
}

bool TraditionalDetector::IsValidLight(const LightBlob &light) {
  if (abs(light.angle) > light_params_.angle_to_vertigal_max) {
    return false;
  }
  if (light.length / light.width < light_params_.height_width_min_ratio) {
    return false;
  }

  return true;
}

result_sp<std::vector<Armor>> TraditionalDetector::MatchLights(tbb::concurrent_vector<LightBlob> &lights) {
  auto ret = std::make_shared<std::vector<Armor>>();
  const auto comp = [](const LightBlob &a, const LightBlob &b) { return a.rrect.center.x < b.rrect.center.x; };

  // 从左到右依次查找是装甲板的灯条对
  tbb::parallel_sort(lights.begin(), lights.end(), comp);
  for (int i = 0; i < lights.size() - 1; i++) {
    for (int j = i + 1; j < lights.size(); j++) {
      int match_flag = 1;
      const auto &[is_armor, type] = IsValidArmor(lights[i], lights[j]);
      if (!is_armor) {
        continue;
      }
      const float x_min =
          lights[i].rrect.center.x < lights[j].rrect.center.x ? lights[i].rrect.center.x : lights[j].rrect.center.x;
      const float x_max =
          lights[i].rrect.center.x > lights[j].rrect.center.x ? lights[i].rrect.center.x : lights[j].rrect.center.x;
      const float y_min = lights[i].up.y < lights[j].up.y ? lights[i].up.y : lights[j].up.y;
      const float y_max = lights[i].down.y > lights[j].down.y ? lights[i].down.y : lights[j].down.y;
      for (int m = i + 1; m < j; m++) {
        const float light_center_x = lights[m].rrect.center.x;
        const float light_center_y = lights[m].rrect.center.y;
        if ((light_center_x > x_min && light_center_x < x_max) && (light_center_y > y_min && light_center_y < y_max)) {
          match_flag = 0;
          break;
        }
      }
      if (match_flag == 1) {
        lights[i].matched_count++;
        lights[j].matched_count++;
        Armor temp(lights[i], lights[j]);
        temp.type = type;
        ret->push_back(temp);
      } else {
        continue;
      }
    }
  }

  return ret;
}

auto TraditionalDetector::IsValidArmor(const LightBlob &light_1,
                                       const LightBlob &light_2) -> std::pair<bool,  ///< 这一对灯条是不是装甲板
                                                                              Armor::Type>  ///< 大装甲还是小装甲
{
  const float width = light_2.rrect.center.x - light_1.rrect.center.x;
  const float height = (light_2.length + light_1.length) / 2.0;
  const float angle_i_org = light_1.angle;
  const float angle_j_org = light_2.angle;
  const float avg_light_length = (light_1.length + light_2.length) / 2;
  const float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;

  bool center_distance_ok =
      (0.8 <= center_distance && center_distance < 3.2) || (1.8 <= center_distance && center_distance < 6.4);
  if (!center_distance_ok) {
    return std::make_pair(false, Armor::Type::DEFAULT);
  }

  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  if (angle > 30) {  ///< 倾斜角度太大
    return std::make_pair(false, Armor::Type::DEFAULT);
  }

  // 匹配灯条：角度差、长度比、高度差、组成的宽高比，滤除/\ \/
  if (abs(angle_i_org - angle_j_org) > armor_params_.lights_angle_max_diff) {  ///< 两个灯条的角度差别太大
    return std::make_pair(false, Armor::Type::DEFAULT);
  }

  if (std::max(light_1.length, light_2.length) / std::min(light_1.length, light_2.length) >
      armor_params_.lights_length_max_ratio) {  ///< 两个灯条的长度差别太大
    return std::make_pair(false, Armor::Type::DEFAULT);
  }

  if (width / height > armor_params_.width_height_max_ratio ||
      width / height < armor_params_.width_height_min_ratio) {  ///< 灯条围成的装甲板长宽比不对
    return std::make_pair(false, Armor::Type::DEFAULT);
  }

  Armor temp(light_1, light_2);
  if (fabs(temp.rrect.angle) > armor_params_.max_angle) {  ///< 灯条围成的装甲板倾斜角度太大
    return std::make_pair(false, Armor::Type::DEFAULT);
  }

  // 滤除类如窗户形成的伪装甲板，即装甲板区域过亮
  cv::Mat temp_inarmor = src_(temp.rect & cv::Rect2d(cv::Point(0, 0), src_.size()));
  cv::cvtColor(temp_inarmor, temp_inarmor, cv::COLOR_BGR2GRAY);
  if (cv::mean(temp_inarmor)[0] > armor_params_.inside_thresh) {
    return std::make_pair(false, Armor::Type::DEFAULT);
  }

  // 利用估算的距离粗略估计是大还是小装甲板，方便下一步进行pnp解算，pnp解算完成之后进行的数字分类会修正一次这个结果
  return std::make_pair(true, center_distance > 3.2 ? Armor::Type::BIG : Armor::Type::SMALL);
}

void TraditionalDetector::SetEnemyColor(Color enemy_color) {
  assert(enemy_color == Color::RED || enemy_color == Color::BLUE);
  enemy_color_ = enemy_color;
}
}  // namespace detection
}  // namespace ia
