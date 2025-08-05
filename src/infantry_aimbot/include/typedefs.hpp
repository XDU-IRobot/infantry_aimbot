
#pragma once

#include <opencv2/opencv.hpp>
#include <outcome.hpp>

namespace ia {

template <typename T>
using sp = std::shared_ptr<T>;
template <typename T>
using up = std::unique_ptr<T>;

using outcome_v2::result;
template <typename T>
using result_sp = result<sp<T>>;
template <typename T>
using result_up = result<up<T>>;

enum Color {
  RED = 0,
  BLUE,
};

struct LightBlob {
  LightBlob() = default;
  LightBlob(cv::RotatedRect box) {
    rrect = box;
    if (rrect.size.width > rrect.size.height) {
      std::swap<float>(rrect.size.width, rrect.size.height);
    }
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
    up = (p[0] + p[1]) / 2;
    down = (p[2] + p[3]) / 2;
    center = (up + down) / 2;
    length = cv::norm(up - down);
    width = cv::norm(p[0] - p[1]);
    angle = std::atan2(up.x - down.x, up.y - down.y);

    angle = angle / CV_PI * 180;
    if (angle < 0) {
      angle = -(angle + 180);
    } else {
      angle = 180 - angle;
    }
  }

  cv::Point2f up, down, center;
  double length;
  double width;
  cv::RotatedRect rrect;
  Color color;
  int matched_count;
  float angle;
};

struct Armor {
  enum Type {
    WRONG = 0,
    SMALL,
    BIG,
    RUNE_ARMOR,
    GRAY_BIG_ARMOR,
    GRAY_SMALL_ARMOR,
    DEAD_ARMOR,
  };

  Armor() = default;
  Armor(const LightBlob &l1, const LightBlob &l2)
      : points{left_light.down, left_light.up, right_light.up, right_light.down},
        center_point{(left_light.rrect.center + right_light.rrect.center) / 2} {
    if (l1.rrect.center.x < l2.rrect.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }

    // get rect
    const auto width = 15 + std::sqrt(std::pow(right_light.rrect.center.x - left_light.rrect.center.x, 2) +
                                      std::pow(right_light.rrect.center.y - left_light.rrect.center.y, 2));
    const auto height = 15 + MAX(left_light.rrect.size.height, right_light.rrect.size.height);
    const auto angle = std::atan2(right_light.rrect.center.y - left_light.rrect.center.y,
                                  right_light.rrect.center.x - left_light.rrect.center.x) *
                       180 / CV_PI;
    this->rrect = cv::RotatedRect(center_point, cv::Size(width, height), angle);
    this->rect = cv::Rect2d(center_point - cv::Point2d(width / 2.0, height / 2.0), cv::Size(width, height));
  }

  cv::RotatedRect rrect;
  cv::Rect2d rect;

  LightBlob left_light;
  LightBlob right_light;

  int num_id{10};
  Type type{WRONG};
  double distance_to_image_center;
  double distance;

  cv::Point2d center_point;

  double confidence;
  std::vector<cv::Point2d> points{4};

  double yaw;
  double pitch;
  double roll;

  cv::Point3f position;
  cv::Point3d pyr;
};
}  // namespace ia