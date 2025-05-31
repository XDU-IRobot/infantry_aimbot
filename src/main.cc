
#include "daheng_cam/daheng.hpp"

camera::DahengCam cam;

int main() {
  cv::Mat img;
  if (!cam.open()) {
    std::cout << cam.error_message() << std::endl;
    return -1;
  }
  for (int i = 0; i < 3000; i++) {
    if (!cam.grab_image(img)) {
      std::cout << cam.error_message() << std::endl;
      return -1;
    }
    cv::imshow("DahengCam", img);
    cv::waitKey(1);
  }
  return 0;
}