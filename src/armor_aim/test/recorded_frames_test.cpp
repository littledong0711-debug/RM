#include "armor_aim/armor_detector.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iostream>
#include <string>

// Ground truth is specific to the inspected 20260921-111048 capture:
// red launcher, no blue enemies, two red friendly plates in frame 11.
// This test is read-only with respect to the running game and has no serial IO.
int main(int argc, char ** argv)
{
  if (argc != 3) {return 2;}
  // 只读源码中的固定输入，标注图写入构建目录，避免测试污染Git工作区。
  const std::filesystem::path output_directory(argv[2]);
  std::filesystem::create_directories(output_directory);
  armor_aim::ArmorDetector detector;
  bool passed = true;
  for (int i = 0; i < 12; ++i) {
    const std::string index = (i < 10 ? "0" : "") + std::to_string(i);
    const auto path = std::filesystem::path(argv[1]) / ("frame_" + index + "_RGBA.png");
    cv::Mat image = cv::imread(path.string());
    if (image.empty()) {
      std::cerr << "Missing captured frame: " << path << '\n';
      return 1;
    }
    const auto launcher = detector.DetectLauncher(image);
    const auto enemies = detector.Detect(image, armor_aim::TeamColor::kBlue);
    const auto friends = detector.Detect(image, armor_aim::TeamColor::kRed);
    const bool launcher_ok = launcher.has_value() &&
      launcher->color == armor_aim::TeamColor::kRed &&
      cv::norm(launcher->center - cv::Point2f{576.0F, 612.0F}) < 6.0;
    const bool frame_ok = launcher_ok && enemies.empty() && (i != 11 || friends.size() == 2U);
    passed = passed && frame_ok;
    std::cout << "Frame " << index << ": launcher=" << (launcher_ok ? "OK" : "FAIL") <<
      " enemies=" << enemies.size() << " friends=" << friends.size() << '\n';
    if (launcher.has_value()) {
      cv::circle(image, launcher->center, 25, cv::Scalar(0, 255, 0), 2);
    }
    for (const auto & friendly : friends) {
      cv::rectangle(image, friendly.box, cv::Scalar(0, 255, 0), 2);
    }
    for (const auto & enemy : enemies) {
      cv::rectangle(image, enemy.box, cv::Scalar(0, 0, 255), 2);
    }
    if (!cv::imwrite((output_directory /
      ("verified_" + index + ".png")).string(), image)) {return 1;}
  }
  return passed ? 0 : 1;
}
