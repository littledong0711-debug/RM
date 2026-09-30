#ifndef ARMOR_AIM__ARMOR_DETECTOR_HPP_
#define ARMOR_AIM__ARMOR_DETECTOR_HPP_

#include "armor_aim/types.hpp"

#include <opencv2/core/mat.hpp>

#include <optional>
#include <vector>

namespace armor_aim
{

class ArmorDetector
{
public:
  struct Launcher
  {
    TeamColor color{TeamColor::kUnknown};
    cv::Point2f center{};
    double area{0.0};
  };

  struct Parameters
  {
    int color_difference_threshold{100};
    int minimum_brightness{100};
    double minimum_area{12.0};
    double maximum_area{3500.0};
    double minimum_aspect_ratio{0.55};
    double maximum_aspect_ratio{8.0};
  };

  ArmorDetector();
  explicit ArmorDetector(Parameters parameters);

  std::optional<Launcher> DetectLauncher(const cv::Mat & image) const;
  std::vector<Detection> Detect(const cv::Mat & image, TeamColor target_color) const;

private:
  cv::Mat MakeColorMask(const cv::Mat & image, TeamColor color) const;

  Parameters parameters_;
};

}  // namespace armor_aim

#endif  // ARMOR_AIM__ARMOR_DETECTOR_HPP_
