#include "armor_aim/armor_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace armor_aim
{

ArmorDetector::ArmorDetector()
: ArmorDetector(Parameters{}) {}

ArmorDetector::ArmorDetector(Parameters parameters)
: parameters_(std::move(parameters)) {}

cv::Mat ArmorDetector::MakeColorMask(const cv::Mat & image, const TeamColor color) const
{
  // 输入为BGR；同时排除绿色较强的区域，避免浅黄色背景被误认为红色。
  std::vector<cv::Mat> channels;
  cv::split(image, channels);
  cv::Mat difference;
  cv::Mat brightness;
  if (color == TeamColor::kRed) {
    cv::subtract(channels[2], channels[0], difference);
    brightness = channels[2];
  } else {
    cv::subtract(channels[0], channels[2], difference);
    brightness = channels[0];
  }

  cv::Mat color_mask;
  cv::Mat brightness_mask;
  cv::threshold(
    difference, color_mask, parameters_.color_difference_threshold, 255, cv::THRESH_BINARY);
  cv::threshold(
    brightness, brightness_mask, parameters_.minimum_brightness, 255, cv::THRESH_BINARY);
  cv::bitwise_and(color_mask, brightness_mask, color_mask);
  // Yellow has strong red AND green. Require dominance over green as well.
  cv::Mat green_difference;
  cv::subtract(brightness, channels[1], green_difference);
  cv::Mat purity_mask;
  cv::threshold(green_difference, purity_mask, 35, 255, cv::THRESH_BINARY);
  cv::bitwise_and(color_mask, purity_mask, color_mask);
  return color_mask;
}

std::optional<ArmorDetector::Launcher> ArmorDetector::DetectLauncher(const cv::Mat & image) const
{
  if (image.empty()) {
    return std::nullopt;
  }
  // The launcher is permanently located in the lowest strip. Restricting the
  // search prevents falling armor plates from being mistaken for the launcher.
  const int region_top = static_cast<int>(static_cast<double>(image.rows) * 0.86);
  const cv::Rect bottom_region{0, region_top, image.cols, image.rows - region_top};

  std::optional<Launcher> best;
  for (const TeamColor color : {TeamColor::kRed, TeamColor::kBlue}) {
    cv::Mat mask = MakeColorMask(image(bottom_region), color);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size{5, 5});
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto & contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < 80.0 || (best.has_value() && area <= best->area)) {
        continue;
      }
      const cv::Rect bounds = cv::boundingRect(contour);
      if (bounds.width < 12 || bounds.height < 12 ||
        bounds.width > image.cols / 5 || bounds.height > image.rows / 5) {
        continue;
      }
      // The thick circular base is the pivot; the barrel biases a centroid.
      cv::Mat component = cv::Mat::zeros(mask.size(), CV_8UC1);
      cv::drawContours(component, std::vector<std::vector<cv::Point>>{contour},
        0, cv::Scalar(255), cv::FILLED);
      cv::Mat distance;
      cv::distanceTransform(component, distance, cv::DIST_L2, 5);
      double radius = 0.0;
      cv::Point pivot;
      cv::minMaxLoc(distance, nullptr, &radius, nullptr, &pivot);
      if (radius < 5.0) {
        continue;
      }
      best = Launcher{
        color,
        cv::Point2f{
          static_cast<float>(pivot.x),
          static_cast<float>(pivot.y) + static_cast<float>(region_top)},
        area};
    }
  }
  return best;
}

std::vector<Detection> ArmorDetector::Detect(
  const cv::Mat & image, const TeamColor target_color) const
{
  std::vector<Detection> detections;
  // 配对同色竖直灯条；没有独立灰块检测。颜色消失不能直接证明被击毁。
  if (image.empty() || target_color == TeamColor::kUnknown) {
    return detections;
  }

  cv::Mat mask = MakeColorMask(image, target_color);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<cv::Rect> bars;
  for (const auto & contour : contours) {
    const cv::Rect bar = cv::boundingRect(contour);
    if (bar.height >= 6 && bar.width <= bar.height && bar.area() >= 6 &&
      bar.y + bar.height < static_cast<int>(image.rows * 0.90)) {
      bars.push_back(bar);
    }
  }
  std::sort(bars.begin(), bars.end(), [](const cv::Rect & a, const cv::Rect & b) {
    return a.x < b.x;
  });
  std::vector<bool> used(bars.size(), false);
  for (std::size_t i = 0; i < bars.size(); ++i) {
    if (used[i]) {continue;}
    std::size_t best = bars.size();
    float best_cost = 1.0e9F;
    for (std::size_t j = i + 1; j < bars.size(); ++j) {
      if (used[j]) {continue;}
      const float height = static_cast<float>(std::max(bars[i].height, bars[j].height));
      const float dy = std::abs(static_cast<float>(bars[i].y - bars[j].y) +
        0.5F * static_cast<float>(bars[i].height - bars[j].height));
      const float gap = static_cast<float>(bars[j].x - bars[i].x);
      if (dy > height * 0.3F || gap < height * 0.8F || gap > height * 5.0F ||
        std::min(bars[i].height, bars[j].height) < height * 0.6F) {continue;}
      const float cost = gap + 4.0F * dy;
      if (cost < best_cost) {best = j; best_cost = cost;}
    }
    if (best == bars.size()) {continue;}
    const cv::Rect box = bars[i] | bars[best];
    if (box.area() > parameters_.maximum_area) {continue;}
    used[i] = true;
    used[best] = true;
    detections.push_back(Detection{
      cv::Point2f{static_cast<float>(box.x) + box.width * 0.5F,
        static_cast<float>(box.y) + box.height * 0.5F}, cv::Rect2f(box), 1.0F});
  }
  return detections;
}

}  // namespace armor_aim
