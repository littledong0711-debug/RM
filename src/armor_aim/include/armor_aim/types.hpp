#ifndef ARMOR_AIM__TYPES_HPP_
#define ARMOR_AIM__TYPES_HPP_

#include <opencv2/core/types.hpp>

#include <cstdint>
#include <string>

namespace armor_aim
{

enum class TeamColor : std::uint8_t { kUnknown, kRed, kBlue };

inline TeamColor OppositeColor(const TeamColor color)
{
  if (color == TeamColor::kRed) {
    return TeamColor::kBlue;
  }
  if (color == TeamColor::kBlue) {
    return TeamColor::kRed;
  }
  return TeamColor::kUnknown;
}

inline std::string ToString(const TeamColor color)
{
  switch (color) {
    case TeamColor::kRed:
      return "red";
    case TeamColor::kBlue:
      return "blue";
    default:
      return "unknown";
  }
}

struct Detection
{
  cv::Point2f center{};
  cv::Rect2f box{};
  float confidence{0.0F};
};

struct TrackedTarget
{
  int id{-1};
  cv::Point2f position{};
  cv::Point2f velocity{};
  cv::Point2f acceleration{};
  cv::Rect2f box{};
  float confidence{0.0F};
  int hit_streak{0};
  // hit_streak实际为累计观测次数，不是连续次数。
  int missed_frames{0};
  int consecutive_hits{0};
  // consecutive_*用于连续确认，漏检或长时间无图像会重新计数。
  float consecutive_seconds{0.0F};
};

}  // namespace armor_aim

#endif  // ARMOR_AIM__TYPES_HPP_
