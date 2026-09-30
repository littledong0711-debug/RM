#ifndef ARMOR_AIM__BALLISTICS_HPP_
#define ARMOR_AIM__BALLISTICS_HPP_

#include <opencv2/core/types.hpp>

#include <optional>

namespace armor_aim
{

struct AimSolution
{
  float angle_degrees{0.0F};
  float flight_time_seconds{0.0F};
  cv::Point2f intercept{};
};

class Ballistics
{
public:
  explicit Ballistics(float projectile_speed_pixels_per_second = 600.0F);

  std::optional<AimSolution> Solve(
    const cv::Point2f & shooter, const cv::Point2f & target,
    const cv::Point2f & target_velocity,
    const cv::Point2f & target_acceleration = cv::Point2f{},
    float control_latency_seconds = 0.0F) const;

private:
  float projectile_speed_;
};

}  // namespace armor_aim

#endif  // ARMOR_AIM__BALLISTICS_HPP_
