#include "armor_aim/ballistics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace armor_aim
{

Ballistics::Ballistics(const float projectile_speed_pixels_per_second)
: projectile_speed_(projectile_speed_pixels_per_second) {}

std::optional<AimSolution> Ballistics::Solve(
  const cv::Point2f & shooter, const cv::Point2f & target,
  const cv::Point2f & target_velocity, const cv::Point2f & target_acceleration,
  const float control_latency_seconds) const
{
  // 拦截条件：目标在(补偿+飞行时间)后到炮台的距离=弹速*飞行时间。
  // 调小额外补偿不会取消对子弹飞行时间的预测。
  // Use bounded root search for the acceleration fit; keep the exact quadratic
  // solution below for the constant-velocity fallback.
  if (!std::isfinite(projectile_speed_) || projectile_speed_ <= 0.0F ||
    !std::isfinite(control_latency_seconds)) {return std::nullopt;}
  if (cv::norm(target_acceleration) > 1.0e-4) {
    auto position = [&](float time) {
        const float horizon = time + std::max(0.0F, control_latency_seconds);
        return target + target_velocity * horizon + target_acceleration * (0.5F * horizon * horizon);
      };
    auto residual = [&](float time) {
        return cv::norm(position(time) - shooter) - projectile_speed_ * time;
      };
    float lo = 0.0F;
    if (!std::isfinite(residual(lo)) || residual(lo) <= 0.0) {return std::nullopt;}
    // 4 seconds bounds prediction; no solution means no fire, not direct aim.
    for (int step = 1; step <= 400; ++step) {
      float hi = static_cast<float>(step) * 0.01F;
      if (residual(hi) <= 0.0) {
        for (int i = 0; i < 24; ++i) {
          const float middle = (lo + hi) * 0.5F;
          if (residual(middle) > 0.0) {lo = middle;} else {hi = middle;}
        }
        const float time = (lo + hi) * 0.5F;
        const cv::Point2f intercept = position(time);
        const cv::Point2f direction = intercept - shooter;
        return AimSolution{
          std::atan2(-direction.y, direction.x) * 180.0F / static_cast<float>(CV_PI),
          time, intercept};
      }
      lo = hi;
    }
    return std::nullopt;
  }
  const cv::Point2f delayed = target +
    target_velocity * std::max(0.0F, control_latency_seconds);
  const cv::Point2f relative = delayed - shooter;
  const double a = target_velocity.dot(target_velocity) -
    static_cast<double>(projectile_speed_) * projectile_speed_;
  const double b = 2.0 * relative.dot(target_velocity);
  const double c = relative.dot(relative);
  if (!std::isfinite(a + b + c) || c <= 1.0e-8) {return std::nullopt;}
  double time = std::numeric_limits<double>::infinity();
  if (std::abs(a) < 1.0e-8) {
    if (b < -1.0e-8) {time = -c / b;}
  } else {
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {return std::nullopt;}
    const double q = -0.5 * (b + std::copysign(std::sqrt(discriminant), b));
    for (const double root : {q / a, q != 0.0 ? c / q : -1.0}) {
      if (root > 0.0 && root < time) {time = root;}
    }
  }
  if (!std::isfinite(time)) {return std::nullopt;}
  const float flight_time = static_cast<float>(time);
  const cv::Point2f intercept = delayed + target_velocity * flight_time;
  const cv::Point2f direction = intercept - shooter;
  constexpr float kRadiansToDegrees = 180.0F / static_cast<float>(CV_PI);
  // The simulator image uses a downward-positive Y axis, while its serial
  // turret angle uses the conventional upward-positive Cartesian Y axis.
  const float angle = std::atan2(-direction.y, direction.x) * kRadiansToDegrees;
  return AimSolution{angle, flight_time, intercept};
}

}  // namespace armor_aim
