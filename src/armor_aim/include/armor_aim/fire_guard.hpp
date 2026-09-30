#ifndef ARMOR_AIM__FIRE_GUARD_HPP_
#define ARMOR_AIM__FIRE_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

namespace armor_aim
{

struct GuardMotion
{
  // 运动量以像素/秒计，框为半宽半高；uncertainty_speed控制误差范围增长。
  double x;
  double y;
  double vx;
  double vy;
  double ax;
  double ay;
  double half_width;
  double half_height;
  double uncertainty_speed;
};

inline double ScreenFlightTime(
  const double x, const double y, const double vx, const double vy,
  const double width, const double height)
{
  double time = std::numeric_limits<double>::infinity();
  if (vx > 1.0e-6) {
    time = std::min(time, (width - x) / vx);
  }
  if (vx < -1.0e-6) {
    time = std::min(time, -x / vx);
  }
  if (vy > 1.0e-6) {
    time = std::min(time, (height - y) / vy);
  }
  if (vy < -1.0e-6) {
    time = std::min(time, -y / vy);
  }
  return std::max(0.0, time);
}

inline bool FriendlyCrossesShot(
  const double shooter_x, const double shooter_y,
  const double bullet_vx, const double bullet_vy,
  const double flight_time, const double latency,
  const GuardMotion & friendly)
{
  if (!std::isfinite(flight_time) || flight_time <= 0.0 || flight_time > 10.0) {
    return true;
  }
  for (const double value : {
      shooter_x, shooter_y, bullet_vx, bullet_vy, latency,
      friendly.x, friendly.y, friendly.vx, friendly.vy, friendly.ax, friendly.ay,
      friendly.half_width, friendly.half_height, friendly.uncertainty_speed})
  {
    if (!std::isfinite(value)) {
      return true;
    }
  }
  if (friendly.half_width < 0.0 || friendly.half_height < 0.0 ||
    friendly.uncertainty_speed < 0.0)
  {
    return true;
  }

  constexpr double kStepSeconds = 0.005;
  // 检查完整离屏弹道：采样加扫掠裕量减少间隙漏判，但无法弥补视觉漏检。
  const int steps = std::max(
    1, static_cast<int>(std::ceil(flight_time / kStepSeconds)));
  const double step = flight_time / static_cast<double>(steps);
  for (int index = 0; index <= steps; ++index) {
    const double time = step * static_cast<double>(index);
    const double horizon = time + std::max(0.0, latency);
    const double delta_x = friendly.x + friendly.vx * horizon +
      0.5 * friendly.ax * horizon * horizon - (shooter_x + bullet_vx * time);
    const double delta_y = friendly.y + friendly.vy * horizon +
      0.5 * friendly.ay * horizon * horizon - (shooter_y + bullet_vy * time);
    const double sweep_x =
      (std::abs(friendly.vx + friendly.ax * horizon - bullet_vx) +
      std::abs(friendly.ax) * step) * step * 0.5;
    const double sweep_y =
      (std::abs(friendly.vy + friendly.ay * horizon - bullet_vy) +
      std::abs(friendly.ay) * step) * step * 0.5;
    const double margin = 8.0 + friendly.uncertainty_speed * horizon;
    if (std::abs(delta_x) <= friendly.half_width + margin + sweep_x &&
      std::abs(delta_y) <= friendly.half_height + margin + sweep_y)
    {
      return true;
    }
  }
  return false;
}

// Conservative game mode: veto if CA, CV, or the current-position hypothesis
// intersects the shot. These are alternatives, not a claim to bound all motion.
inline bool FriendlyCrossesShotConservative(
  const double shooter_x, const double shooter_y,
  const double bullet_vx, const double bullet_vy,
  const double flight_time, const double latency, GuardMotion friendly,
  const bool balanced = false)
{
  // First preserve every veto from the original guard, including invalid data.
  if (FriendlyCrossesShot(shooter_x, shooter_y, bullet_vx, bullet_vy,
      flight_time, latency, friendly)) {return true;}
  friendly.uncertainty_speed = std::max(balanced ? 40.0 : 80.0, friendly.uncertainty_speed);
  const auto blocked = [&](const GuardMotion & motion) {
      return FriendlyCrossesShot(shooter_x, shooter_y, bullet_vx, bullet_vy,
        flight_time, latency, motion);
    };
  if (blocked(friendly)) {return true;}
  friendly.ax = 0.0;
  friendly.ay = 0.0;
  if (blocked(friendly)) {return true;}
  // Balanced mode keeps CA and CV vetoes but does not assume a mature moving
  // friendly will suddenly stop. Low-confidence tracks are widened by caller.
  if (balanced) {return false;}
  friendly.vx = 0.0;
  friendly.vy = 0.0;
  return blocked(friendly);
}

}  // namespace armor_aim

#endif  // ARMOR_AIM__FIRE_GUARD_HPP_
