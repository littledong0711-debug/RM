#ifndef ARMOR_AIM__GAME_AIM_HPP_
#define ARMOR_AIM__GAME_AIM_HPP_

#include "armor_aim/ballistics.hpp"
#include "armor_aim/types.hpp"
#include <cmath>

namespace armor_aim
{
inline bool IsShootableTrack(
  const TrackedTarget & track, const float confirmation_seconds = 0.0F)
{
  // Keep missing tracks alive for association and friendly-fire protection,
  // but never aim at a target that was not observed in the current frame.
  // Five observations suppress the highly seed-dependent first velocity
  // estimate at a screen edge while adding only a few frames of latency.
  return track.missed_frames == 0 && track.hit_streak >= 5 &&
    (confirmation_seconds <= 0.0F ||
    (track.consecutive_hits >= 5 && track.consecutive_seconds >= confirmation_seconds));
}

// Simulator plates travel horizontally; image pairing jitter is not Y motion.
inline std::optional<AimSolution> SolveGameAim(
  const Ballistics & ballistics, const cv::Point2f & shooter,
  const TrackedTarget & track, const float latency, const cv::Size & screen,
  const float confirmation_seconds = 0.0F)
{
  if (!IsShootableTrack(track, confirmation_seconds)) {
    return std::nullopt;
  }
  const auto solution = ballistics.Solve(
    shooter, track.position, {track.velocity.x, 0.0F},
    {track.acceleration.x, 0.0F}, latency);
  if (!solution || !std::isfinite(solution->intercept.x) ||
    !std::isfinite(solution->intercept.y) ||
    solution->intercept.x < 0.0F || solution->intercept.y < 0.0F ||
    solution->intercept.x >= static_cast<float>(screen.width) ||
    solution->intercept.y >= static_cast<float>(screen.height))
  {
    return std::nullopt;
  }
  return solution;
}

inline float ShotRisk(
  const AimSolution & solution, const TrackedTarget & track,
  const float latency, const float projectile_speed, const bool locked)
{
  const float horizon = solution.flight_time_seconds + std::max(0.0F, latency);
  const float acceleration_displacement =
    0.5F * std::abs(track.acceleration.x) * horizon * horizon;
  const float uncertainty_time = projectile_speed > 0.0F ?
    acceleration_displacement / projectile_speed : 10.0F;
  // A small hysteresis bonus prevents rapid target switching, while allowing a
  // substantially shorter and less acceleration-sensitive shot to take over.
  return solution.flight_time_seconds + uncertainty_time - (locked ? 0.12F : 0.0F);
}
}  // namespace armor_aim
#endif
