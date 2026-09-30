#include "armor_aim/fire_guard.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <random>

int main()
{
  using armor_aim::FriendlyCrossesShot;
  using armor_aim::GuardMotion;
  int failures = 0;
  const auto check = [&failures](const bool condition, const char * name) {
      if (!condition) {
        std::cerr << "FAIL: " << name << '\n';
        ++failures;
      }
    };
  const auto blocked = [](const GuardMotion & motion) {
      return FriendlyCrossesShot(576.0, 612.0, 0.0, -600.0, 1.02, 0.0, motion);
    };

  check(blocked({576, 400, 0, 0, 0, 0, 32, 16, 0}), "stationary friendly in front");
  check(!blocked({800, 400, 0, 0, 0, 0, 32, 16, 0}), "clear distant friendly");
  check(blocked({376, 312, 400, 0, 0, 0, 32, 16, 0}), "moving friendly crosses shot");
  check(blocked({376, 312, 0, 0, 1600, 0, 32, 16, 0}), "accelerating friendly crosses");
  check(blocked({576, 100, 0, 0, 0, 0, 32, 16, 0}), "friendly beyond target still blocks");
  check(blocked({650, 312, 0, 0, 0, 0, 32, 16, 100}), "uncertain track expands corridor");
  check(!blocked({376, 312, -400, 0, 0, 0, 32, 16, 0}), "friendly moves away");
  const auto conservative = [](const GuardMotion & motion) {
      return armor_aim::FriendlyCrossesShotConservative(
        576, 612, 0, -600, 1.02, 0, motion);
    };
  const auto balanced = [](const GuardMotion & motion) {
      return armor_aim::FriendlyCrossesShotConservative(
        576, 612, 0, -600, 1.02, 0, motion, true);
    };
  check(!balanced({576, 312, 400, 0, 0, 0, 32, 16, 20}),
    "balanced mode permits a stable friendly moving away from ray");
  check(balanced({376, 312, 400, 0, 0, 0, 32, 16, 20}),
    "balanced mode still vetoes crossing friendly");
  check(balanced({650, 312, 0, 0, 0, 0, 32, 16, 1000}),
    "balanced mode preserves uncertain new friendly veto");
  check(!blocked({576, 312, 400, 0, 0, 0, 32, 16, 20}) &&
    conservative({576, 312, 400, 0, 0, 0, 32, 16, 20}),
    "conservative veto retains current-position risk when friend is predicted to leave");
  check(!conservative({1100, 312, 0, 0, 0, 0, 32, 16, 20}),
    "conservative mode still permits a clearly separated lane");
  check(
    std::abs(armor_aim::ScreenFlightTime(576, 612, 0, -600, 1152, 648) - 1.02) < 1.0e-9,
    "flight horizon reaches screen boundary");
  check(blocked({900, 312, 0, 0, 0, 0, 32, 16, 1000}), "new track fails safe");
  check(
    blocked({std::numeric_limits<double>::quiet_NaN(), 312, 0, 0, 0, 0, 32, 16, 0}),
    "invalid motion fails safe");

  std::mt19937 random(20260921);
  std::uniform_real_distribution<double> x(0, 1152);
  std::uniform_real_distribution<double> y(0, 600);
  std::uniform_real_distribution<double> speed(-800, 800);
  std::uniform_real_distribution<double> acceleration(-200, 200);
  int reference_collisions = 0;
  for (int scene = 0; scene < 1000; ++scene) {
    const GuardMotion motion{
      x(random), y(random), speed(random), 0, acceleration(random), 0, 32, 16, 0};
    check(!blocked(motion) || conservative(motion),
      "conservative guard never relaxes an original veto");
    check(!blocked(motion) || balanced(motion),
      "balanced guard never relaxes an original veto");
    check(!balanced(motion) || conservative(motion),
      "balanced guard is no stricter than conservative for identical motion");
    bool reference = false;
    for (int sample = 0; sample <= 2040; ++sample) {
      const double time = static_cast<double>(sample) * 0.0005;
      const double delta_x =
        motion.x + motion.vx * time + 0.5 * motion.ax * time * time - 576.0;
      const double delta_y = motion.y - (612.0 - 600.0 * time);
      if (std::abs(delta_x) <= 40.0 && std::abs(delta_y) <= 24.0) {
        reference = true;
        break;
      }
    }
    if (reference) {
      ++reference_collisions;
      check(blocked(motion), "dense reference collision cannot slip through samples");
    }
  }
  check(reference_collisions > 0, "randomized oracle exercised collisions");
  std::cout << "Fire guard: 1000 randomized scenes, " << reference_collisions <<
    " reference collisions, " << failures << " failures\n";
  return failures == 0 ? 0 : 1;
}
