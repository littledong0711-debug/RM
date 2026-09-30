#include "armor_aim/armor_detector.hpp"
#include "armor_aim/ballistics.hpp"
#include "armor_aim/game_aim.hpp"
#include "armor_aim/serial_port.hpp"
#include "armor_aim/target_tracker.hpp"

#include <opencv2/imgproc.hpp>

#include <pty.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace
{

bool Check(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

}  // namespace

int main()
{
  bool passed = true;
  armor_aim::ArmorDetector detector;
  // Simulator background is pale yellow; it must not be mistaken for red.
  cv::Mat image(648, 1152, CV_8UC3, cv::Scalar{173, 232, 239});
  cv::circle(image, cv::Point{576, 620}, 16, cv::Scalar{255, 0, 0}, cv::FILLED);
  cv::rectangle(image, cv::Rect{100, 100, 4, 16}, cv::Scalar{0, 0, 255}, cv::FILLED);
  cv::rectangle(image, cv::Rect{130, 100, 4, 16}, cv::Scalar{0, 0, 255}, cv::FILLED);
  const auto launcher = detector.DetectLauncher(image);
  passed &= Check(
    launcher.has_value() && launcher->color == armor_aim::TeamColor::kBlue,
    "launcher color detection");
  passed &= Check(
    launcher.has_value() && cv::norm(launcher->center - cv::Point2f{576.0F, 620.0F}) < 3.0,
    "launcher center detection");
  const auto detections = detector.Detect(image, armor_aim::TeamColor::kRed);
  passed &= Check(detections.size() == 1U, "enemy armor edge pairing");
  cv::Mat wide(648, 1152, CV_8UC3, cv::Scalar{173, 232, 239});
  cv::rectangle(wide, cv::Rect{300, 150, 3, 24}, cv::Scalar{0, 0, 255}, cv::FILLED);
  cv::rectangle(wide, cv::Rect{366, 150, 3, 24}, cv::Scalar{0, 0, 255}, cv::FILLED);
  const auto wide_detections = detector.Detect(wide, armor_aim::TeamColor::kRed);
  passed &= Check(wide_detections.size() == 1U &&
    std::abs(wide_detections.front().center.x - 334.5F) < 1.0F,
    "wide armor pair beyond old 31-pixel closing kernel");
  passed &= Check(!detector.DetectLauncher(wide).has_value(), "background is not launcher");

  armor_aim::TargetTracker tracker;
  const auto first_time = std::chrono::steady_clock::now();
  static_cast<void>(tracker.Update(detections, first_time));
  std::vector<armor_aim::Detection> moved = detections;
  if (!moved.empty()) {
    moved.front().center.y += 5.0F;
    moved.front().box.y += 5.0F;
  }
  const auto tracks = tracker.Update(moved, first_time + std::chrono::milliseconds(20));
  passed &= Check(tracks.size() == 1U, "two-frame track confirmation");
  passed &= Check(!tracks.empty() && std::abs(tracks.front().velocity.y - 250.0F) < 1.0F,
    "velocity initializes without long convergence delay");
  const auto coasting = tracker.Update({}, first_time + std::chrono::milliseconds(40));
  passed &= Check(!coasting.empty() && coasting.front().missed_frames == 1,
    "missing observation retains track but marks it unavailable for firing");
  passed &= Check(!coasting.empty() && coasting.front().consecutive_hits == 0 &&
    coasting.front().hit_streak == 2,
    "miss resets continuity without resetting friendly cumulative observation count");

  armor_aim::TargetTracker confirmation_tracker;
  std::vector<armor_aim::TrackedTarget> confirmation_tracks;
  const armor_aim::Detection stationary{{300.0F, 200.0F},
    {270.0F, 185.0F, 60.0F, 30.0F}, 1.0F};
  for (int i = 0; i <= 4; ++i) {
    confirmation_tracks = confirmation_tracker.Update(
      {stationary}, first_time + std::chrono::milliseconds(i * 10));
  }
  passed &= Check(!confirmation_tracks.empty() &&
    armor_aim::IsShootableTrack(confirmation_tracks.front()) &&
    !armor_aim::IsShootableTrack(confirmation_tracks.front(), 0.15F),
    "baseline stays shootable but confirmation profile rejects five fast frames");
  for (int i = 5; i <= 16; ++i) {
    confirmation_tracks = confirmation_tracker.Update(
      {stationary}, first_time + std::chrono::milliseconds(i * 10));
  }
  passed &= Check(!confirmation_tracks.empty() &&
    armor_aim::IsShootableTrack(confirmation_tracks.front(), 0.15F),
    "confirmation profile accepts continuous observations spanning 150 ms");
  static_cast<void>(confirmation_tracker.Update({}, first_time + std::chrono::milliseconds(170)));
  confirmation_tracks = confirmation_tracker.Update(
    {stationary}, first_time + std::chrono::milliseconds(180));
  passed &= Check(!confirmation_tracks.empty() &&
    confirmation_tracks.front().consecutive_hits == 1 &&
    confirmation_tracks.front().consecutive_seconds == 0.0F &&
    armor_aim::IsShootableTrack(confirmation_tracks.front()) &&
    !armor_aim::IsShootableTrack(confirmation_tracks.front(), 0.15F),
    "reacquired enemy reconfirms while baseline behavior is preserved");
  confirmation_tracks = confirmation_tracker.Update(
    {stationary}, first_time + std::chrono::milliseconds(500));
  passed &= Check(!confirmation_tracks.empty() &&
    confirmation_tracks.front().consecutive_hits == 1 &&
    confirmation_tracks.front().consecutive_seconds == 0.0F,
    "long image gap restarts continuity even without explicit empty frame");

  armor_aim::Ballistics ballistics{600.0F};
  armor_aim::TrackedTarget game_track;
  game_track.hit_streak = 5;
  game_track.position = {1118.0F, 81.0F};
  game_track.velocity = {-58.0F, 58.0F};
  game_track.acceleration = {0.0F, 40.0F};
  const auto horizontal = armor_aim::SolveGameAim(
    ballistics, {576.0F, 611.0F}, game_track, 0.12F, {1152, 648});
  passed &= Check(horizontal && std::abs(horizontal->intercept.y - 81.0F) < 0.001F,
    "recorded vertical jitter must not shift horizontal plate intercept");
  armor_aim::AimSolution short_shot;
  short_shot.flight_time_seconds = 0.45F;
  armor_aim::AimSolution long_shot;
  long_shot.flight_time_seconds = 0.9F;
  game_track.acceleration.x = 100.0F;
  passed &= Check(
    armor_aim::ShotRisk(short_shot, game_track, 0.12F, 600.0F, false) <
    armor_aim::ShotRisk(long_shot, game_track, 0.12F, 600.0F, true),
    "a much shorter accelerating-target shot overrides lock hysteresis");
  game_track.acceleration = {};
  for (const float sign : {-1.0F, 1.0F}) {
    auto fast_track = game_track;
    fast_track.position = {576.0F, 200.0F};
    fast_track.velocity = {sign * 300.0F, 0.0F};
    const auto normal_lead = armor_aim::SolveGameAim(
      ballistics, {576.0F, 611.0F}, fast_track, 0.12F, {1152, 648});
    const auto reduced_lead = armor_aim::SolveGameAim(
      ballistics, {576.0F, 611.0F}, fast_track, 0.09F, {1152, 648});
    passed &= Check(normal_lead && reduced_lead &&
      sign * (normal_lead->intercept.x - reduced_lead->intercept.x) > 0.0F &&
      sign * (reduced_lead->intercept.x - fast_track.position.x) > 0.0F,
      "reduced latency reduces lead in either direction without removing flight prediction");
  }
  game_track.hit_streak = 4;
  passed &= Check(
    !armor_aim::SolveGameAim(
      ballistics, {576.0F, 611.0F}, game_track, 0.12F, {1152, 648}),
    "new edge track must stabilize before it becomes shootable");
  game_track.hit_streak = 5;
  game_track.missed_frames = 1;
  passed &= Check(
    !armor_aim::SolveGameAim(
      ballistics, {576.0F, 611.0F}, game_track, 0.12F, {1152, 648}),
    "temporarily lost or destroyed plate remains trackable but is not shootable");
  game_track.missed_frames = 0;
  for (const float direction : {-1.0F, 1.0F}) {
    game_track.position = {direction < 0 ? 155.0F : 997.0F, 168.0F};
    game_track.velocity = {direction * 264.0F, 0.0F};
    game_track.acceleration = {};
    passed &= Check(!armor_aim::SolveGameAim(
        ballistics, {576.0F, 611.0F}, game_track, 0.12F, {1152, 648}),
      "escaping plate with off-screen intercept must not be selected");
    game_track.velocity.x *= -1.0F;
    passed &= Check(armor_aim::SolveGameAim(
        ballistics, {576.0F, 611.0F}, game_track, 0.12F, {1152, 648}).has_value(),
      "inward-moving edge plate remains selectable");
  }
  const auto solution = ballistics.Solve(
    cv::Point2f{0.0F, 0.0F}, cv::Point2f{600.0F, 0.0F}, cv::Point2f{0.0F, 0.0F});
  passed &= Check(solution.has_value(), "ballistic solution exists");
  if (solution.has_value()) {
    passed &= Check(std::abs(solution->flight_time_seconds - 1.0F) < 0.001F, "flight time");
    passed &= Check(std::abs(solution->angle_degrees) < 0.001F, "aim angle");
  }
  const auto upward_solution = ballistics.Solve(
    cv::Point2f{0.0F, 600.0F}, cv::Point2f{0.0F, 0.0F}, cv::Point2f{0.0F, 0.0F});
  passed &= Check(
    upward_solution.has_value() && std::abs(upward_solution->angle_degrees - 90.0F) < 0.001F,
    "image-to-turret Y-axis conversion");
  const auto moving = ballistics.Solve(
    {0.0F, 0.0F}, {600.0F, 0.0F}, {300.0F, 0.0F}, {}, 0.1F);
  passed &= Check(moving.has_value() &&
    std::abs(moving->flight_time_seconds - 2.1F) < 0.001F &&
    std::abs(moving->intercept.x - 1260.0F) < 0.01F,
    "moving intercept includes latency and distance at 600 pixels per second");
  passed &= Check(!ballistics.Solve({0.0F, 0.0F}, {100.0F, 0.0F},
    {700.0F, 0.0F}).has_value(), "unreachable receding target rejected");
  const auto accelerating = ballistics.Solve(
    {0.0F, 0.0F}, {500.0F, 0.0F}, {}, {200.0F, 0.0F});
  passed &= Check(accelerating.has_value() &&
    std::abs(accelerating->flight_time_seconds - 1.0F) < 0.001F,
    "accelerating target intercept chooses earliest root");
  armor_aim::TargetTracker accelerating_tracker;
  std::vector<armor_aim::TrackedTarget> acceleration_tracks;
  for (int i = 0; i <= 60; ++i) {
    const float t = static_cast<float>(i) / 60.0F;
    const float x = 100.0F + 150.0F * t + 40.0F * t * t;
    acceleration_tracks = accelerating_tracker.Update(
      {armor_aim::Detection{{x, 200.0F}, {x - 30.0F, 185.0F, 60.0F, 30.0F}, 1.0F}},
      first_time + std::chrono::microseconds(i * 1000000 / 60));
  }
  const float regularized_future = acceleration_tracks.empty() ? 0.0F :
    acceleration_tracks.front().position.x + 0.8F * acceleration_tracks.front().velocity.x +
    0.5F * 0.8F * 0.8F * acceleration_tracks.front().acceleration.x;
  passed &= Check(acceleration_tracks.size() == 1U &&
    std::abs(acceleration_tracks.front().acceleration.x - 80.0F) < 10.0F &&
    std::abs(acceleration_tracks.front().velocity.x - 230.0F) < 5.0F &&
    std::abs(regularized_future - 499.6F) < 8.0F,
    "regularized trajectory fit preserves useful acceleration and future position");

  int master_descriptor = -1;
  int slave_descriptor = -1;
  std::array<char, 128> slave_name{};
  passed &= Check(
    openpty(&master_descriptor, &slave_descriptor, slave_name.data(), nullptr, nullptr) == 0,
    "pseudo terminal creation");
  if (master_descriptor >= 0 && slave_descriptor >= 0) {
    close(slave_descriptor);
    armor_aim::SerialPort serial;
    passed &= Check(serial.Open(slave_name.data(), 115200), "serial open");
    passed &= Check(serial.SendAim(90.0F), "aim packet write");
    std::array<unsigned char, 5> packet{};
    const ssize_t aim_bytes = read(master_descriptor, packet.data(), packet.size());
    float decoded_angle = 0.0F;
    std::memcpy(&decoded_angle, packet.data() + 1, sizeof(decoded_angle));
    passed &= Check(aim_bytes == 5 && packet[0] == 0x01U, "aim packet framing");
    passed &= Check(std::abs(decoded_angle - 90.0F) < 0.001F, "aim packet float32 payload");
    passed &= Check(serial.SendFire(), "fire packet write");
    unsigned char fire_packet = 0U;
    const ssize_t fire_bytes = read(master_descriptor, &fire_packet, 1U);
    passed &= Check(fire_bytes == 1 && fire_packet == 0x02U, "fire packet framing");
    close(master_descriptor);
  }

  if (passed) {
    std::cout << "All armor_aim self-tests passed.\n";
    return 0;
  }
  return 1;
}
