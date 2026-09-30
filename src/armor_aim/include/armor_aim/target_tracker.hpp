#ifndef ARMOR_AIM__TARGET_TRACKER_HPP_
#define ARMOR_AIM__TARGET_TRACKER_HPP_

#include "armor_aim/types.hpp"

#include <opencv2/video/tracking.hpp>

#include <chrono>
#include <deque>
#include <vector>

namespace armor_aim
{

class TargetTracker
{
public:
  struct Parameters
  {
    float maximum_match_distance{100.0F};
    int maximum_missed_frames{8};
    int minimum_hit_streak{2};
  };

  TargetTracker();
  explicit TargetTracker(Parameters parameters);
  void Reset();
  std::vector<TrackedTarget> Update(
    const std::vector<Detection> & detections,
    std::chrono::steady_clock::time_point timestamp);

private:
  struct Track
  {
    struct Observation {
      std::chrono::steady_clock::time_point time;
      cv::Point2f position;
    };
    std::deque<Observation> history;
    cv::Point2f fitted_position{};
    cv::Point2f fitted_velocity{};
    cv::Point2f fitted_acceleration{};
    bool fitted{false};
    int id{-1};
    cv::KalmanFilter filter{4, 2, 0, CV_32F};
    cv::Rect2f box{};
    float confidence{0.0F};
    int hit_streak{0};
    int missed_frames{0};
    int consecutive_hits{0};
    std::chrono::steady_clock::time_point consecutive_start{};
    float consecutive_seconds{0.0F};
  };

  Track MakeTrack(const Detection & detection) const;
  static void ConfigureTransition(cv::KalmanFilter & filter, float delta_seconds);
  static TrackedTarget ToPublicTrack(const Track & track);
  static void FitMotion(Track & track);

  Parameters parameters_;
  std::vector<Track> tracks_;
  std::chrono::steady_clock::time_point previous_timestamp_{};
  int next_id_{1};
};

}  // namespace armor_aim

#endif  // ARMOR_AIM__TARGET_TRACKER_HPP_
