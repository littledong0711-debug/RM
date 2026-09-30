#include "armor_aim/target_tracker.hpp"

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace armor_aim
{

TargetTracker::TargetTracker()
: TargetTracker(Parameters{}) {}

TargetTracker::TargetTracker(Parameters parameters)
: parameters_(std::move(parameters)) {}

void TargetTracker::Reset()
{
  tracks_.clear();
  previous_timestamp_ = {};
  next_id_ = 1;
}

void TargetTracker::ConfigureTransition(cv::KalmanFilter & filter, const float delta_seconds)
{
  // 状态[x,y,vx,vy]。Kalman平滑并预测位置，历史拟合有效时替代输出运动量。
  filter.transitionMatrix = (cv::Mat_<float>(4, 4) <<
    1.0F, 0.0F, delta_seconds, 0.0F,
    0.0F, 1.0F, 0.0F, delta_seconds,
    0.0F, 0.0F, 1.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F);
}

TargetTracker::Track TargetTracker::MakeTrack(const Detection & detection) const
{
  Track track;
  track.filter.measurementMatrix = cv::Mat::zeros(2, 4, CV_32F);
  track.filter.measurementMatrix.at<float>(0, 0) = 1.0F;
  track.filter.measurementMatrix.at<float>(1, 1) = 1.0F;
  cv::setIdentity(track.filter.processNoiseCov, cv::Scalar::all(3.0));
  track.filter.processNoiseCov.at<float>(2, 2) = 60.0F;
  track.filter.processNoiseCov.at<float>(3, 3) = 60.0F;
  cv::setIdentity(track.filter.measurementNoiseCov, cv::Scalar::all(2.0));
  cv::setIdentity(track.filter.errorCovPost, cv::Scalar::all(30.0));
  track.filter.statePost = (cv::Mat_<float>(4, 1) <<
    detection.center.x, detection.center.y, 0.0F, 0.0F);
  track.box = detection.box;
  track.confidence = detection.confidence;
  track.hit_streak = 1;
  return track;
}

TrackedTarget TargetTracker::ToPublicTrack(const Track & track)
{
  auto limit_vector = [](cv::Point2f value, const float maximum_norm) {
      const float norm = std::sqrt(value.dot(value));
      if (norm > maximum_norm) {
        value *= maximum_norm / norm;
      }
      return value;
    };
  return TrackedTarget{
    track.id,
    track.fitted && track.missed_frames == 0 ? track.fitted_position :
      cv::Point2f{track.filter.statePost.at<float>(0), track.filter.statePost.at<float>(1)},
    track.fitted && track.missed_frames == 0 ? track.fitted_velocity :
    limit_vector(
      cv::Point2f{track.filter.statePost.at<float>(2), track.filter.statePost.at<float>(3)}, 1000.0F),
    track.fitted && track.missed_frames == 0 ? track.fitted_acceleration : cv::Point2f{},
    track.box, track.confidence, track.hit_streak, track.missed_frames,
    track.consecutive_hits, track.consecutive_seconds};
}

void TargetTracker::FitMotion(Track & track)
{
  // 以最新观测为t=0，拟合p(t)=p0+v*t+0.5*a*t²。
  // 样本不足或残差过大则回退匀速；输出加速度0不一定表示真实匀速。
  track.fitted = false;
  const auto & samples = track.history;
  if (samples.size() < 10 ||
    std::chrono::duration<double>(samples.back().time - samples.front().time).count() < 0.6) {
    return;
  }
  const int count = static_cast<int>(samples.size());
  cv::Mat design(count, 3, CV_64F), observations(count, 2, CV_64F), coefficients;
  for (int i = 0; i < count; ++i) {
    const auto & sample = samples[static_cast<std::size_t>(i)];
    const double t = std::chrono::duration<double>(sample.time - samples.back().time).count();
    design.at<double>(i, 0) = 1.0;
    design.at<double>(i, 1) = t;
    design.at<double>(i, 2) = 0.5 * t * t;
    observations.at<double>(i, 0) = sample.position.x;
    observations.at<double>(i, 1) = sample.position.y;
  }
  // 只正则加速度系数，抑制像素噪声引起的极端外推；0.01并非已证明最优。
  cv::Mat normal = design.t() * design;
  normal.at<double>(2, 2) += 0.01;
  if (!cv::solve(normal, design.t() * observations, coefficients, cv::DECOMP_SVD)) {return;}
  const double rms = cv::norm(design * coefficients - observations) / std::sqrt(count);
  auto row = [&coefficients](int i) {
      return cv::Point2f{static_cast<float>(coefficients.at<double>(i, 0)),
        static_cast<float>(coefficients.at<double>(i, 1))};
    };
  const auto velocity = row(1);
  const auto acceleration = row(2);
  // Reject identity swaps and abrupt maneuvers instead of extrapolating a bad fit.
  if (!std::isfinite(rms) || rms > 6.0 || cv::norm(acceleration) > 200.0 ||
    cv::norm(velocity) > 1000.0) {return;}
  track.fitted = true;
  track.fitted_position = row(0);
  track.fitted_velocity = velocity;
  track.fitted_acceleration = acceleration;
}

std::vector<TrackedTarget> TargetTracker::Update(
  const std::vector<Detection> & detections,
  const std::chrono::steady_clock::time_point timestamp)
{
  float delta_seconds = 1.0F / 60.0F;
  if (previous_timestamp_.time_since_epoch().count() != 0) {
    delta_seconds = std::chrono::duration<float>(timestamp - previous_timestamp_).count();
    delta_seconds = std::clamp(delta_seconds, 1.0F / 240.0F, 0.2F);
  }
  previous_timestamp_ = timestamp;

  std::vector<cv::Point2f> predictions;
  predictions.reserve(tracks_.size());
  for (auto & track : tracks_) {
    ConfigureTransition(track.filter, delta_seconds);
    const cv::Mat prediction = track.filter.predict();
    predictions.emplace_back(prediction.at<float>(0), prediction.at<float>(1));
  }

  struct Match {std::size_t track; std::size_t detection; float distance;};
  std::vector<Match> candidates;
  for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
    for (std::size_t detection_index = 0; detection_index < detections.size(); ++detection_index) {
      const float distance = static_cast<float>(
        cv::norm(predictions[track_index] - detections[detection_index].center));
      if (distance <= parameters_.maximum_match_distance) {
        candidates.push_back(Match{track_index, detection_index, distance});
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const Match & left, const Match & right) {
    return left.distance < right.distance;
  });

  std::set<std::size_t> used_tracks;
  // 贪心最近邻实现一对一配对；目标交叉时仍可能出现ID交换。
  std::set<std::size_t> used_detections;
  for (const Match & match : candidates) {
    if (used_tracks.count(match.track) != 0U || used_detections.count(match.detection) != 0U) {
      continue;
    }
    Track & track = tracks_[match.track];
    const Detection & detection = detections[match.detection];
    // Separate continuity from cumulative hits: friendly protection still uses
    // the original counters and retains tracks across missed observations.
    if (track.consecutive_hits == 0 || track.history.empty() ||
      std::chrono::duration<double>(timestamp - track.history.back().time).count() > 0.2)
    {
      track.consecutive_start = timestamp;
      track.consecutive_hits = 0;
    }
    ++track.consecutive_hits;
    track.consecutive_seconds = std::chrono::duration<float>(
      timestamp - track.consecutive_start).count();
    const cv::Mat measurement = (cv::Mat_<float>(2, 1) << detection.center.x, detection.center.y);
    track.filter.correct(measurement);
    if (track.hit_streak == 1 && track.missed_frames == 0) {
      // Bootstrap velocity from the first two observations; zero velocity with
      // tiny covariance otherwise takes many frames to converge.
      const cv::Point2f previous{
        track.box.x + track.box.width * 0.5F, track.box.y + track.box.height * 0.5F};
      const cv::Point2f velocity = (detection.center - previous) / delta_seconds;
      track.filter.statePost.at<float>(2) = velocity.x;
      track.filter.statePost.at<float>(3) = velocity.y;
    }
    track.box = detection.box;
    if (!track.history.empty() &&
      std::chrono::duration<double>(timestamp - track.history.back().time).count() > 0.2) {
      track.history.clear();
    }
    track.history.push_back({timestamp, detection.center});
    // 最多保留1.2秒/144次观测：长窗口更平滑，但对突变的反应会滞后。
    while (track.history.size() > 144 || (!track.history.empty() &&
      std::chrono::duration<double>(timestamp - track.history.front().time).count() > 1.2)) {
      track.history.pop_front();
    }
    FitMotion(track);
    track.confidence = detection.confidence;
    ++track.hit_streak;
    track.missed_frames = 0;
    used_tracks.insert(match.track);
    used_detections.insert(match.detection);
  }

  for (std::size_t index = 0; index < tracks_.size(); ++index) {
    if (used_tracks.count(index) == 0U) {
      tracks_[index].filter.statePost = tracks_[index].filter.statePre.clone();
      ++tracks_[index].missed_frames;
      tracks_[index].consecutive_hits = 0;
      tracks_[index].consecutive_seconds = 0.0F;
    }
  }
  for (std::size_t index = 0; index < detections.size(); ++index) {
    if (used_detections.count(index) == 0U) {
      Track track = MakeTrack(detections[index]);
      track.consecutive_hits = 1;
      track.consecutive_start = timestamp;
      track.history.push_back({timestamp, detections[index].center});
      track.id = next_id_++;
      tracks_.push_back(std::move(track));
    }
  }
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(), [this](const Track & track) {
      return track.missed_frames > parameters_.maximum_missed_frames;
    }),
    tracks_.end());

  std::vector<TrackedTarget> result;
  for (const Track & track : tracks_) {
    if (track.hit_streak >= parameters_.minimum_hit_streak &&
      track.missed_frames <= parameters_.maximum_missed_frames)
    {
      result.push_back(ToPublicTrack(track));
    }
  }
  return result;
}

}  // namespace armor_aim
