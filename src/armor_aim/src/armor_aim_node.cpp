#include "armor_aim/armor_detector.hpp"
#include "armor_aim/ballistics.hpp"
#include "armor_aim/fire_guard.hpp"
#include "armor_aim/game_aim.hpp"
#include "armor_aim/serial_port.hpp"
#include "armor_aim/target_tracker.hpp"
#include "armor_aim/types.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace armor_aim
{

class ArmorAimNode final : public rclcpp::Node
{
  // 流程：解码 -> 确认炮台颜色 -> 敌我跟踪 -> 求拦截点 -> 友军检查
  // -> 先转向、下一帧复核后开火。单位：像素、秒、角度。
public:
  ArmorAimNode()
  : Node("armor_aim"),
    detector_(ReadDetectorParameters()),
    tracker_(ReadTrackerParameters()),
    friendly_tracker_(TargetTracker::Parameters{120.0F, 30, 1}),
    ballistics_(static_cast<float>(declare_parameter<double>("projectile_speed", 600.0))),
    configured_color_(ParseColor(declare_parameter<std::string>("own_color", "auto"))),
    serial_device_(declare_parameter<std::string>("serial_port", "/dev/pts/0")),
    serial_baud_(static_cast<int>(declare_parameter<std::int64_t>("serial_baud", 115200))),
    prediction_latency_seconds_(static_cast<float>(
      declare_parameter<double>("prediction_latency_seconds", 0.12))),
    fire_interval_(std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(declare_parameter<double>("fire_interval_seconds", 0.0))))
  {
    enemy_prediction_latency_seconds_ = static_cast<float>(declare_parameter<double>(
      "enemy_prediction_latency_seconds", prediction_latency_seconds_));
    enemy_confirmation_seconds_ = static_cast<float>(declare_parameter<double>(
      "enemy_confirmation_seconds", 0.0));
    friendly_safety_priority_ = declare_parameter<bool>("friendly_safety_priority", false);
    friendly_balanced_ = declare_parameter<bool>("friendly_balanced", false);
    RCLCPP_INFO(get_logger(), "Balanced friendly protection: %s",
      friendly_balanced_ ? "enabled" : "disabled");
    RCLCPP_INFO(get_logger(), "Conservative friendly veto: %s",
      friendly_safety_priority_ ? "enabled" : "disabled");
    if (!std::isfinite(enemy_prediction_latency_seconds_) ||
      enemy_prediction_latency_seconds_ < 0.0F || enemy_prediction_latency_seconds_ > 0.5F ||
      !std::isfinite(enemy_confirmation_seconds_) ||
      enemy_confirmation_seconds_ < 0.0F || enemy_confirmation_seconds_ > 1.0F)
    {
      throw std::invalid_argument("Invalid enemy prediction/confirmation seconds");
    }
    RCLCPP_INFO(get_logger(),
      "Experiment settings: enemy_latency=%.3fs guard_latency=%.3fs confirmation=%.3fs",
      enemy_prediction_latency_seconds_, prediction_latency_seconds_, enemy_confirmation_seconds_);
    projectile_speed_ = static_cast<float>(get_parameter("projectile_speed").as_double());
    const std::string image_topic = declare_parameter<std::string>("image_topic", "/image_raw");
    TryOpenSerial();
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&ArmorAimNode::OnImage, this, std::placeholders::_1));
    reconnect_timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {TryOpenSerial();});
    RCLCPP_INFO(get_logger(), "Listening on %s; serial device: %s", image_topic.c_str(),
      serial_device_.c_str());
  }

private:
  ArmorDetector::Parameters ReadDetectorParameters()
  {
    ArmorDetector::Parameters parameters;
    parameters.color_difference_threshold = static_cast<int>(
      declare_parameter<std::int64_t>("detector.color_threshold", 100));
    parameters.minimum_brightness = static_cast<int>(
      declare_parameter<std::int64_t>("detector.minimum_brightness", 100));
    parameters.minimum_area = declare_parameter<double>("detector.minimum_area", 12.0);
    parameters.maximum_area = declare_parameter<double>("detector.maximum_area", 3500.0);
    parameters.minimum_aspect_ratio = declare_parameter<double>("detector.minimum_aspect_ratio", 0.55);
    parameters.maximum_aspect_ratio = declare_parameter<double>("detector.maximum_aspect_ratio", 8.0);
    return parameters;
  }

  TargetTracker::Parameters ReadTrackerParameters()
  {
    TargetTracker::Parameters parameters;
    parameters.maximum_match_distance = static_cast<float>(
      declare_parameter<double>("tracker.maximum_match_distance", 100.0));
    parameters.maximum_missed_frames = static_cast<int>(
      declare_parameter<std::int64_t>("tracker.maximum_missed_frames", 8));
    parameters.minimum_hit_streak = static_cast<int>(
      declare_parameter<std::int64_t>("tracker.minimum_hit_streak", 5));
    return parameters;
  }

  static TeamColor ParseColor(const std::string & text)
  {
    if (text == "red") {
      return TeamColor::kRed;
    }
    if (text == "blue") {
      return TeamColor::kBlue;
    }
    return TeamColor::kUnknown;
  }

  void TryOpenSerial()
  {
    if (serial_.IsOpen()) {
      return;
    }
    if (serial_.Open(serial_device_, serial_baud_)) {
      RCLCPP_INFO(get_logger(), "Opened serial port %s", serial_device_.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for serial port %s", serial_device_.c_str());
    }
  }

  std::optional<std::pair<TrackedTarget, AimSolution>> SelectTarget(
    const std::vector<TrackedTarget> & tracks, const cv::Point2f & shooter,
    const cv::Size & screen) const
  {
    std::optional<std::pair<TrackedTarget, AimSolution>> selected;
    float best_priority = std::numeric_limits<float>::max();
    for (const TrackedTarget & track : tracks) {
      const auto solution = SolveGameAim(
        ballistics_, shooter, track, enemy_prediction_latency_seconds_, screen,
        enemy_confirmation_seconds_);
      if (!solution.has_value()) {
        continue;
      }
      // Targets move horizontally, so vertical position does not indicate that
      // they are about to escape. Prefer the shortest, least acceleration-
      // sensitive shot and retain the current lock through a small hysteresis.
      const float priority = ShotRisk(
        *solution, track, enemy_prediction_latency_seconds_, projectile_speed_,
        track.id == locked_target_id_);
      if (priority < best_priority) {
        best_priority = priority;
        selected = std::make_pair(track, *solution);
      }
    }
    return selected;
  }

  void OnImage(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    // 只处理最新画面；日志的1秒限频不等于实际开火间隔。
    cv::Mat image;
    try {
      // The supplied simulator labels its buffer as rgb8, but publishes four
      // bytes per pixel (step == width * 4). Handle that malformed RGBA message
      // explicitly; otherwise cv_bridge interprets alpha bytes as RGB pixels and
      // produces vertical color stripes.
      const bool mislabeled_rgba =
        message->encoding == sensor_msgs::image_encodings::RGB8 &&
        message->step == message->width * 4U &&
        message->data.size() >= static_cast<std::size_t>(message->step) * message->height;
      if (mislabeled_rgba) {
        const cv::Mat rgba(
          static_cast<int>(message->height), static_cast<int>(message->width), CV_8UC4,
          const_cast<unsigned char *>(message->data.data()), message->step);
        // Captured raw frames (20260921-111048) establish RGBA: this restores
        // the pale-yellow background and the red launcher seen in the game.
        cv::cvtColor(rgba, image, cv::COLOR_RGBA2BGR);
      } else {
        image = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8)->image;
      }
    } catch (const cv_bridge::Exception & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Image conversion failed: %s", exception.what());
      return;
    }

    const auto launcher = detector_.DetectLauncher(image);
    if (!launcher.has_value()) {
      launcher_color_streak_ = 0;
      fire_on_next_frame_ = false;
      return;
    }
    if (launcher.has_value()) {
      launcher_position_ = launcher->center;
    }
    if (configured_color_ != TeamColor::kUnknown) {
      own_color_ = configured_color_;
    } else if (launcher.has_value()) {
      if (launcher->color == launcher_color_candidate_) {
        launcher_color_streak_ = std::min(launcher_color_streak_ + 1, 8);
      } else {
        launcher_color_candidate_ = launcher->color;
        launcher_color_streak_ = 1;
      }
      const int required_streak = own_color_ == TeamColor::kUnknown ? 3 : 8;
      if (launcher_color_streak_ >= required_streak && own_color_ != launcher_color_candidate_) {
        own_color_ = launcher_color_candidate_;
        tracker_.Reset();
        friendly_tracker_.Reset();
        locked_target_id_ = -1;
        fire_on_next_frame_ = false;
        RCLCPP_INFO(
          get_logger(), "Confirmed launcher color: %s at (%.1f, %.1f)",
          ToString(own_color_).c_str(), launcher->center.x, launcher->center.y);
      }
    }
    if (own_color_ == TeamColor::kUnknown) {
      return;
    }
    if (launcher->color != own_color_) {
      fire_on_next_frame_ = false;
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const std::vector<Detection> detections = detector_.Detect(image, OppositeColor(own_color_));
    const std::vector<Detection> friendly_detections = detector_.Detect(image, own_color_);
    const auto friendly_tracks = friendly_tracker_.Update(friendly_detections, now);
    // 友军漏检后保留轨迹供避让；敌军当前帧漏检则禁止射击。
    const std::vector<TrackedTarget> tracks = tracker_.Update(detections, now);
    const cv::Point2f shooter = launcher_position_.value_or(cv::Point2f{
      static_cast<float>(image.cols) * 0.5F, static_cast<float>(image.rows) - 27.0F});
    const auto selected = SelectTarget(tracks, shooter, image.size());
    if (!selected.has_value()) {
      locked_target_id_ = -1;
      fire_on_next_frame_ = false;
      return;
    }
    const float shot_angle = fire_on_next_frame_ ? pending_angle_ : selected->second.angle_degrees;
    const float radians = shot_angle * static_cast<float>(CV_PI) / 180.0F;
    const cv::Point2f direction{std::cos(radians), -std::sin(radians)};
    // A missed bullet keeps travelling after the selected target, so protect the
    // complete on-screen ray rather than only the intended target's flight time.
    const double bullet_velocity_x = static_cast<double>(direction.x * projectile_speed_);
    const double bullet_velocity_y = static_cast<double>(direction.y * projectile_speed_);
    const double screen_flight_time = ScreenFlightTime(
      shooter.x, shooter.y, bullet_velocity_x, bullet_velocity_y,
      static_cast<double>(image.cols), static_cast<double>(image.rows));
    const auto blocks_shot = [&](const GuardMotion & motion) {
        if (friendly_safety_priority_ || friendly_balanced_) {
          return FriendlyCrossesShotConservative(
            shooter.x, shooter.y, bullet_velocity_x, bullet_velocity_y,
            screen_flight_time, prediction_latency_seconds_, motion, friendly_balanced_);
        }
        return FriendlyCrossesShot(
          shooter.x, shooter.y, bullet_velocity_x, bullet_velocity_y,
          screen_flight_time, prediction_latency_seconds_, motion);
      };

    bool friendly_in_fire_corridor = false;
    for (const auto & track : friendly_tracks) {
      // Plates in the supplied game move horizontally. Ignoring tiny fitted Y
      // jitter avoids inventing vertical motion while preserving X acceleration.
      double uncertainty_speed = track.hit_streak < 2 ? 1000.0 :
        (track.missed_frames > 0 ? 200.0 : (track.hit_streak < 10 ? 100.0 : 20.0));
      // Newly seen, lost, or reacquired friendlies must not become "reliable"
      // solely because their lifetime observation count is large.
      if (friendly_balanced_) {
        // Keep first sightings maximally uncertain; allow faster recovery once
        // several observations are available, without weakening baseline guard.
        if (track.consecutive_hits < 2) {
          uncertainty_speed = 1000.0;
        } else if (track.missed_frames > 0 || track.consecutive_hits < 5 ||
          track.consecutive_seconds < 0.12F)
        {
          uncertainty_speed = std::max(200.0, uncertainty_speed);
        }
      } else if (friendly_safety_priority_ &&
        (track.missed_frames > 0 || track.consecutive_hits < 5 ||
        track.consecutive_seconds < 0.25F))
      {
        uncertainty_speed = 1000.0;
      }
      if (blocks_shot(GuardMotion{
          track.position.x, track.position.y, track.velocity.x, 0.0,
          track.acceleration.x, 0.0,
          0.5 * track.box.width, 0.5 * track.box.height, uncertainty_speed}))
      {
        friendly_in_fire_corridor = true;
        break;
      }
    }
    // Fail safe if a current detection is not represented by a tracker output.
    if (!friendly_in_fire_corridor) {
      for (const auto & detection : friendly_detections) {
        const bool represented = std::any_of(
          friendly_tracks.begin(), friendly_tracks.end(), [&](const TrackedTarget & track) {
            return track.missed_frames == 0 && cv::norm(track.position - detection.center) < 20.0;
          });
        if (!represented && blocks_shot(GuardMotion{
            detection.center.x, detection.center.y, 0.0, 0.0, 0.0, 0.0,
            0.5 * detection.box.width, 0.5 * detection.box.height, 1000.0}))
        {
          friendly_in_fire_corridor = true;
          break;
        }
      }
    }
    if (friendly_in_fire_corridor) {
      fire_on_next_frame_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Fire inhibited: friendly armor near aim point");
      return;
    }
    if (fire_on_next_frame_) {
      // 使用上一帧真正发出的pending_angle_复核，不能假设炮口已转到新角度。
      fire_on_next_frame_ = false;
      const cv::Point2f offset = selected->second.intercept - shooter;
      const float cross_error = std::abs(offset.x * direction.y - offset.y * direction.x);
      const float tolerance = std::max(2.0F,
        0.35F * std::min(selected->first.box.width, selected->first.box.height));
      if (selected->first.id != pending_target_id_ || selected->first.missed_frames != 0 ||
        offset.dot(direction) <= 0.0F || cross_error > tolerance ||
        std::chrono::duration<double>(now - pending_time_).count() > 0.1) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Fire cancelled: pending_id=%d current_id=%d missed=%d cross_error=%.2f "
          "tolerance=%.2f forward=%.1f aim_age=%.3fs",
          pending_target_id_, selected->first.id, selected->first.missed_frames,
          cross_error, tolerance, offset.dot(direction),
          std::chrono::duration<double>(now - pending_time_).count());
        return;
      }
      if (serial_.SendFire()) {
        last_fire_ = now;
        fire_on_next_frame_ = false;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Fire command sent");
      } else {
        serial_.Close();
      }
      return;
    }
    locked_target_id_ = selected->first.id;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Lock %d: target=(%.0f, %.0f), speed=(%.0f, %.0f), flight=%.3fs, "
      "intercept=(%.0f, %.0f), ax=%.2f hits=%d missed=%d consecutive=%d span=%.3fs",
      selected->first.id, selected->first.position.x, selected->first.position.y,
      selected->first.velocity.x, selected->first.velocity.y,
      selected->second.flight_time_seconds, selected->second.intercept.x,
      selected->second.intercept.y, selected->first.acceleration.x,
      selected->first.hit_streak, selected->first.missed_frames,
      selected->first.consecutive_hits, selected->first.consecutive_seconds);

    if (!serial_.SendAim(selected->second.angle_degrees)) {
      serial_.Close();
      return;
    }
    if (now - last_fire_ >= fire_interval_) {
      fire_on_next_frame_ = selected->first.missed_frames == 0;
      pending_target_id_ = selected->first.id;
      pending_angle_ = selected->second.angle_degrees;
      pending_time_ = now;
    }
  }

  ArmorDetector detector_;
  TargetTracker tracker_;
  TargetTracker friendly_tracker_;
  Ballistics ballistics_;
  SerialPort serial_;
  TeamColor configured_color_{TeamColor::kUnknown};
  TeamColor own_color_{TeamColor::kUnknown};
  TeamColor launcher_color_candidate_{TeamColor::kUnknown};
  int launcher_color_streak_{0};
  std::optional<cv::Point2f> launcher_position_;
  std::string serial_device_;
  int serial_baud_{115200};
  float prediction_latency_seconds_{0.12F};
  float enemy_prediction_latency_seconds_{0.12F};
  float enemy_confirmation_seconds_{0.0F};
  bool friendly_safety_priority_{false};
  bool friendly_balanced_{false};
  std::chrono::steady_clock::duration fire_interval_{};
  std::chrono::steady_clock::time_point last_fire_{};
  bool fire_on_next_frame_{false};
  int locked_target_id_{-1};
  int pending_target_id_{-1};
  float pending_angle_{0.0F};
  std::chrono::steady_clock::time_point pending_time_{};
  float projectile_speed_{600.0F};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

}  // namespace armor_aim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<armor_aim::ArmorAimNode>());
  rclcpp::shutdown();
  return 0;
}
