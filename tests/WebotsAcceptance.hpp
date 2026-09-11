#pragma once

// Read-only evidence subscriber, compiled only into rm_auto_aim_acceptance.
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "ArmorDetector.hpp"
#include "ArmorTracker.hpp"
#include "Aimer.hpp"
#include "WebotsRefereeTypes.hpp"
#include "xrobot_constexpr.hpp"

namespace WebotsAcceptance
{
constexpr auto layout = ProjectConstexpr::MainFrameLayout;
using Sync = CameraFrameSync<layout>;
using Detection = DetectedFrame<layout>;
using Tracking = TrackedFrame<layout>;
static_assert(std::is_same_v<AimerRefereeSummary,
                             WebotsRefereeTypes::RobotGameRefereeSummary>);

class Observer
{
 public:
  explicit Observer(const std::filesystem::path& directory) : root_(directory)
  {
    std::filesystem::create_directories(root_);
    frames_.open(root_ / "frames.tsv");
    detections_.open(root_ / "detections.tsv");
    tracking_.open(root_ / "tracking.tsv");
    commands_.open(root_ / "commands.tsv");
    firing_.open(root_ / "firing.tsv");
    referee_.open(root_ / "referee.tsv");
    if (!frames_ || !detections_ || !tracking_ || !commands_ || !firing_ || !referee_)
    {
      throw std::runtime_error("cannot create Webots acceptance evidence files");
    }
    frames_ << "stage\tsequence\timage_ts\timu_ts\timage_id\tvalid\n";
    detections_ << "sequence\tnumber\tcolor\tconfidence\tpnp_valid\treprojection_px\ttx\tty\ttz\tquad_valid";
    for (unsigned i = 0; i < 4; ++i) detections_ << "\tp" << i << "x\tp" << i << "y";
    detections_ << '\n';
    tracking_ << "sequence\ttimestamp\ttracking\tnumber\tx\ty\tz\tv_yaw\tfinite\n";
    commands_ << "timestamp\troll\tyaw\troll_vel\tyaw_vel\troll_acc\tyaw_acc\tfinite\n";
    firing_ << "timestamp\tfire\n";
    referee_ << "timestamp\tsize\trobot_id\theat_limit\tcooling\n";
    detections_ << std::setprecision(10);
    tracking_ << std::setprecision(10);
    commands_ << std::setprecision(10);
    if (const char* video = std::getenv("XR_WEBOTS_ACCEPTANCE_VIDEO");
        video != nullptr && video[0] != '\0')
    {
      video_path_ = video;
    }
    if (const char* max_frames = std::getenv("XR_WEBOTS_ACCEPTANCE_VIDEO_MAX_FRAMES");
        max_frames != nullptr && max_frames[0] != '\0')
    {
      video_max_frames_ = static_cast<uint64_t>(std::strtoull(max_frames, nullptr, 10));
    }
    register_thread_.Create(this, RegisterThread, "webots_acceptance", 2048,
                            LibXR::Thread::Priority::LOW);
  }

 private:
  template <typename Packet>
  void Frame(const char* stage, const Packet* packet)
  {
    if (packet == nullptr || !packet->Valid())
    {
      frames_ << stage << "\t0\t0\t0\t0\t0\n";
      frames_.flush();
      return;
    }
    const auto* image = packet->image.Get();
    const auto& geometry = image->geometry;
    const bool valid = geometry.width == layout.width && geometry.height == layout.height &&
                       geometry.step == layout.step && geometry.decimation_x == 1 &&
                       geometry.decimation_y == 1 && geometry.roi_offset_x_native == 0 &&
                       geometry.roi_offset_y_native == 0;
    frames_ << stage << '\t' << packet->sequence << '\t'
            << static_cast<uint64_t>(image->timestamp_us) << '\t'
            << static_cast<uint64_t>(packet->imu.timestamp_us) << '\t'
            << reinterpret_cast<std::uintptr_t>(image) << '\t' << valid << '\n';
    frames_.flush();
  }

  void DetectionFrame(const Detection* packet)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Frame("detector", packet);
    if (packet == nullptr || !packet->Valid()) return;
    const auto* image = packet->image.Get();
    const cv::Mat raw(layout.height, layout.width, CV_8UC3,
                      const_cast<uint8_t*>(image->data.data()), layout.step);
    if (!saved_blank_)
    {
      if (!cv::imwrite((root_ / "first-camera.png").string(), raw))
        throw std::runtime_error("cannot save first Webots camera frame");
      saved_blank_ = true;
    }
    const bool save_target = !packet->detections.empty() && !saved_target_;
    const bool record_video = !video_path_.empty() &&
                              (video_max_frames_ == 0U || video_frames_ < video_max_frames_);
    cv::Mat overlay;
    if (save_target || record_video) overlay = raw.clone();
    for (const auto& result : packet->detections)
    {
      const auto& translation = result.pose.translation;
      std::vector<cv::Point2f> corners(result.points.begin(), result.points.end());
      bool quad_valid = cv::isContourConvex(corners) && cv::contourArea(corners, true) > 16.0;
      for (const auto& point : result.points)
      {
        quad_valid = quad_valid && std::isfinite(point.x) && std::isfinite(point.y) &&
                     point.x >= 0 && point.x < layout.width && point.y >= 0 && point.y < layout.height;
      }
      detections_ << packet->sequence << '\t' << static_cast<unsigned>(result.number)
                  << '\t' << static_cast<unsigned>(result.color) << '\t' << result.confidence
                  << '\t' << result.pnp_valid << '\t' << result.pnp_reprojection_error_px
                  << '\t' << translation[0] << '\t' << translation[1] << '\t' << translation[2]
                  << '\t' << quad_valid;
      for (const auto& point : result.points) detections_ << '\t' << point.x << '\t' << point.y;
      detections_ << '\n';
      if (save_target || record_video)
      {
        for (std::size_t i = 0; i < result.points.size(); ++i)
        {
          const auto& point = result.points[i];
          cv::line(overlay, point, result.points[(i + 1) % 4], cv::Scalar(0, 255, 0), 2,
                   cv::LINE_AA);
          cv::circle(overlay, point, 4, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
          cv::putText(overlay, std::to_string(i), point + cv::Point2f(5, -5),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1,
                      cv::LINE_AA);
        }
        std::ostringstream label;
        label << "N" << static_cast<unsigned>(result.number)
              << " C" << static_cast<unsigned>(result.color)
              << " conf=" << std::fixed << std::setprecision(2) << result.confidence;
        if (result.pnp_valid)
        {
          label << " z=" << std::setprecision(2) << result.pose.translation[2]
                << "m err=" << std::setprecision(2) << result.pnp_reprojection_error_px
                << "px";
        }
        const auto anchor = result.points[0] + cv::Point2f(0.0F, -12.0F);
        cv::putText(overlay, label.str(), anchor, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
      }
    }
    detections_.flush();
    if (save_target)
    {
      if (!cv::imwrite((root_ / "target-camera.png").string(), raw) ||
          !cv::imwrite((root_ / "target-corners.png").string(), overlay))
        throw std::runtime_error("cannot save Webots target evidence");
      saved_target_ = true;
    }
    if (record_video)
    {
      DrawRuntimeOverlay(overlay, packet->sequence, packet->detections.size());
      WriteVideoFrame(overlay);
    }
  }

  void TrackingFrame(const Tracking* packet)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Frame("tracker", packet);
    if (packet == nullptr || !packet->Valid()) return;
    const auto& target = packet->target;
    const bool finite = target.position.allFinite() && target.velocity.allFinite() &&
                        std::isfinite(target.yaw) && std::isfinite(target.v_yaw);
    tracking_ << packet->sequence << '\t' << target.image_timestamp_us << '\t'
              << target.tracking << '\t' << static_cast<unsigned>(target.id) << '\t'
              << target.position[0] << '\t' << target.position[1] << '\t' << target.position[2]
              << '\t' << target.v_yaw << '\t' << finite << '\n';
    tracking_.flush();
    latest_tracking_ = target;
    have_tracking_ = true;
  }

  static void RegisterThread(Observer *self) { self->Register(); }

  static void Subscribe(LibXR::Topic topic, LibXR::Topic::Callback callback)
  {
    topic.RegisterCallback(callback);
  }

  void Register()
  {
    LibXR::Topic::Domain detector_domain("armor_detector"), tracker_domain("tracker"), host("host");
    LibXR::Topic synced(LibXR::Topic::WaitTopic("camera_image_synced"));
    Subscribe(synced, LibXR::Topic::Callback::Create(
        [](bool, Observer* self, Sync::SyncedFrameTopicPayload frame)
        { std::lock_guard<std::mutex> lock(self->mutex_); self->Frame("sync", frame); }, this));
    LibXR::Topic detector(LibXR::Topic::WaitTopic("armors_frame", UINT32_MAX, &detector_domain));
    Subscribe(detector, LibXR::Topic::Callback::Create(
        [](bool, Observer* self, const Detection* frame) { self->DetectionFrame(frame); }, this));
    LibXR::Topic tracker(LibXR::Topic::WaitTopic("target_frame", UINT32_MAX, &tracker_domain));
    Subscribe(tracker, LibXR::Topic::Callback::Create(
        [](bool, Observer* self, const Tracking* frame) { self->TrackingFrame(frame); }, this));
    LibXR::Topic command(LibXR::Topic::WaitTopic("target_euler", UINT32_MAX, &host));
    Subscribe(command, LibXR::Topic::Callback::Create(
        [](bool, Observer* self, LibXR::MicrosecondTimestamp timestamp, const AimerHostGimbalTarget& value)
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          const bool finite = std::isfinite(value.rol) && std::isfinite(value.yaw) &&
                              std::isfinite(value.rol_dot) && std::isfinite(value.yaw_dot) &&
                              std::isfinite(value.rol_ddot) && std::isfinite(value.yaw_ddot);
          self->commands_ << static_cast<uint64_t>(timestamp) << '\t' << value.rol << '\t'
                          << value.yaw << '\t' << value.rol_dot << '\t' << value.yaw_dot << '\t'
                          << value.rol_ddot << '\t' << value.yaw_ddot << '\t' << finite << '\n';
          self->commands_.flush();
          self->latest_command_ = value;
          self->have_command_ = true;
        }, this));
    LibXR::Topic fire(LibXR::Topic::WaitTopic("fire_notify", UINT32_MAX, &host));
    Subscribe(fire, LibXR::Topic::Callback::Create(
        [](bool, Observer* self, LibXR::MicrosecondTimestamp timestamp, const AimerHostFireNotify& value)
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          self->firing_ << static_cast<uint64_t>(timestamp) << '\t' << value.isfire << '\n';
          self->firing_.flush();
          self->latest_fire_ = value.isfire;
          self->have_fire_ = true;
        }, this));
    LibXR::Topic referee(LibXR::Topic::WaitTopic("robot_game_ref", UINT32_MAX, &host));
    Subscribe(referee, LibXR::Topic::Callback::Create(
        [](bool, Observer* self, LibXR::MicrosecondTimestamp timestamp, const AimerRefereeSummary& value)
        {
          std::lock_guard<std::mutex> lock(self->mutex_);
          self->referee_ << static_cast<uint64_t>(timestamp) << '\t' << sizeof(value) << '\t'
                         << static_cast<unsigned>(value.robot_status.robot_id) << '\t'
                         << value.robot_status.shooter_heat_limit << '\t'
                         << value.robot_status.shooter_cooling_value << '\n';
          self->referee_.flush();
        }, this));
  }

  void DrawRuntimeOverlay(cv::Mat& image, uint64_t sequence, std::size_t detections) const
  {
    cv::rectangle(image, cv::Rect(0, 0, image.cols, 82), cv::Scalar(0, 0, 0), -1);
    std::ostringstream line1;
    line1 << "Webots AutoAim  seq=" << sequence << "  detections=" << detections;
    cv::putText(image, line1.str(), cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX, 0.58,
                cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    std::ostringstream line2;
    line2 << std::fixed << std::setprecision(2);
    if (have_tracking_)
    {
      line2 << "Tracker: " << (latest_tracking_.tracking ? "TRACK" : "LOST")
            << " id=" << static_cast<unsigned>(latest_tracking_.id)
            << " xyz=(" << latest_tracking_.position[0] << ',' << latest_tracking_.position[1]
            << ',' << latest_tracking_.position[2] << ") vyaw=" << latest_tracking_.v_yaw;
    }
    else
    {
      line2 << "Tracker: waiting";
    }
    cv::putText(image, line2.str(), cv::Point(12, 48), cv::FONT_HERSHEY_SIMPLEX, 0.48,
                cv::Scalar(0, 220, 255), 1, cv::LINE_AA);

    std::ostringstream line3;
    line3 << std::fixed << std::setprecision(3);
    if (have_command_)
      line3 << "Aimer: roll=" << latest_command_.rol << " yaw=" << latest_command_.yaw;
    else
      line3 << "Aimer: waiting";
    line3 << "  fire=" << (have_fire_ ? (latest_fire_ ? "ON" : "OFF") : "-");
    cv::putText(image, line3.str(), cv::Point(12, 72), cv::FONT_HERSHEY_SIMPLEX, 0.48,
                latest_fire_ ? cv::Scalar(0, 0, 255) : cv::Scalar(160, 255, 160), 1,
                cv::LINE_AA);
  }

  void WriteVideoFrame(const cv::Mat& image)
  {
    if (!video_writer_.isOpened())
    {
      std::filesystem::create_directories(std::filesystem::path(video_path_).parent_path());
      constexpr double output_fps = 50.0;
      if (!video_writer_.open(video_path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                              output_fps, image.size(), true))
        throw std::runtime_error("cannot open Webots acceptance video writer");
    }
    video_writer_.write(image);
    ++video_frames_;
    if (video_max_frames_ != 0U && video_frames_ >= video_max_frames_)
    {
      video_writer_.release();
      std::ofstream(root_ / "video-complete.txt")
          << "frames=" << video_frames_ << "\npath=" << video_path_ << "\n";
      video_path_.clear();
    }
  }

  LibXR::Thread register_thread_{};
  std::filesystem::path root_;
  std::mutex mutex_;
  std::ofstream frames_, detections_, tracking_, commands_, firing_, referee_;
  bool saved_blank_{false}, saved_target_{false};
  bool have_tracking_{false}, have_command_{false}, have_fire_{false};
  ArmorTrackerTarget latest_tracking_{};
  AimerHostGimbalTarget latest_command_{};
  bool latest_fire_{false};
  std::string video_path_{};
  uint64_t video_frames_{0U};
  uint64_t video_max_frames_{0U};
  cv::VideoWriter video_writer_{};
};

inline void Install()
{
  const char* directory = std::getenv("XR_WEBOTS_ACCEPTANCE_DIR");
  if (directory == nullptr || directory[0] == '\0')
    throw std::invalid_argument("rm_auto_aim_acceptance requires XR_WEBOTS_ACCEPTANCE_DIR");
  static Observer observer(directory);
  (void)observer;
}
}  // namespace WebotsAcceptance
