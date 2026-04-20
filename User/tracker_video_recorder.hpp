#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "ArmorTracker.hpp"
#include "CameraBase.hpp"
#include "SolveTrajectory.hpp"
#include "armor.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "webots_truth_visible_plane.hpp"
#include "xrobot_constexpr.hpp"

namespace
{
const char* ArmorsTopicName()
{
  const char* env = std::getenv("XR_ARMORS_TOPIC_NAME");
  return (env != nullptr && env[0] != '\0') ? env : "armors_result";
}
}  // namespace

class TrackerVideoRecorder
{
 public:
  TrackerVideoRecorder()
  {
    camera_info_ = std::make_shared<CameraBase::CameraInfo>(ProjectConstexpr::MainCameraInfo);
    const char* video_env = std::getenv("XR_TRACKER_VIDEO_PATH");
    if (video_env != nullptr && video_env[0] != '\0')
    {
      video_path_ = video_env;
    }
    else
    {
      video_path_ = "tracker_overlay_100fps.mp4";
    }

    const char* summary_env = std::getenv("XR_TRACKER_VIDEO_SUMMARY_PATH");
    if (summary_env != nullptr && summary_env[0] != '\0')
    {
      summary_path_ = summary_env;
    }
    else
    {
      summary_path_ = video_path_ + ".summary.txt";
    }

    const char* truth_env = std::getenv("XR_TRACKER_VIDEO_TRUTH_PATH");
    if (truth_env != nullptr && truth_env[0] != '\0')
    {
      truth_path_ = truth_env;
    }
    else
    {
      truth_path_ = video_path_ + ".truth.tsv";
    }
    ekf_truth_path_ = video_path_ + ".ekf_truth.tsv";
    ekf_truth_summary_path_ = ekf_truth_path_ + ".summary.txt";

    const char* corner_audit_env = std::getenv("XR_DETECTOR_CORNER_AUDIT_PATH");
    if (corner_audit_env != nullptr && corner_audit_env[0] != '\0')
    {
      detector_corner_audit_path_ = corner_audit_env;
    }
    else
    {
      detector_corner_audit_path_ = video_path_ + ".detector_corners.tsv";
    }

    const char* corner_audit_summary_env =
        std::getenv("XR_DETECTOR_CORNER_AUDIT_SUMMARY_PATH");
    if (corner_audit_summary_env != nullptr && corner_audit_summary_env[0] != '\0')
    {
      detector_corner_audit_summary_path_ = corner_audit_summary_env;
    }
    else
    {
      detector_corner_audit_summary_path_ = detector_corner_audit_path_ + ".summary.txt";
    }

    const char* max_frames_env = std::getenv("XR_TRACKER_VIDEO_MAX_FRAMES");
    if (max_frames_env != nullptr && max_frames_env[0] != '\0')
    {
      char* end = nullptr;
      const unsigned long parsed = std::strtoul(max_frames_env, &end, 10);
      if (end != max_frames_env && parsed > 0UL)
      {
        max_frames_ = static_cast<uint32_t>(
            std::min<unsigned long>(parsed, std::numeric_limits<uint32_t>::max()));
      }
    }

    const char* fps_env = std::getenv("XR_TRACKER_VIDEO_FPS");
    if (fps_env != nullptr && fps_env[0] != '\0')
    {
      char* end = nullptr;
      const double parsed = std::strtod(fps_env, &end);
      if (end != fps_env && std::isfinite(parsed) && parsed > 0.0)
      {
        output_fps_ = parsed;
      }
    }
  }

  void Init(webots::Supervisor* supervisor) { supervisor_ = supervisor; }

  void InstallBlocking()
  {
    auto image_topic = LibXR::Topic(LibXR::Topic::WaitTopic("image_raw", UINT32_MAX));
    auto image_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* image = reinterpret_cast<cv::Mat*>(data.addr_);
          self->ImageCallback(image);
        },
        this);
    image_topic.RegisterCallback(image_cb);

    LibXR::Topic::Domain armor_domain("armor_detector");
    auto armors_topic = LibXR::Topic(
        LibXR::Topic::WaitTopic(ArmorsTopicName(), UINT32_MAX, &armor_domain));
    auto armors_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* armors = reinterpret_cast<ArmorDetectionsMessage*>(data.addr_);
          self->ArmorsCallback(armors);
        },
        this);
    armors_topic.RegisterCallback(armors_cb);

    LibXR::Topic::Domain tracker_domain("tracker");

    auto tracker_info_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("info", UINT32_MAX, &tracker_domain));
    auto tracker_info_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* info = reinterpret_cast<ArmorTracker::TrackerInfo*>(data.addr_);
          self->TrackerInfoCallback(info);
        },
        this);
    tracker_info_topic.RegisterCallback(tracker_info_cb);

    auto target_eulr_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("target_eulr", UINT32_MAX, &tracker_domain));
    auto target_eulr_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* eulr = reinterpret_cast<LibXR::EulerAngle<float>*>(data.addr_);
          self->TargetEulrCallback(eulr);
        },
        this);
    target_eulr_topic.RegisterCallback(target_eulr_cb);

    auto fire_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("fire_notify", UINT32_MAX, &tracker_domain));
    auto fire_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* fire = reinterpret_cast<uint8_t*>(data.addr_);
          self->FireNotifyCallback(fire);
        },
        this);
    fire_topic.RegisterCallback(fire_cb);

    auto send_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("send", UINT32_MAX, &tracker_domain));
    auto send_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* send = reinterpret_cast<ArmorTracker::Send*>(data.addr_);
          self->SendCallback(send);
        },
        this);
    send_topic.RegisterCallback(send_cb);

    auto ekf_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("ekf_points", UINT32_MAX, &tracker_domain));
    auto ekf_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* msg = reinterpret_cast<ArmorTracker::EkfPointsMsg*>(data.addr_);
          self->EkfPointsCallback(msg);
        },
        this);
    ekf_topic.RegisterCallback(ekf_cb);

    auto candidate_debug_topic = LibXR::Topic(
        LibXR::Topic::WaitTopic("candidate_debug", UINT32_MAX, &tracker_domain));
    auto candidate_debug_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* msg = reinterpret_cast<ArmorTracker::CandidateDebugMsg*>(data.addr_);
          self->CandidateDebugCallback(msg);
        },
        this);
    candidate_debug_topic.RegisterCallback(candidate_debug_cb);

    auto target_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("target", UINT32_MAX, &tracker_domain));
    auto target_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerVideoRecorder* self, LibXR::RawData& data)
        {
          auto* target = reinterpret_cast<SolveTrajectory::Target*>(data.addr_);
          self->TargetCallback(target);
        },
        this);
    target_topic.RegisterCallback(target_cb);

    XR_LOG_PASS(
        "TrackerVideoRecorder subscribed: image_raw + %s + tracker/* -> %s",
        ArmorsTopicName(), video_path_.c_str());
  }

  bool Done() const { return done_.load(std::memory_order_relaxed); }

 private:
  static constexpr std::size_t kMaxIndependentTracks = 16;
  static constexpr uint8_t kInvalidArmorIndex = 255;
  static constexpr std::array<const char*, 4> kTruthLabels = {
      "front", "right", "back", "left"};
  static constexpr std::array<const char*, 4> kTruthNodeLabels = {
      "armor_front", "armor_right", "armor_back", "armor_left"};
  static constexpr const char* kTruthVisibleFaceProtoDef = "XR_VISIBLE_FACE_POSE";
  static constexpr std::array<std::array<double, 3>, 4> kArmorLocalPosSpin = {
      std::array<double, 3>{0.0, 0.205, -0.06},
      std::array<double, 3>{0.205, 0.0, -0.11},
      std::array<double, 3>{0.0, -0.205, -0.06},
      std::array<double, 3>{-0.205, 0.0, -0.11}};
  static constexpr double kIndependentYawRateMax = 12.0;

  struct IndependentArmorTrack
  {
    bool active{false};
    bool confirmed{false};
    uint16_t track_id{0};
    uint16_t stable_track_id{0};
    bool stable_track_id_valid{false};
    ArmorColor color{ArmorColor::UNKNOWN};
    ArmorNumber number{ArmorNumber::INVALID};
    ArmorType type{ArmorType::INVALID};
    float confidence{0.0f};
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double yaw{0.0};
    double yaw_rate{0.0};
    cv::Point2f image_center{};
    cv::Point2f image_velocity{};
    double area{0.0};
    double area_rate{0.0};
    uint64_t last_timestamp_us{0};
    uint32_t age{0};
    uint32_t hit_count{0};
    uint32_t miss_count{0};
    bool matched_this_frame{false};
    uint8_t matched_armor_index{kInvalidArmorIndex};
    float last_match_score{0.0f};
    int truth_gt_index{-1};
    double truth_error_m{0.0};
    uint32_t truth_switch_count{0};
    bool truth_valid_this_frame{false};
    bool truth_switched_this_frame{false};
  };

  struct IndependentTracksSnapshot
  {
    uint64_t image_timestamp_us{0};
    uint8_t detection_count{0};
    uint8_t active_count{0};
    uint8_t matched_count{0};
    uint8_t new_track_count{0};
    uint8_t removed_count{0};
    std::array<IndependentArmorTrack, kMaxIndependentTracks> tracks{};
  };

  struct IndependentMatchCandidate
  {
    std::size_t track_slot{0};
    std::size_t armor_index{0};
    double score{0.0};
    double order_bias{0.0};
    double center_diff{0.0};
    double area_log{0.0};
    double position_diff{0.0};
    double yaw_diff{0.0};
    double resolved_yaw{0.0};
  };

  struct IndependentTrackStats
  {
    uint32_t frames{0};
    uint32_t candidate_total{0};
    uint32_t matched_total{0};
    uint32_t new_track_total{0};
    uint32_t removed_total{0};
    uint32_t suppressed_spawn_total{0};
    uint32_t duplicate_pair_frames{0};
    uint32_t duplicate_pair_total{0};
  };

  struct DetectionTruthAssignment
  {
    bool valid{false};
    int gt_index{-1};
    double error_m{0.0};
    double continuity_error_m{0.0};
    double total_cost{0.0};
  };

  struct DetectorCornerTruthAssignment
  {
    bool valid{false};
    int gt_index{-1};
    double point_mean_px{0.0};
    double point_rms_px{0.0};
    double point_max_px{0.0};
    double center_dx_px{0.0};
    double center_dy_px{0.0};
    double center_err_px{0.0};
    double shape_mean_px{0.0};
    double shape_rms_px{0.0};
  };

  struct TrackTruthPersistent
  {
    int last_gt_index{-1};
    uint32_t switch_count{0};
  };

  struct TruthGtPersistent
  {
    bool valid{false};
    uint64_t last_ts_us{0};
    Eigen::Vector3d last_det_pos{0.0, 0.0, 0.0};
  };

  struct TrackTruthSnapshot
  {
    bool valid{false};
    int gt_index{-1};
    double error_m{0.0};
    uint32_t switch_count{0};
    bool switched_this_frame{false};
  };

  struct IndependentTruthStats
  {
    uint32_t frame_total{0};
    uint32_t detection_assignment_total{0};
    uint32_t track_assignment_total{0};
    uint32_t switch_total{0};
    uint32_t switched_track_total{0};
    uint32_t switch_frame_total{0};
    double track_error_sum{0.0};
    uint32_t track_error_count{0};
  };

  struct EkfTruthAssignment
  {
    bool valid{false};
    int gt_index{-1};
    double error_m{0.0};
  };

  struct EkfTruthStats
  {
    uint32_t frame_total{0};
    uint32_t row_count{0};
    std::vector<double> armor_err_values{};
    std::vector<double> frame_mean_err_values{};
    std::vector<double> frame_max_err_values{};
  };

  struct DetectorCornerAuditStats
  {
    uint32_t frame_total{0};
    uint32_t row_count{0};
    std::vector<double> point_mean_values{};
    std::vector<double> point_rms_values{};
    std::vector<double> point_max_values{};
    std::vector<double> center_err_values{};
    std::vector<double> shape_mean_values{};
    std::vector<double> shape_rms_values{};
  };

  static cv::Scalar ColorToScalar(ArmorColor color)
  {
    switch (color)
    {
      case ArmorColor::BLUE:
        return cv::Scalar(255, 180, 40);
      case ArmorColor::RED:
        return cv::Scalar(60, 90, 255);
      case ArmorColor::EXTINGUISH:
        return cv::Scalar(180, 180, 180);
      case ArmorColor::PURPLE:
        return cv::Scalar(220, 70, 220);
      case ArmorColor::UNKNOWN:
      default:
        return cv::Scalar(150, 220, 150);
    }
  }

  static std::string ArmorNumberString(ArmorNumber number)
  {
    const auto index = static_cast<std::size_t>(number);
    if (index >= ARMOR_NUMBER_NAMES.size())
    {
      return "invalid";
    }
    return std::string(ARMOR_NUMBER_NAMES[index]);
  }

  static std::string ArmorTypeString(ArmorType type)
  {
    const auto index = static_cast<std::size_t>(type);
    if (index >= ARMOR_TYPE_NAMES.size())
    {
      return "invalid";
    }
    return std::string(ARMOR_TYPE_NAMES[index]);
  }

  static const char* TruthLabelString(int gt_index)
  {
    if (gt_index < 0 || gt_index >= static_cast<int>(kTruthLabels.size()))
    {
      return "na";
    }
    return kTruthLabels[static_cast<std::size_t>(gt_index)];
  }

  static cv::Scalar TrackIdColor(uint16_t track_id)
  {
    static const std::array<cv::Scalar, 8> kPalette = {
        cv::Scalar(255, 210, 120), cv::Scalar(120, 220, 255),
        cv::Scalar(120, 255, 160), cv::Scalar(255, 150, 210),
        cv::Scalar(190, 170, 255), cv::Scalar(255, 190, 120),
        cv::Scalar(160, 240, 240), cv::Scalar(240, 240, 140)};
    return kPalette[track_id % kPalette.size()];
  }

  static double ArmorYaw(const ArmorDetectorResult& armor)
  {
    LibXR::EulerAngle<double> eulr =
        LibXR::RotationMatrix<double>(armor.pose.rotation.ToRotationMatrix())
            .ToEulerAngle();
    return eulr.Yaw();
  }

  static double UnwrapAngleNear(double value, double reference)
  {
    constexpr double kTau = 6.28318530717958647692;
    return reference + std::remainder(value - reference, kTau);
  }

  static double ClampAbs(double value, double limit)
  {
    return std::clamp(value, -limit, limit);
  }

  static double ArmorArea(const ArmorDetectorResult& armor)
  {
    return std::abs(cv::contourArea(
        std::vector<cv::Point2f>(armor.points.begin(), armor.points.end())));
  }

  static double TimestampDeltaSeconds(uint64_t newer, uint64_t older)
  {
    if (newer > older)
    {
      return static_cast<double>(newer - older) / 1000000.0;
    }
    return 0.0;
  }

  static bool CompatibleTrackLabel(const IndependentArmorTrack& track,
                                   const ArmorDetectorResult& armor)
  {
    if (track.type != ArmorType::INVALID && armor.type != ArmorType::INVALID &&
        track.type != armor.type)
    {
      return false;
    }
    if (track.color != ArmorColor::UNKNOWN && armor.color != ArmorColor::UNKNOWN &&
        track.color != armor.color)
    {
      return false;
    }
    if (track.number != ArmorNumber::INVALID && armor.number != ArmorNumber::INVALID &&
        track.number != armor.number)
    {
      return false;
    }
    return true;
  }

  static bool CompatibleTrackLabels(const IndependentArmorTrack& lhs,
                                    const IndependentArmorTrack& rhs)
  {
    if (lhs.type != ArmorType::INVALID && rhs.type != ArmorType::INVALID &&
        lhs.type != rhs.type)
    {
      return false;
    }
    if (lhs.color != ArmorColor::UNKNOWN && rhs.color != ArmorColor::UNKNOWN &&
        lhs.color != rhs.color)
    {
      return false;
    }
    if (lhs.number != ArmorNumber::INVALID && rhs.number != ArmorNumber::INVALID &&
        lhs.number != rhs.number)
    {
      return false;
    }
    return true;
  }

  static bool TrackShouldDisplay(const IndependentArmorTrack& track)
  {
    return track.active && track.confirmed && track.stable_track_id_valid;
  }

  static uint16_t TrackDisplayId(const IndependentArmorTrack& track)
  {
    return track.stable_track_id;
  }

  static cv::Mat ConvertToBgr(const cv::Mat& input)
  {
    if (input.type() == CV_8UC3)
    {
      return input.clone();
    }
    if (input.type() == CV_8UC4)
    {
      cv::Mat output;
      cv::cvtColor(input, output, cv::COLOR_BGRA2BGR);
      return output;
    }
    if (input.type() == CV_8UC1)
    {
      cv::Mat output;
      cv::cvtColor(input, output, cv::COLOR_GRAY2BGR);
      return output;
    }
    return {};
  }

  static const char* AcceptedModeString(uint8_t mode)
  {
    switch (mode)
    {
      case 1:
        return "strict";
      case 2:
        return "relaxed_same_face";
      case 3:
        return "relaxed_face_switch";
      default:
        return "none";
    }
  }

  static std::string TrackerImageIdString(int16_t track_id, bool confirmed)
  {
    if (track_id < 0)
    {
      return "-";
    }
    std::ostringstream ss;
    ss << 'I' << track_id << (confirmed ? '!' : '?');
    return ss.str();
  }

  static int16_t CandidateDetectionTrackId(
      const ArmorTracker::CandidateDebugMsg* candidate_debug, std::size_t armor_index)
  {
    if (candidate_debug == nullptr ||
        armor_index >= ArmorTracker::CandidateDebugMsg::kMaxDetections ||
        armor_index >= candidate_debug->detection_count)
    {
      return -1;
    }
    return candidate_debug->detection_track_ids[armor_index];
  }

  static bool CandidateDetectionTrackConfirmed(
      const ArmorTracker::CandidateDebugMsg* candidate_debug, std::size_t armor_index)
  {
    if (candidate_debug == nullptr ||
        armor_index >= ArmorTracker::CandidateDebugMsg::kMaxDetections ||
        armor_index >= candidate_debug->detection_count)
    {
      return false;
    }
    return candidate_debug->detection_track_confirmed[armor_index] != 0;
  }

  static bool IndependentTrackDebugEnabled()
  {
    const char* env = std::getenv("XR_INDEPENDENT_TRACK_DEBUG");
    return env != nullptr && env[0] != '\0' && env[0] != '0';
  }

  static bool IndependentTrackAuditEnabled()
  {
    const char* env = std::getenv("XR_INDEPENDENT_TRACK_AUDIT");
    return env != nullptr && env[0] != '\0' && env[0] != '0';
  }

  static bool DetectorCornerAuditEnabled()
  {
    const char* env = std::getenv("XR_DETECTOR_CORNER_AUDIT");
    return env != nullptr && env[0] != '\0' && env[0] != '0';
  }

  static bool TruthQuadOverlayEnabled()
  {
    const char* env = std::getenv("XR_TRACKER_DRAW_TRUTH_QUADS");
    return env != nullptr && env[0] != '\0' && env[0] != '0';
  }

  static void DrawPanelRow(cv::Mat& panel, int& y, const std::string& key,
                           const std::string& value, const cv::Scalar& value_color)
  {
    constexpr int kFont = cv::FONT_HERSHEY_DUPLEX;
    constexpr double kScale = 0.48;
    cv::putText(panel, key, cv::Point(16, y), kFont, kScale, cv::Scalar(170, 182, 196),
                1, cv::LINE_AA);
    cv::putText(panel, value, cv::Point(172, y), kFont, kScale, value_color, 1,
                cv::LINE_AA);
    y += 24;
  }

  static void DrawPaneTitle(cv::Mat& frame, const std::string& title)
  {
    cv::rectangle(frame, cv::Rect(0, 0, frame.cols, 34), cv::Scalar(18, 22, 28),
                  cv::FILLED, cv::LINE_AA);
    cv::putText(frame, title, cv::Point(12, 24), cv::FONT_HERSHEY_DUPLEX, 0.62,
                cv::Scalar(240, 244, 250), 1, cv::LINE_AA);
  }

  static void DrawLabel(cv::Mat& frame, const cv::Point& origin, const std::string& text,
                        const cv::Scalar& color)
  {
    constexpr int kFont = cv::FONT_HERSHEY_DUPLEX;
    constexpr double kScale = 0.48;
    constexpr int kThickness = 1;
    int baseline = 0;
    const cv::Size size = cv::getTextSize(text, kFont, kScale, kThickness, &baseline);
    const cv::Rect bg(origin.x, std::max(0, origin.y - size.height - 8),
                      size.width + 10, size.height + 10);
    cv::rectangle(frame, bg, color, cv::FILLED, cv::LINE_AA);
    cv::putText(frame, text, cv::Point(bg.x + 5, bg.y + size.height + 1), kFont, kScale,
                cv::Scalar(12, 16, 24), kThickness, cv::LINE_AA);
  }

  static void DrawSmallLine(cv::Mat& frame, int& y, const std::string& text,
                            const cv::Scalar& color)
  {
    cv::putText(frame, text, cv::Point(16, y), cv::FONT_HERSHEY_DUPLEX, 0.44, color, 1,
                cv::LINE_AA);
    y += 16;
  }

  static const ArmorTracker::CandidateDebugItem* SelectedCandidate(
      const ArmorTracker::CandidateDebugMsg* candidate_debug)
  {
    if (candidate_debug == nullptr)
    {
      return nullptr;
    }
    if (candidate_debug->selected_index >= candidate_debug->count ||
        candidate_debug->selected_index >= ArmorTracker::CandidateDebugMsg::kMaxItems)
    {
      return nullptr;
    }
    return &candidate_debug->items[candidate_debug->selected_index];
  }

  void ImageCallback(cv::Mat* img_msg)
  {
    if (img_msg == nullptr || img_msg->empty() || Done())
    {
      return;
    }
    cv::Mat bgr = ConvertToBgr(*img_msg);
    if (bgr.empty())
    {
      return;
    }
    std::lock_guard<std::mutex> lock(image_lock_);
    latest_image_ = std::move(bgr);
  }

  void ArmorsCallback(ArmorDetectionsMessage* msg)
  {
    if (msg == nullptr)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_lock_);
    latest_armors_ = *msg;
    has_armors_ = true;
    UpdateIndependentTracks(*msg);
  }

  void TrackerInfoCallback(ArmorTracker::TrackerInfo* info)
  {
    if (info == nullptr)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_lock_);
    latest_info_ = *info;
    has_info_ = true;
  }

  void TargetEulrCallback(LibXR::EulerAngle<float>* eulr)
  {
    if (eulr == nullptr)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_lock_);
    latest_target_eulr_ = *eulr;
    has_target_eulr_ = true;
  }

  void FireNotifyCallback(uint8_t* fire)
  {
    if (fire == nullptr)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_lock_);
    latest_fire_notify_ = *fire;
    has_fire_notify_ = true;
  }

  void SendCallback(ArmorTracker::Send* send)
  {
    if (send == nullptr)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_lock_);
    latest_send_ = *send;
    has_send_ = true;
  }

  void EkfPointsCallback(ArmorTracker::EkfPointsMsg* msg)
  {
    if (msg == nullptr)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_lock_);
    latest_ekf_points_ = *msg;
    has_ekf_points_ = true;
    MaybeSyncEkfOverlayLocked();
  }

  void CandidateDebugCallback(ArmorTracker::CandidateDebugMsg* msg)
  {
    if (msg == nullptr)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_lock_);
    latest_candidate_debug_ = *msg;
    has_candidate_debug_ = true;
    MaybeSyncEkfOverlayLocked();
  }

  void MaybeSyncEkfOverlayLocked()
  {
    if (!has_ekf_points_ || !has_candidate_debug_)
    {
      return;
    }
    synced_ekf_points_ = latest_ekf_points_;
    synced_ekf_image_timestamp_us_ = latest_candidate_debug_.image_timestamp_us;
    has_synced_ekf_points_ = true;
  }

  void ResetIndependentTrack(IndependentArmorTrack& track)
  {
    track = IndependentArmorTrack{};
  }

  void AssignIndependentTrack(IndependentArmorTrack& track,
                              const ArmorDetectorResult& armor,
                              uint8_t armor_index, uint64_t image_timestamp_us,
                              double score, double resolved_yaw)
  {
    const bool was_confirmed = track.confirmed;
    const double dt_raw = TimestampDeltaSeconds(image_timestamp_us, track.last_timestamp_us);
    const double dt = dt_raw > 1e-4 ? dt_raw : 1.0 / 100.0;
    const Eigen::Vector3d measured_position(armor.pose.translation.x(),
                                            armor.pose.translation.y(),
                                            armor.pose.translation.z());
    const double measured_yaw = resolved_yaw;
    const double measured_area = std::max(1.0, ArmorArea(armor));

    if (track.age > 0)
    {
      const Eigen::Vector3d measured_velocity =
          (measured_position - track.position) / dt;
      track.velocity = 0.65 * track.velocity + 0.35 * measured_velocity;

      const double measured_yaw_rate =
          ClampAbs((measured_yaw - track.yaw) / dt, kIndependentYawRateMax);
      track.yaw_rate = 0.65 * track.yaw_rate + 0.35 * measured_yaw_rate;

      const cv::Point2f measured_image_velocity(
          static_cast<float>((armor.center.x - track.image_center.x) / dt),
          static_cast<float>((armor.center.y - track.image_center.y) / dt));
      track.image_velocity.x = 0.65f * track.image_velocity.x +
                               0.35f * measured_image_velocity.x;
      track.image_velocity.y = 0.65f * track.image_velocity.y +
                               0.35f * measured_image_velocity.y;

      const double measured_area_rate = (measured_area - track.area) / dt;
      track.area_rate = 0.65 * track.area_rate + 0.35 * measured_area_rate;
    }
    else
    {
      track.velocity.setZero();
      track.yaw_rate = 0.0;
      track.image_velocity = cv::Point2f(0.0f, 0.0f);
      track.area_rate = 0.0;
    }

    track.position = measured_position;
    track.yaw = measured_yaw;
    track.image_center = armor.center;
    track.area = measured_area;
    track.confidence = armor.confidence;
    if (armor.color != ArmorColor::UNKNOWN || track.color == ArmorColor::UNKNOWN)
    {
      track.color = armor.color;
    }
    if (armor.number != ArmorNumber::INVALID || track.number == ArmorNumber::INVALID)
    {
      track.number = armor.number;
    }
    if (armor.type != ArmorType::INVALID || track.type == ArmorType::INVALID)
    {
      track.type = armor.type;
    }
    track.last_timestamp_us = image_timestamp_us;
    track.age++;
    track.hit_count++;
    track.confirmed = track.confirmed || track.hit_count >= 2U;
    if (!was_confirmed && track.confirmed && !track.stable_track_id_valid)
    {
      track.stable_track_id = next_independent_track_id_++;
      track.stable_track_id_valid = true;
      if (IndependentTrackDebugEnabled())
      {
        XR_LOG_WARN(
            "IndependentTrack confirm: iid=%u -> T%u hit=%u det=%u num=%d type=%d",
            static_cast<unsigned>(track.track_id),
            static_cast<unsigned>(track.stable_track_id),
            static_cast<unsigned>(track.hit_count), static_cast<unsigned>(armor_index),
            static_cast<int>(track.number), static_cast<int>(track.type));
      }
    }
    track.miss_count = 0;
    track.matched_this_frame = true;
    track.matched_armor_index = armor_index;
    track.last_match_score = static_cast<float>(score);
    track.active = true;
  }

  void CreateIndependentTrack(const ArmorDetectorResult& armor, uint8_t armor_index,
                              uint64_t image_timestamp_us)
  {
    for (auto& track : independent_tracks_)
    {
      if (track.active)
      {
        continue;
      }
      track = IndependentArmorTrack{};
      track.active = true;
      track.confirmed = false;
      track.track_id = next_independent_internal_track_id_++;
      AssignIndependentTrack(track, armor, armor_index, image_timestamp_us, 0.0,
                           ArmorYaw(armor));
      if (IndependentTrackDebugEnabled())
      {
        XR_LOG_WARN(
            "IndependentTrack create: tid=%u det=%u num=%d type=%d conf=%.2f center=(%.1f, %.1f) xyz=(%.3f, %.3f, %.3f)",
            static_cast<unsigned>(track.track_id), static_cast<unsigned>(armor_index),
            static_cast<int>(armor.number), static_cast<int>(armor.type),
            static_cast<double>(armor.confidence), static_cast<double>(armor.center.x),
            static_cast<double>(armor.center.y), armor.pose.translation.x(),
            armor.pose.translation.y(), armor.pose.translation.z());
      }
      return;
    }
  }

  void UpdateIndependentTracks(const ArmorDetectionsMessage& msg)
  {
    stats_.frames++;
    const bool debug_enabled = IndependentTrackDebugEnabled();
    const bool audit_enabled = IndependentTrackAuditEnabled();
    if (debug_enabled || audit_enabled)
    {
      std::size_t active_before = 0;
      for (const auto& track : independent_tracks_)
      {
        if (track.active)
        {
          active_before++;
        }
      }
      XR_LOG_WARN("IndependentTrack frame: ts=%llu det=%zu active_before=%zu",
                  static_cast<unsigned long long>(msg.image_timestamp_us),
                  msg.results.size(), active_before);
    }

    auto spawn_suppressed = [&](const ArmorDetectorResult& armor)
    {
      for (const auto& track : independent_tracks_)
      {
        if (!track.active || track.miss_count > 6U)
        {
          continue;
        }
        if (!CompatibleTrackLabel(track, armor))
        {
          continue;
        }

        const double dt = std::max(TimestampDeltaSeconds(msg.image_timestamp_us,
                                                         track.last_timestamp_us),
                                   1.0 / 100.0);
        const cv::Point2f predicted_center(
            track.image_center.x + track.image_velocity.x * static_cast<float>(dt),
            track.image_center.y + track.image_velocity.y * static_cast<float>(dt));
        const double center_diff = std::hypot(
            static_cast<double>(armor.center.x - predicted_center.x),
            static_cast<double>(armor.center.y - predicted_center.y));
        const double predicted_area = std::max(1.0, track.area + track.area_rate * dt);
        const double area = std::max(1.0, ArmorArea(armor));
        const double area_log = std::abs(std::log(area / predicted_area));

        const bool strong_same_track =
            (center_diff < 55.0 && area_log < 0.45) ||
            (center_diff < 28.0 && area_log < 0.90);
        const bool relaxed_confirmed_same_track =
            track.confirmed &&
            ((center_diff < 72.0 && area_log < 0.28) ||
             (center_diff < 42.0 && area_log < 0.45));
        if (strong_same_track || relaxed_confirmed_same_track)
        {
          stats_.suppressed_spawn_total++;
          if (debug_enabled)
          {
            XR_LOG_WARN(
                "IndependentTrack suppress spawn: tid=%u confirmed=%d miss=%u num=%d type=%d center=%.1f area_log=%.3f",
                static_cast<unsigned>(track.track_id), track.confirmed ? 1 : 0,
                static_cast<unsigned>(track.miss_count), static_cast<int>(track.number),
                static_cast<int>(track.type), center_diff, area_log);
          }
          return true;
        }
      }
      return false;
    };

    for (auto& track : independent_tracks_)
    {
      if (!track.active)
      {
        continue;
      }
      track.matched_this_frame = false;
      track.matched_armor_index = kInvalidArmorIndex;
    }

    std::vector<IndependentMatchCandidate> candidates;
    candidates.reserve(independent_tracks_.size() * msg.results.size());
    std::array<bool, kMaxIndependentTracks> track_used{};
    std::vector<bool> detection_used(msg.results.size(), false);
    std::vector<int> detection_owner(msg.results.size(), -1);
    uint32_t matched_count = 0;
    uint32_t new_track_count = 0;
    uint32_t removed_count = 0;

    std::array<cv::Point2f, kMaxIndependentTracks> predicted_centers{};
    std::array<bool, kMaxIndependentTracks> predicted_center_valid{};
    for (std::size_t track_slot = 0; track_slot < independent_tracks_.size(); ++track_slot)
    {
      const auto& track = independent_tracks_[track_slot];
      if (!track.active)
      {
        continue;
      }
      const double dt = std::max(TimestampDeltaSeconds(msg.image_timestamp_us,
                                                       track.last_timestamp_us),
                                 1.0 / 100.0);
      predicted_centers[track_slot] = cv::Point2f(
          track.image_center.x + track.image_velocity.x * static_cast<float>(dt),
          track.image_center.y + track.image_velocity.y * static_cast<float>(dt));
      predicted_center_valid[track_slot] = true;
    }

    std::array<std::vector<double>, kMaxIndependentTracks> order_bias_by_track;
    for (auto& bias_vec : order_bias_by_track)
    {
      bias_vec.assign(msg.results.size(), 0.0);
    }

    auto apply_dual_order_bias = [&](std::size_t lhs_slot, std::size_t rhs_slot)
    {
      const auto& lhs_track = independent_tracks_[lhs_slot];
      const auto& rhs_track = independent_tracks_[rhs_slot];
      if (!lhs_track.active || !rhs_track.active || !lhs_track.confirmed ||
          !rhs_track.confirmed)
      {
        return;
      }
      if (!CompatibleTrackLabels(lhs_track, rhs_track) ||
          !predicted_center_valid[lhs_slot] || !predicted_center_valid[rhs_slot])
      {
        return;
      }

      std::size_t compatible_track_count = 0;
      for (std::size_t track_slot = 0; track_slot < independent_tracks_.size(); ++track_slot)
      {
        const auto& track = independent_tracks_[track_slot];
        if (!track.active || !track.confirmed)
        {
          continue;
        }
        if (CompatibleTrackLabels(lhs_track, track))
        {
          compatible_track_count++;
        }
      }
      if (compatible_track_count != 2U)
      {
        return;
      }

      std::vector<std::size_t> compatible_detections;
      for (std::size_t armor_index = 0; armor_index < msg.results.size(); ++armor_index)
      {
        const auto& armor = msg.results[armor_index];
        if (CompatibleTrackLabel(lhs_track, armor) && CompatibleTrackLabel(rhs_track, armor))
        {
          compatible_detections.push_back(armor_index);
        }
      }
      if (compatible_detections.size() != 2U)
      {
        return;
      }

      cv::Point2f axis(predicted_centers[rhs_slot].x - predicted_centers[lhs_slot].x,
                       predicted_centers[rhs_slot].y - predicted_centers[lhs_slot].y);
      double axis_norm = std::hypot(static_cast<double>(axis.x), static_cast<double>(axis.y));
      if (axis_norm < 20.0)
      {
        if (std::abs(axis.x) >= std::abs(axis.y))
        {
          axis = cv::Point2f(1.0f, 0.0f);
        }
        else
        {
          axis = cv::Point2f(0.0f, 1.0f);
        }
        axis_norm = 1.0;
      }
      axis.x = static_cast<float>(axis.x / axis_norm);
      axis.y = static_cast<float>(axis.y / axis_norm);

      auto project = [&](const cv::Point2f& p)
      {
        return static_cast<double>(p.x) * static_cast<double>(axis.x) +
               static_cast<double>(p.y) * static_cast<double>(axis.y);
      };

      std::array<std::size_t, 2> ordered_track_slots = {lhs_slot, rhs_slot};
      if (project(predicted_centers[ordered_track_slots[0]]) >
          project(predicted_centers[ordered_track_slots[1]]))
      {
        std::swap(ordered_track_slots[0], ordered_track_slots[1]);
      }

      std::array<std::size_t, 2> ordered_detection_indices = {compatible_detections[0],
                                                               compatible_detections[1]};
      if (project(msg.results[ordered_detection_indices[0]].center) >
          project(msg.results[ordered_detection_indices[1]].center))
      {
        std::swap(ordered_detection_indices[0], ordered_detection_indices[1]);
      }

      const double detection_sep = std::abs(
          project(msg.results[ordered_detection_indices[1]].center) -
          project(msg.results[ordered_detection_indices[0]].center));
      if (detection_sep < 18.0)
      {
        return;
      }

      const double predicted_sep = std::abs(project(predicted_centers[ordered_track_slots[1]]) -
                                            project(predicted_centers[ordered_track_slots[0]]));
      const double order_bias =
          predicted_sep > 90.0 && detection_sep > 60.0 ? 0.30 : 0.20;

      order_bias_by_track[ordered_track_slots[0]][ordered_detection_indices[0]] -= order_bias;
      order_bias_by_track[ordered_track_slots[0]][ordered_detection_indices[1]] += order_bias;
      order_bias_by_track[ordered_track_slots[1]][ordered_detection_indices[0]] += order_bias;
      order_bias_by_track[ordered_track_slots[1]][ordered_detection_indices[1]] -= order_bias;

      if (audit_enabled)
      {
        XR_LOG_WARN(
            "IndependentTrack audit order: ts=%llu slots=(%zu,%zu) dets=(%zu,%zu) bias=%.3f pred_sep=%.1f det_sep=%.1f",
            static_cast<unsigned long long>(msg.image_timestamp_us),
            ordered_track_slots[0], ordered_track_slots[1], ordered_detection_indices[0],
            ordered_detection_indices[1], order_bias, predicted_sep, detection_sep);
      }
    };

    for (std::size_t lhs_slot = 0; lhs_slot < independent_tracks_.size(); ++lhs_slot)
    {
      for (std::size_t rhs_slot = lhs_slot + 1; rhs_slot < independent_tracks_.size(); ++rhs_slot)
      {
        apply_dual_order_bias(lhs_slot, rhs_slot);
      }
    }

    for (std::size_t track_slot = 0; track_slot < independent_tracks_.size(); ++track_slot)
    {
      const auto& track = independent_tracks_[track_slot];
      if (!track.active)
      {
        continue;
      }

      const double dt = std::max(TimestampDeltaSeconds(msg.image_timestamp_us,
                                                       track.last_timestamp_us),
                                 1.0 / 100.0);
      const double miss_scale =
          static_cast<double>(std::min<uint32_t>(track.miss_count, 6U));
      const cv::Point2f predicted_center = predicted_centers[track_slot];
      const double predicted_area = std::max(1.0, track.area + track.area_rate * dt);

      const double center_score_gate = 80.0 + 15.0 * miss_scale;
      const double center_gate = 140.0 + 20.0 * miss_scale;
      const double area_gate = 0.55 + 0.08 * miss_scale;

      if (audit_enabled)
      {
        XR_LOG_WARN(
            "IndependentTrack audit track: ts=%llu tid=%u slot=%zu confirmed=%d miss=%u hit=%u pred_center=(%.1f, %.1f) pred_area=%.1f gates=(center %.1f area %.3f)",
            static_cast<unsigned long long>(msg.image_timestamp_us),
            static_cast<unsigned>(track.track_id), track_slot, track.confirmed ? 1 : 0,
            static_cast<unsigned>(track.miss_count), static_cast<unsigned>(track.hit_count),
            static_cast<double>(predicted_center.x), static_cast<double>(predicted_center.y),
            predicted_area, center_gate, area_gate);
      }

      for (std::size_t armor_index = 0; armor_index < msg.results.size(); ++armor_index)
      {
        const auto& armor = msg.results[armor_index];
        const bool compatible = CompatibleTrackLabel(track, armor);
        const double center_diff = std::hypot(
            static_cast<double>(armor.center.x - predicted_center.x),
            static_cast<double>(armor.center.y - predicted_center.y));
        const double area = std::max(1.0, ArmorArea(armor));
        const double area_log = std::abs(std::log(area / predicted_area));
        const bool pass_center = center_diff <= center_gate;
        const bool pass_area = area_log <= area_gate;
        const bool candidate_ok = compatible && pass_center && pass_area;

        if (audit_enabled)
        {
          const char* reason = !compatible ? "label"
                               : !pass_center ? "center"
                               : !pass_area ? "area"
                               : "pass";
          XR_LOG_WARN(
              "IndependentTrack audit eval: ts=%llu tid=%u slot=%zu det=%zu compat=%d num=%s type=%s conf=%.2f center=%.1f/%.1f area=%.3f/%.3f candidate=%d reason=%s",
              static_cast<unsigned long long>(msg.image_timestamp_us),
              static_cast<unsigned>(track.track_id), track_slot, armor_index,
              compatible ? 1 : 0, ArmorNumberString(armor.number).c_str(),
              ArmorTypeString(armor.type).c_str(), static_cast<double>(armor.confidence),
              center_diff, center_gate, area_log, area_gate, candidate_ok ? 1 : 0,
              reason);
        }

        if (!candidate_ok)
        {
          continue;
        }

        const double order_bias = order_bias_by_track[track_slot][armor_index];
        double score = 0.78 * center_diff / center_score_gate +
                       0.22 * area_log / area_gate -
                       0.05 * static_cast<double>(armor.confidence) +
                       order_bias;
        if (track.confirmed)
        {
          score -= 0.18;
        }
        if (track.miss_count == 0U)
        {
          score -= 0.08;
        }
        if (track.confirmed && track.miss_count == 0U && center_diff < 18.0 &&
            area_log < 0.12)
        {
          score -= 0.16;
        }
        candidates.push_back(IndependentMatchCandidate{track_slot, armor_index, score,
                                                       order_bias, center_diff, area_log,
                                                       0.0, 0.0, ArmorYaw(armor)});
      }
    }

    stats_.candidate_total += static_cast<uint32_t>(
        std::min<std::size_t>(candidates.size(),
                              static_cast<std::size_t>(std::numeric_limits<uint32_t>::max())));

    auto candidate_less = [&](const IndependentMatchCandidate& lhs,
                              const IndependentMatchCandidate& rhs)
    {
      const auto& lhs_track = independent_tracks_[lhs.track_slot];
      const auto& rhs_track = independent_tracks_[rhs.track_slot];
      if (lhs_track.confirmed != rhs_track.confirmed &&
          std::abs(lhs.score - rhs.score) < 0.25)
      {
        return lhs_track.confirmed > rhs_track.confirmed;
      }
      if (lhs_track.miss_count != rhs_track.miss_count &&
          std::abs(lhs.score - rhs.score) < 0.20)
      {
        return lhs_track.miss_count < rhs_track.miss_count;
      }
      if (std::abs(lhs.score - rhs.score) > 1e-6)
      {
        return lhs.score < rhs.score;
      }
      return lhs_track.track_id < rhs_track.track_id;
    };

    std::vector<std::size_t> ordered_candidate_indices;
    ordered_candidate_indices.reserve(candidates.size());
    for (std::size_t candidate_index = 0; candidate_index < candidates.size();
         ++candidate_index)
    {
      ordered_candidate_indices.push_back(candidate_index);
    }
    std::sort(ordered_candidate_indices.begin(), ordered_candidate_indices.end(),
              [&](std::size_t lhs_index, std::size_t rhs_index)
              {
                return candidate_less(candidates[lhs_index], candidates[rhs_index]);
              });

    std::vector<bool> candidate_selected(candidates.size(), false);
    std::array<bool, kMaxIndependentTracks> selected_track_slots{};
    std::vector<bool> selected_detection_slots(msg.results.size(), false);
    std::vector<int> selected_detection_owner(msg.results.size(), -1);

    std::array<std::vector<std::size_t>, kMaxIndependentTracks> track_candidate_indices;
    std::vector<int> detection_bit_indices(msg.results.size(), -1);
    std::vector<uint8_t> candidate_detection_bits(candidates.size(), 0);
    std::vector<std::size_t> unique_detection_indices;
    unique_detection_indices.reserve(msg.results.size());
    for (std::size_t candidate_index = 0; candidate_index < candidates.size();
         ++candidate_index)
    {
      const auto& candidate = candidates[candidate_index];
      track_candidate_indices[candidate.track_slot].push_back(candidate_index);
      if (detection_bit_indices[candidate.armor_index] < 0)
      {
        detection_bit_indices[candidate.armor_index] =
            static_cast<int>(unique_detection_indices.size());
        unique_detection_indices.push_back(candidate.armor_index);
      }
      candidate_detection_bits[candidate_index] = static_cast<uint8_t>(
          detection_bit_indices[candidate.armor_index]);
    }

    for (auto& candidate_indices : track_candidate_indices)
    {
      std::sort(candidate_indices.begin(), candidate_indices.end(),
                [&](std::size_t lhs_index, std::size_t rhs_index)
                {
                  return candidate_less(candidates[lhs_index],
                                        candidates[rhs_index]);
                });
    }

    bool used_global_assignment = false;
    std::vector<std::size_t> assignment_track_slots;
    assignment_track_slots.reserve(independent_tracks_.size());
    for (std::size_t track_slot = 0; track_slot < independent_tracks_.size(); ++track_slot)
    {
      if (!track_candidate_indices[track_slot].empty())
      {
        assignment_track_slots.push_back(track_slot);
      }
    }
    std::sort(assignment_track_slots.begin(), assignment_track_slots.end(),
              [&](std::size_t lhs_slot, std::size_t rhs_slot)
              {
                const auto& lhs_track = independent_tracks_[lhs_slot];
                const auto& rhs_track = independent_tracks_[rhs_slot];
                if (lhs_track.confirmed != rhs_track.confirmed)
                {
                  return lhs_track.confirmed > rhs_track.confirmed;
                }
                if (lhs_track.miss_count != rhs_track.miss_count)
                {
                  return lhs_track.miss_count < rhs_track.miss_count;
                }
                if (track_candidate_indices[lhs_slot].size() !=
                    track_candidate_indices[rhs_slot].size())
                {
                  return track_candidate_indices[lhs_slot].size() <
                         track_candidate_indices[rhs_slot].size();
                }
                return lhs_track.track_id < rhs_track.track_id;
              });

    const std::size_t unique_detection_count = unique_detection_indices.size();
    if (!assignment_track_slots.empty() && unique_detection_count <= 16U)
    {
      struct AssignmentState
      {
        bool valid{false};
        uint16_t matched_count{0};
        uint16_t confirmed_matched_count{0};
        double total_score{0.0};
      };

      const std::size_t state_count = std::size_t{1} << unique_detection_count;
      const std::size_t memo_size =
          (assignment_track_slots.size() + 1U) * state_count;
      std::vector<uint8_t> memo_seen(memo_size, 0);
      std::vector<AssignmentState> memo(memo_size);
      std::vector<int32_t> memo_choice(memo_size, -2);

      auto assignment_better = [&](const AssignmentState& lhs,
                                   const AssignmentState& rhs)
      {
        if (!lhs.valid)
        {
          return false;
        }
        if (!rhs.valid)
        {
          return true;
        }
        if (lhs.matched_count != rhs.matched_count)
        {
          return lhs.matched_count > rhs.matched_count;
        }
        if (lhs.confirmed_matched_count != rhs.confirmed_matched_count)
        {
          return lhs.confirmed_matched_count > rhs.confirmed_matched_count;
        }
        if (std::abs(lhs.total_score - rhs.total_score) > 1e-9)
        {
          return lhs.total_score < rhs.total_score;
        }
        return false;
      };

      auto memo_index_of = [&](std::size_t track_order_index, std::size_t used_mask)
      {
        return track_order_index * state_count + used_mask;
      };

      auto solve_assignment =
          [&](auto&& self, std::size_t track_order_index,
              std::size_t used_mask) -> AssignmentState
      {
        const std::size_t memo_index =
            memo_index_of(track_order_index, used_mask);
        if (memo_seen[memo_index] != 0U)
        {
          return memo[memo_index];
        }
        memo_seen[memo_index] = 1U;

        AssignmentState best;
        best.valid = true;
        memo_choice[memo_index] = -1;
        if (track_order_index >= assignment_track_slots.size())
        {
          memo[memo_index] = best;
          return best;
        }

        best = self(self, track_order_index + 1U, used_mask);

        const std::size_t track_slot = assignment_track_slots[track_order_index];
        const auto& track = independent_tracks_[track_slot];
        for (std::size_t candidate_index : track_candidate_indices[track_slot])
        {
          const std::size_t detection_bit = candidate_detection_bits[candidate_index];
          const std::size_t detection_mask = std::size_t{1} << detection_bit;
          if ((used_mask & detection_mask) != 0U)
          {
            continue;
          }

          AssignmentState next =
              self(self, track_order_index + 1U, used_mask | detection_mask);
          if (!next.valid)
          {
            continue;
          }
          next.matched_count++;
          if (track.confirmed)
          {
            next.confirmed_matched_count++;
          }
          next.total_score += candidates[candidate_index].score;
          if (assignment_better(next, best))
          {
            best = next;
            memo_choice[memo_index] = static_cast<int32_t>(candidate_index);
          }
        }

        memo[memo_index] = best;
        return best;
      };

      const AssignmentState best_assignment =
          solve_assignment(solve_assignment, 0U, 0U);
      if (best_assignment.valid)
      {
        used_global_assignment = true;
        std::size_t used_mask = 0U;
        for (std::size_t track_order_index = 0;
             track_order_index < assignment_track_slots.size(); ++track_order_index)
        {
          const std::size_t memo_index =
              memo_index_of(track_order_index, used_mask);
          const int32_t choice = memo_choice[memo_index];
          if (choice < 0)
          {
            continue;
          }
          const std::size_t candidate_index = static_cast<std::size_t>(choice);
          candidate_selected[candidate_index] = true;
          used_mask |= std::size_t{1} << candidate_detection_bits[candidate_index];
        }
      }
    }

    if (!used_global_assignment)
    {
      for (std::size_t candidate_index : ordered_candidate_indices)
      {
        const auto& candidate = candidates[candidate_index];
        if (selected_track_slots[candidate.track_slot] ||
            selected_detection_slots[candidate.armor_index])
        {
          continue;
        }
        candidate_selected[candidate_index] = true;
        selected_track_slots[candidate.track_slot] = true;
        selected_detection_slots[candidate.armor_index] = true;
        selected_detection_owner[candidate.armor_index] = static_cast<int>(
            independent_tracks_[candidate.track_slot].track_id);
      }
    }

    std::fill(selected_track_slots.begin(), selected_track_slots.end(), false);
    std::fill(selected_detection_slots.begin(), selected_detection_slots.end(), false);
    std::fill(selected_detection_owner.begin(), selected_detection_owner.end(), -1);
    for (std::size_t candidate_index = 0; candidate_index < candidates.size();
         ++candidate_index)
    {
      if (!candidate_selected[candidate_index])
      {
        continue;
      }
      const auto& candidate = candidates[candidate_index];
      selected_track_slots[candidate.track_slot] = true;
      selected_detection_slots[candidate.armor_index] = true;
      selected_detection_owner[candidate.armor_index] = static_cast<int>(
          independent_tracks_[candidate.track_slot].track_id);
    }

    if (audit_enabled)
    {
      XR_LOG_WARN(
          "IndependentTrack audit assign: ts=%llu mode=%s unique_det=%zu candidate=%zu",
          static_cast<unsigned long long>(msg.image_timestamp_us),
          used_global_assignment ? "optimal" : "greedy", unique_detection_count,
          candidates.size());
      for (std::size_t candidate_rank = 0;
           candidate_rank < ordered_candidate_indices.size(); ++candidate_rank)
      {
        const std::size_t candidate_index =
            ordered_candidate_indices[candidate_rank];
        const auto& candidate = candidates[candidate_index];
        const auto& track = independent_tracks_[candidate.track_slot];
        const bool selected = candidate_selected[candidate_index];
        const bool skip_track_used =
            !selected && selected_track_slots[candidate.track_slot];
        const bool skip_det_used =
            !selected && selected_detection_slots[candidate.armor_index];
        const int owner_tid = skip_det_used
                                  ? selected_detection_owner[candidate.armor_index]
                                  : -1;
        const char* action = selected ? "select"
                              : skip_track_used ? "skip_track_used"
                              : skip_det_used ? "skip_det_used"
                              : "skip_not_selected";
        XR_LOG_WARN(
            "IndependentTrack audit cand: ts=%llu rank=%zu tid=%u slot=%zu det=%zu score=%.3f ord=%.3f confirmed=%d miss=%u center=%.1f area_log=%.3f pos=%.3f yaw=%.3f action=%s owner_tid=%d",
            static_cast<unsigned long long>(msg.image_timestamp_us), candidate_rank,
            static_cast<unsigned>(track.track_id), candidate.track_slot,
            candidate.armor_index, candidate.score, candidate.order_bias,
            track.confirmed ? 1 : 0, static_cast<unsigned>(track.miss_count),
            candidate.center_diff, candidate.area_log, candidate.position_diff,
            candidate.yaw_diff, action, owner_tid);
      }
    }

    for (std::size_t candidate_index : ordered_candidate_indices)
    {
      if (!candidate_selected[candidate_index])
      {
        continue;
      }
      const auto& candidate = candidates[candidate_index];
      auto& track = independent_tracks_[candidate.track_slot];
      AssignIndependentTrack(track, msg.results[candidate.armor_index],
                             static_cast<uint8_t>(candidate.armor_index),
                             msg.image_timestamp_us, candidate.score,
                             candidate.resolved_yaw);
      track_used[candidate.track_slot] = true;
      detection_used[candidate.armor_index] = true;
      detection_owner[candidate.armor_index] = static_cast<int>(track.track_id);
      stats_.matched_total++;
      if (track.confirmed)
      {
        matched_count++;
      }
      if (debug_enabled)
      {
        XR_LOG_WARN(
            "IndependentTrack match: tid=%u slot=%zu det=%zu confirmed=%d miss=%u score=%.3f ord=%.3f center=%.1f area_log=%.3f pos=%.3f yaw=%.3f mode=%s",
            static_cast<unsigned>(track.track_id), candidate.track_slot,
            candidate.armor_index, track.confirmed ? 1 : 0,
            static_cast<unsigned>(track.miss_count), candidate.score, candidate.order_bias,
            candidate.center_diff, candidate.area_log, candidate.position_diff,
            candidate.yaw_diff, used_global_assignment ? "optimal" : "greedy");
      }
    }

    for (std::size_t track_slot = 0; track_slot < independent_tracks_.size(); ++track_slot)
    {
      auto& track = independent_tracks_[track_slot];
      if (!track.active || track_used[track_slot])
      {
        continue;
      }

      const double dt = TimestampDeltaSeconds(msg.image_timestamp_us, track.last_timestamp_us);
      if (dt > 1e-4)
      {
        track.position += track.velocity * dt;
        track.yaw += track.yaw_rate * dt;
        track.image_center.x += track.image_velocity.x * static_cast<float>(dt);
        track.image_center.y += track.image_velocity.y * static_cast<float>(dt);
        track.area = std::max(1.0, track.area + track.area_rate * dt);
        track.last_timestamp_us = msg.image_timestamp_us;
      }
      track.miss_count++;
      track.matched_this_frame = false;
      track.matched_armor_index = kInvalidArmorIndex;
      if (!track.confirmed && track.miss_count > 2U)
      {
        if (debug_enabled)
        {
          XR_LOG_WARN(
              "IndependentTrack remove tentative: tid=%u hit=%u miss=%u age=%u",
              static_cast<unsigned>(track.track_id), static_cast<unsigned>(track.hit_count),
              static_cast<unsigned>(track.miss_count), static_cast<unsigned>(track.age));
        }
        ResetIndependentTrack(track);
        removed_count++;
        stats_.removed_total++;
        continue;
      }
      if (track.miss_count > 8U)
      {
        if (debug_enabled)
        {
          XR_LOG_WARN(
              "IndependentTrack remove stale: tid=%u hit=%u miss=%u age=%u",
              static_cast<unsigned>(track.track_id), static_cast<unsigned>(track.hit_count),
              static_cast<unsigned>(track.miss_count), static_cast<unsigned>(track.age));
        }
        ResetIndependentTrack(track);
        removed_count++;
        stats_.removed_total++;
      }
    }

    for (std::size_t armor_index = 0; armor_index < msg.results.size(); ++armor_index)
    {
      if (detection_used[armor_index])
      {
        continue;
      }
      if (spawn_suppressed(msg.results[armor_index]))
      {
        continue;
      }
      CreateIndependentTrack(msg.results[armor_index],
                             static_cast<uint8_t>(armor_index),
                             msg.image_timestamp_us);
      detection_used[armor_index] = true;
      new_track_count++;
      stats_.new_track_total++;
    }

    auto keep_lhs_duplicate = [](const IndependentArmorTrack& lhs,
                                 const IndependentArmorTrack& rhs)
    {
      if (lhs.matched_this_frame != rhs.matched_this_frame)
      {
        return lhs.matched_this_frame;
      }
      if (lhs.miss_count != rhs.miss_count)
      {
        return lhs.miss_count < rhs.miss_count;
      }
      if (lhs.hit_count != rhs.hit_count)
      {
        return lhs.hit_count > rhs.hit_count;
      }
      if (lhs.age != rhs.age)
      {
        return lhs.age > rhs.age;
      }
      return lhs.track_id < rhs.track_id;
    };

    uint32_t duplicate_pairs = 0;
    for (std::size_t i = 0; i < independent_tracks_.size(); ++i)
    {
      auto& lhs = independent_tracks_[i];
      if (!TrackShouldDisplay(lhs))
      {
        continue;
      }
      for (std::size_t j = i + 1; j < independent_tracks_.size(); ++j)
      {
        auto& rhs = independent_tracks_[j];
        if (!TrackShouldDisplay(rhs) || !CompatibleTrackLabels(lhs, rhs))
        {
          continue;
        }
        const double center_diff = std::hypot(
            static_cast<double>(lhs.image_center.x - rhs.image_center.x),
            static_cast<double>(lhs.image_center.y - rhs.image_center.y));
        const double position_diff = (lhs.position - rhs.position).norm();
        if (center_diff < 40.0 || position_diff < 0.22 ||
            ((lhs.miss_count > 3U || rhs.miss_count > 3U) && center_diff < 70.0 &&
             position_diff < 0.40))
        {
          duplicate_pairs++;
          const bool keep_lhs = keep_lhs_duplicate(lhs, rhs);
          auto& loser = keep_lhs ? rhs : lhs;
          if (debug_enabled)
          {
            XR_LOG_WARN(
                "IndependentTrack merge duplicate: keep=%u drop=%u center=%.1f pos=%.3f miss=(%u,%u) hit=(%u,%u)",
                static_cast<unsigned>(keep_lhs ? lhs.track_id : rhs.track_id),
                static_cast<unsigned>(loser.track_id), center_diff, position_diff,
                static_cast<unsigned>(lhs.miss_count), static_cast<unsigned>(rhs.miss_count),
                static_cast<unsigned>(lhs.hit_count), static_cast<unsigned>(rhs.hit_count));
          }
          ResetIndependentTrack(loser);
          removed_count++;
          stats_.removed_total++;
          if (!keep_lhs)
          {
            break;
          }
          continue;
        }
        if (center_diff < 80.0 || position_diff < 0.25)
        {
          duplicate_pairs++;
          if (debug_enabled)
          {
            XR_LOG_WARN(
                "IndependentTrack duplicate pair: tid=%u tid=%u center=%.1f pos=%.3f miss=(%u,%u)",
                static_cast<unsigned>(lhs.track_id), static_cast<unsigned>(rhs.track_id),
                center_diff, position_diff, static_cast<unsigned>(lhs.miss_count),
                static_cast<unsigned>(rhs.miss_count));
          }
        }
      }
    }
    if (duplicate_pairs > 0)
    {
      stats_.duplicate_pair_frames++;
      stats_.duplicate_pair_total += duplicate_pairs;
    }

    latest_independent_tracks_ = IndependentTracksSnapshot{};
    latest_independent_tracks_.image_timestamp_us = msg.image_timestamp_us;
    latest_independent_tracks_.detection_count = static_cast<uint8_t>(
        std::min<std::size_t>(msg.results.size(), std::numeric_limits<uint8_t>::max()));
    latest_independent_tracks_.matched_count = static_cast<uint8_t>(
        std::min<uint32_t>(matched_count, std::numeric_limits<uint8_t>::max()));
    latest_independent_tracks_.new_track_count = static_cast<uint8_t>(
        std::min<uint32_t>(new_track_count, std::numeric_limits<uint8_t>::max()));
    latest_independent_tracks_.removed_count = static_cast<uint8_t>(
        std::min<uint32_t>(removed_count, std::numeric_limits<uint8_t>::max()));

    std::vector<IndependentArmorTrack> ordered_tracks;
    ordered_tracks.reserve(independent_tracks_.size());
    for (const auto& track : independent_tracks_)
    {
      if (TrackShouldDisplay(track))
      {
        ordered_tracks.push_back(track);
      }
    }
    std::sort(ordered_tracks.begin(), ordered_tracks.end(),
              [](const IndependentArmorTrack& lhs, const IndependentArmorTrack& rhs)
              { return TrackDisplayId(lhs) < TrackDisplayId(rhs); });
    latest_independent_tracks_.active_count = static_cast<uint8_t>(
        std::min<std::size_t>(ordered_tracks.size(), std::numeric_limits<uint8_t>::max()));
    for (std::size_t i = 0;
         i < ordered_tracks.size() && i < latest_independent_tracks_.tracks.size(); ++i)
    {
      latest_independent_tracks_.tracks[i] = ordered_tracks[i];
    }
    has_independent_tracks_ = true;
  }

  const IndependentArmorTrack* FindIndependentTrackForArmor(
      const IndependentTracksSnapshot* independent_tracks, std::size_t armor_index) const
  {
    if (independent_tracks == nullptr)
    {
      return nullptr;
    }
    for (const auto& track : independent_tracks->tracks)
    {
      if (!TrackShouldDisplay(track) || !track.matched_this_frame)
      {
        continue;
      }
      if (track.matched_armor_index == armor_index)
      {
        return &track;
      }
    }
    return nullptr;
  }

  struct Pose3d
  {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  };

  static Eigen::Matrix3d AxisAngleToRotation(const double* rot)
  {
    if (rot == nullptr)
    {
      return Eigen::Matrix3d::Identity();
    }
    const Eigen::Vector3d axis(rot[0], rot[1], rot[2]);
    const double axis_norm = axis.norm();
    if (!(axis_norm > 1e-9) || !std::isfinite(rot[3]))
    {
      return Eigen::Matrix3d::Identity();
    }
    return Eigen::AngleAxisd(rot[3], axis / axis_norm).toRotationMatrix();
  }

  static bool ReadTopLevelRobotPose(webots::Node* node, Pose3d& pose)
  {
    if (node == nullptr)
    {
      return false;
    }
    webots::Field* translation_field = node->getField("translation");
    webots::Field* rotation_field = node->getField("rotation");
    if (translation_field == nullptr || rotation_field == nullptr)
    {
      return false;
    }
    const double* t = translation_field->getSFVec3f();
    const double* r = rotation_field->getSFRotation();
    if (t == nullptr || r == nullptr)
    {
      return false;
    }
    pose.translation = Eigen::Vector3d(t[0], t[1], t[2]);
    pose.rotation = AxisAngleToRotation(r);
    return true;
  }

  static bool ReadRelativePose(webots::Node* node, const webots::Node* from_node,
                               Pose3d& pose, bool& logged, const char* label)
  {
    if (node == nullptr || from_node == nullptr)
    {
      return false;
    }
    const double* m = node->getPose(from_node);
    if (m == nullptr)
    {
      return false;
    }
    const Eigen::Vector3d t_row(m[3], m[7], m[11]);
    const Eigen::Vector3d t_col(m[12], m[13], m[14]);
    Eigen::Matrix3d r_row = Eigen::Matrix3d::Identity();
    r_row << m[0], m[1], m[2],
             m[4], m[5], m[6],
             m[8], m[9], m[10];
    Eigen::Matrix3d r_col = Eigen::Matrix3d::Identity();
    r_col << m[0], m[4], m[8],
             m[1], m[5], m[9],
             m[2], m[6], m[10];
    const bool use_col_major = t_col.norm() > t_row.norm();
    pose.translation = use_col_major ? t_col : t_row;
    pose.rotation = use_col_major ? r_col : r_row;
    if (!logged)
    {
      XR_LOG_PASS(
          "TrackerVideoRecorder %s row_t=(%.3f, %.3f, %.3f) col_t=(%.3f, %.3f, %.3f) choose=%s",
          label, t_row.x(), t_row.y(), t_row.z(), t_col.x(), t_col.y(), t_col.z(),
          use_col_major ? "col" : "row");
      logged = true;
    }
    return true;
  }

  bool ResolveNamedRobotNode(const char* name, webots::Node*& out)
  {
    if (out != nullptr)
    {
      return true;
    }
    if (supervisor_ == nullptr)
    {
      return false;
    }
    webots::Node* root = supervisor_->getRoot();
    if (root == nullptr)
    {
      return false;
    }
    webots::Field* children = root->getField("children");
    if (children == nullptr)
    {
      return false;
    }
    for (int i = 0; i < children->getCount(); ++i)
    {
      webots::Node* node = children->getMFNode(i);
      if (node == nullptr)
      {
        continue;
      }
      webots::Field* name_field = node->getField("name");
      if (name_field == nullptr)
      {
        continue;
      }
      if (name_field->getSFString() == name)
      {
        out = node;
        return true;
      }
    }
    return false;
  }

  static webots::Node* FindNamedNodeRecursiveImpl(webots::Node* node,
                                                  const char* wanted_name,
                                                  std::unordered_set<const webots::Node*>& visited)
  {
    if (node == nullptr || wanted_name == nullptr)
    {
      return nullptr;
    }

    webots::Field* name_field = node->getField("name");
    if (name_field == nullptr && node->isProto())
    {
      name_field = node->getBaseNodeField("name");
    }
    if (name_field != nullptr && name_field->getSFString() == wanted_name)
    {
      return node;
    }

    if (!visited.insert(node).second)
    {
      return nullptr;
    }

    auto search_field = [&](webots::Field* field) -> webots::Node*
    {
      if (field == nullptr)
      {
        return nullptr;
      }

      if (field->getType() == webots::Field::SF_NODE)
      {
        if (webots::Node* found =
                FindNamedNodeRecursiveImpl(field->getSFNode(), wanted_name, visited))
        {
          return found;
        }
      }
      else if (field->getType() == webots::Field::MF_NODE)
      {
        for (int i = 0; i < field->getCount(); ++i)
        {
          if (webots::Node* found =
                  FindNamedNodeRecursiveImpl(field->getMFNode(i), wanted_name, visited))
          {
            return found;
          }
        }
      }
      return nullptr;
    };

    for (int i = 0; i < node->getNumberOfFields(); ++i)
    {
      if (webots::Node* found = search_field(node->getFieldByIndex(i)))
      {
        return found;
      }
    }
    if (node->isProto())
    {
      for (int i = 0; i < node->getNumberOfBaseNodeFields(); ++i)
      {
        if (webots::Node* found = search_field(node->getBaseNodeFieldByIndex(i)))
        {
          return found;
        }
      }
    }
    return nullptr;
  }

  static webots::Node* FindNamedNodeRecursive(webots::Node* node, const char* wanted_name)
  {
    std::unordered_set<const webots::Node*> visited;
    return FindNamedNodeRecursiveImpl(node, wanted_name, visited);
  }

  bool ResolveTruthNodes()
  {
    bool all_armors_resolved = true;
    for (auto* node : armor_nodes_)
    {
      all_armors_resolved = all_armors_resolved && (node != nullptr);
    }
    if (target_spin_node_ != nullptr && camera_node_ != nullptr &&
        target_robot_node_ != nullptr && self_robot_node_ != nullptr &&
        all_armors_resolved)
    {
      return true;
    }
    if (supervisor_ == nullptr)
    {
      return false;
    }
    target_spin_node_ = supervisor_->getFromDef("TARGET_SPIN");
    camera_node_ = supervisor_->getFromDef("camera");
    if (target_spin_node_ == nullptr || camera_node_ == nullptr)
    {
      if (!truth_resolve_failure_logged_)
      {
        XR_LOG_WARN("TrackerVideoRecorder truth resolve failed: target_spin=%p camera=%p",
                    static_cast<void*>(target_spin_node_),
                    static_cast<void*>(camera_node_));
        truth_resolve_failure_logged_ = true;
      }
      return false;
    }
    if (!ResolveNamedRobotNode("target", target_robot_node_))
    {
      if (!truth_resolve_failure_logged_)
      {
        XR_LOG_WARN("TrackerVideoRecorder truth resolve failed: missing target robot");
        truth_resolve_failure_logged_ = true;
      }
      return false;
    }
    if (!ResolveNamedRobotNode("self", self_robot_node_))
    {
      if (!truth_resolve_failure_logged_)
      {
        XR_LOG_WARN("TrackerVideoRecorder truth resolve failed: missing self robot");
        truth_resolve_failure_logged_ = true;
      }
      return false;
    }
    webots::Node* root = supervisor_->getRoot();
    if (root == nullptr)
    {
      return false;
    }
    for (std::size_t i = 0; i < armor_nodes_.size(); ++i)
    {
      if (armor_nodes_[i] == nullptr)
      {
        armor_nodes_[i] = FindNamedNodeRecursive(root, kTruthNodeLabels[i]);
      }
      if (armor_nodes_[i] == nullptr)
      {
        if (!truth_resolve_failure_logged_)
        {
          XR_LOG_WARN("TrackerVideoRecorder truth resolve failed: missing armor node %s",
                      kTruthNodeLabels[i]);
          truth_resolve_failure_logged_ = true;
        }
        return false;
      }
      if (visible_face_nodes_[i] == nullptr && armor_nodes_[i]->isProto())
      {
        visible_face_nodes_[i] = armor_nodes_[i]->getFromProtoDef(kTruthVisibleFaceProtoDef);
      }
    }
    XR_LOG_PASS("TrackerVideoRecorder resolved target/self/camera/armor nodes");
    truth_resolve_failure_logged_ = false;
    return true;
  }

  bool BuildVisibleTruthFaces(
      const CameraBase::CameraInfo& camera_info,
      std::array<WebotsTruthVisibleFace, 4>& truth_faces)
  {
    if (!ResolveTruthNodes())
    {
      return false;
    }

    for (auto& face : truth_faces)
    {
      face = WebotsTruthVisibleFace{};
    }

    for (int face_index = 0; face_index < 4; ++face_index)
    {
      Pose3d armor_in_camera_node;
      if (!ReadRelativePose(armor_nodes_[face_index], camera_node_, armor_in_camera_node,
                            armor_pose_logged_[face_index], kTruthNodeLabels[face_index]))
      {
        continue;
      }

      if (visible_face_nodes_[face_index] != nullptr)
      {
        Pose3d visible_face_in_camera_node;
        if (ReadRelativePose(visible_face_nodes_[face_index], camera_node_,
                             visible_face_in_camera_node,
                             visible_face_pose_logged_[face_index],
                             kTruthVisibleFaceProtoDef))
        {
          const Eigen::Matrix3d r_optical_box =
              WebotsCameraNodeToOpticalRotation() * visible_face_in_camera_node.rotation;
          const Eigen::Vector3d t_optical_box =
              WebotsCameraNodeToOpticalRotation() * visible_face_in_camera_node.translation;
          if (ProjectWebotsTruthFaceBox(camera_info, r_optical_box, t_optical_box,
                                        kWebotsTruthSpArmorWidth,
                                        kWebotsTruthSpArmorHeight,
                                        kWebotsTruthSpArmorDepth,
                                        truth_faces[face_index]))
          {
            continue;
          }
        }
      }

      const Eigen::Matrix3d r_optical_armor_root =
          WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.rotation;
      const Eigen::Vector3d t_optical_armor_root =
          WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.translation;
      ProjectWebotsTruthVisibleFace(camera_info, r_optical_armor_root,
                                    t_optical_armor_root, truth_faces[face_index]);
    }
    return true;
  }

  bool ReadTruthCamera(const CameraBase::CameraInfo& camera_info,
                       std::array<Eigen::Vector3d, 4>& gt_cam,
                       std::array<bool, 4>& gt_visible)
  {
    (void)camera_info;
    if (!ResolveTruthNodes())
    {
      return false;
    }

    const Eigen::Matrix3d r_optical = WebotsCameraNodeToOpticalRotation();
    for (int i = 0; i < 4; ++i)
    {
      gt_cam[i] = Eigen::Vector3d::Zero();
      gt_visible[i] = false;
      Pose3d armor_in_camera_node;
      if (!ReadRelativePose(armor_nodes_[i], camera_node_, armor_in_camera_node,
                            armor_pose_logged_[i], kTruthNodeLabels[i]))
      {
        continue;
      }
      gt_cam[i] = r_optical * armor_in_camera_node.translation;
      gt_visible[i] = std::isfinite(gt_cam[i].x()) && std::isfinite(gt_cam[i].y()) &&
                      std::isfinite(gt_cam[i].z()) && gt_cam[i].z() > 1e-6;
    }
    return true;
  }


  static double MeanOf(const std::vector<double>& values)
  {
    if (values.empty())
    {
      return 0.0;
    }
    const double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / static_cast<double>(values.size());
  }

  static double Percentile(std::vector<double> values, double p)
  {
    if (values.empty())
    {
      return 0.0;
    }
    const double clamped = std::clamp(p, 0.0, 1.0);
    const std::size_t index = static_cast<std::size_t>(
        std::llround(clamped * static_cast<double>(values.size() - 1)));
    std::nth_element(values.begin(), values.begin() + static_cast<long>(index),
                     values.end());
    return values[index];
  }

  static std::array<cv::Point2f, 4> SortArmorPoints(std::array<cv::Point2f, 4> points)
  {
    const cv::Point2f center =
        (points[0] + points[1] + points[2] + points[3]) * 0.25f;
    std::sort(points.begin(), points.end(),
              [&](const cv::Point2f& lhs, const cv::Point2f& rhs)
              {
                const double lhs_angle = std::atan2(lhs.y - center.y, lhs.x - center.x);
                const double rhs_angle = std::atan2(rhs.y - center.y, rhs.x - center.x);
                return lhs_angle < rhs_angle;
              });
    return points;
  }

  static std::array<cv::Point3f, 4> SmallArmorObjectPoints()
  {
    constexpr float kHalfY = 135.0f * 0.5f / 1000.0f;
    constexpr float kHalfZ = 55.0f * 0.5f / 1000.0f;
    return {cv::Point3f(0.0f, kHalfY, kHalfZ), cv::Point3f(0.0f, -kHalfY, kHalfZ),
            cv::Point3f(0.0f, -kHalfY, -kHalfZ), cv::Point3f(0.0f, kHalfY, -kHalfZ)};
  }

  static Eigen::Matrix3d FaceRotationSpin(int face_index)
  {
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitY();
    switch (face_index)
    {
      case 0:
        x_axis = Eigen::Vector3d::UnitY();
        break;
      case 1:
        x_axis = Eigen::Vector3d::UnitX();
        break;
      case 2:
        x_axis = -Eigen::Vector3d::UnitY();
        break;
      case 3:
        x_axis = -Eigen::Vector3d::UnitX();
        break;
      default:
        break;
    }

    const Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d y_axis = (z_axis.cross(x_axis)).normalized();

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;
    return rotation;
  }

  static bool FinitePoint(const cv::Point2f& point)
  {
    return std::isfinite(point.x) && std::isfinite(point.y);
  }

  bool ReadTruthProjectedCorners(
      const CameraBase::CameraInfo& camera_info,
      std::array<std::array<cv::Point2f, 4>, 4>& gt_points,
      std::array<cv::Point2f, 4>& gt_centers,
      std::array<bool, 4>& gt_valid)
  {
    std::array<WebotsTruthVisibleFace, 4> truth_faces{};
    if (!BuildVisibleTruthFaces(camera_info, truth_faces))
    {
      return false;
    }

    gt_valid.fill(false);
    for (auto& points : gt_points)
    {
      points = {};
    }
    for (auto& center : gt_centers)
    {
      center = cv::Point2f();
    }

    for (int face_index = 0; face_index < 4; ++face_index)
    {
      const auto& face = truth_faces[face_index];
      if (!face.valid)
      {
        continue;
      }
      gt_points[face_index] = face.points;
      gt_centers[face_index] = face.center;
      gt_valid[face_index] = true;
    }
    return true;
  }

  bool AssignTruthToDetectionsByCorners(
      const ArmorDetectionsMessage& armors,
      const std::array<std::array<cv::Point2f, 4>, 4>& gt_points,
      const std::array<cv::Point2f, 4>& gt_centers,
      const std::array<bool, 4>& gt_valid,
      std::vector<DetectorCornerTruthAssignment>& assignments) const
  {
    assignments.assign(armors.results.size(), DetectorCornerTruthAssignment{});
    if (armors.results.empty())
    {
      return true;
    }

    std::vector<int> candidates;
    for (int i = 0; i < 4; ++i)
    {
      if (gt_valid[i])
      {
        candidates.push_back(i);
      }
    }
    if (candidates.size() < armors.results.size())
    {
      return false;
    }

    const std::size_t det_count = armors.results.size();
    struct MatchMetrics
    {
      bool valid{false};
      double assign_cost{std::numeric_limits<double>::infinity()};
      double point_mean_px{0.0};
      double point_rms_px{0.0};
      double point_max_px{0.0};
      double center_dx_px{0.0};
      double center_dy_px{0.0};
      double center_err_px{0.0};
      double shape_mean_px{0.0};
      double shape_rms_px{0.0};
    };
    auto evaluate_match = [&](const ArmorDetectorResult& armor, int gt_index) -> MatchMetrics
    {
      MatchMetrics best;
      auto try_perm = [&](bool reverse, int shift)
      {
        double point_sum = 0.0;
        double point_sq_sum = 0.0;
        double point_max = 0.0;
        double shape_sum = 0.0;
        double shape_sq_sum = 0.0;
        for (std::size_t k = 0; k < armor.points.size(); ++k)
        {
          const std::size_t gt_k =
              reverse ? ((shift + 4 - static_cast<int>(k)) % 4)
                      : ((static_cast<int>(k) + shift) % 4);
          const double dist = cv::norm(armor.points[k] - gt_points[gt_index][gt_k]);
          point_sum += dist;
          point_sq_sum += dist * dist;
          point_max = std::max(point_max, dist);

          const cv::Point2f det_rel = armor.points[k] - armor.center;
          const cv::Point2f gt_rel = gt_points[gt_index][gt_k] - gt_centers[gt_index];
          const double shape_dist = cv::norm(det_rel - gt_rel);
          shape_sum += shape_dist;
          shape_sq_sum += shape_dist * shape_dist;
        }
        const double point_rms = std::sqrt(point_sq_sum / 4.0);
        const double shape_rms = std::sqrt(shape_sq_sum / 4.0);
        const double center_dx = static_cast<double>(armor.center.x - gt_centers[gt_index].x);
        const double center_dy = static_cast<double>(armor.center.y - gt_centers[gt_index].y);
        const double center_err = cv::norm(armor.center - gt_centers[gt_index]);
        const double assign_cost = point_rms + 0.25 * center_err;
        if (assign_cost < best.assign_cost)
        {
          best.valid = true;
          best.assign_cost = assign_cost;
          best.point_mean_px = point_sum / 4.0;
          best.point_rms_px = point_rms;
          best.point_max_px = point_max;
          best.center_dx_px = center_dx;
          best.center_dy_px = center_dy;
          best.center_err_px = center_err;
          best.shape_mean_px = shape_sum / 4.0;
          best.shape_rms_px = shape_rms;
        }
      };
      for (int shift = 0; shift < 4; ++shift)
      {
        try_perm(false, shift);
        try_perm(true, shift);
      }
      return best;
    };

    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_assign(det_count, -1);
    std::vector<int> current_assign(det_count, -1);
    std::vector<bool> used(candidates.size(), false);
    auto dfs = [&](auto&& self, std::size_t det_idx, double accum_cost) -> void
    {
      if (det_idx >= det_count)
      {
        if (accum_cost < best_cost)
        {
          best_cost = accum_cost;
          best_assign = current_assign;
        }
        return;
      }
      if (accum_cost >= best_cost)
      {
        return;
      }
      const auto& armor = armors.results[det_idx];
      for (std::size_t c = 0; c < candidates.size(); ++c)
      {
        if (used[c])
        {
          continue;
        }
        const int gt_index = candidates[c];
        const MatchMetrics metrics = evaluate_match(armor, gt_index);
        if (!metrics.valid)
        {
          continue;
        }
        used[c] = true;
        current_assign[det_idx] = gt_index;
        self(self, det_idx + 1, accum_cost + metrics.assign_cost);
        current_assign[det_idx] = -1;
        used[c] = false;
      }
    };
    dfs(dfs, 0, 0.0);
    if (!std::isfinite(best_cost))
    {
      return false;
    }

    for (std::size_t i = 0; i < det_count; ++i)
    {
      if (best_assign[i] < 0)
      {
        continue;
      }
      const auto& armor = armors.results[i];
      const MatchMetrics metrics = evaluate_match(armor, best_assign[i]);
      if (!metrics.valid)
      {
        continue;
      }
      auto& item = assignments[i];
      item.valid = true;
      item.gt_index = best_assign[i];
      item.point_mean_px = metrics.point_mean_px;
      item.point_rms_px = metrics.point_rms_px;
      item.point_max_px = metrics.point_max_px;
      item.center_dx_px = metrics.center_dx_px;
      item.center_dy_px = metrics.center_dy_px;
      item.center_err_px = metrics.center_err_px;
      item.shape_mean_px = metrics.shape_mean_px;
      item.shape_rms_px = metrics.shape_rms_px;
    }
    return true;
  }

  void RunDetectorCornerAudit(const ArmorDetectionsMessage& armors,
                              const std::shared_ptr<CameraBase::CameraInfo>& cam_info)
  {
    if (!DetectorCornerAuditEnabled() || cam_info == nullptr || supervisor_ == nullptr)
    {
      return;
    }

    std::array<std::array<cv::Point2f, 4>, 4> gt_points{};
    std::array<cv::Point2f, 4> gt_centers{};
    std::array<bool, 4> gt_valid{};
    if (!ReadTruthProjectedCorners(*cam_info, gt_points, gt_centers, gt_valid))
    {
      return;
    }

    std::vector<DetectorCornerTruthAssignment> assignments;
    if (!AssignTruthToDetectionsByCorners(armors, gt_points, gt_centers, gt_valid,
                                          assignments))
    {
      return;
    }

    std::lock_guard<std::mutex> lock(detector_corner_audit_lock_);
    detector_corner_audit_stats_.frame_total++;
    if (!detector_corner_audit_file_.is_open())
    {
      detector_corner_audit_file_.open(detector_corner_audit_path_,
                                       std::ios::out | std::ios::trunc);
      if (detector_corner_audit_file_)
      {
        detector_corner_audit_file_
            << "video_frame	image_ts_us	sim_time_s	detection_index	gt_label	"
               "point_mean_px	point_rms_px	point_max_px	center_dx_px	"
               "center_dy_px	center_err_px	shape_mean_px	shape_rms_px\n";
      }
    }

    const double sim_time_s = supervisor_->getTime();
    for (std::size_t i = 0; i < assignments.size(); ++i)
    {
      const auto& item = assignments[i];
      if (!item.valid)
      {
        continue;
      }
      detector_corner_audit_stats_.row_count++;
      detector_corner_audit_stats_.point_mean_values.push_back(item.point_mean_px);
      detector_corner_audit_stats_.point_rms_values.push_back(item.point_rms_px);
      detector_corner_audit_stats_.point_max_values.push_back(item.point_max_px);
      detector_corner_audit_stats_.center_err_values.push_back(item.center_err_px);
      detector_corner_audit_stats_.shape_mean_values.push_back(item.shape_mean_px);
      detector_corner_audit_stats_.shape_rms_values.push_back(item.shape_rms_px);
      if (detector_corner_audit_file_)
      {
        detector_corner_audit_file_ << frame_count_ << '	' << armors.image_timestamp_us
                                    << '	' << std::fixed << std::setprecision(6)
                                    << sim_time_s << '	' << i << '	'
                                    << TruthLabelString(item.gt_index) << '	'
                                    << item.point_mean_px << '	' << item.point_rms_px
                                    << '\t' << item.point_max_px << '\t'
                                    << item.center_dx_px << '\t'
                                    << item.center_dy_px << '\t'
                                    << item.center_err_px << '\t'
                                    << item.shape_mean_px << '\t'
                                    << item.shape_rms_px << '\n';
      }
    }

    if (detector_corner_audit_file_)
    {
      detector_corner_audit_file_.flush();
    }
  }

  void WriteDetectorCornerAuditSummary()
  {
    if (!DetectorCornerAuditEnabled())
    {
      return;
    }

    std::lock_guard<std::mutex> lock(detector_corner_audit_lock_);
    if (detector_corner_audit_file_.is_open())
    {
      detector_corner_audit_file_.flush();
    }

    std::ofstream summary(detector_corner_audit_summary_path_,
                          std::ios::out | std::ios::trunc);
    summary << "path=" << detector_corner_audit_path_ << '\n';
    summary << "frame_total=" << detector_corner_audit_stats_.frame_total << '\n';
    summary << "row_count=" << detector_corner_audit_stats_.row_count << '\n';
    summary << std::fixed << std::setprecision(6);
    summary << "point_mean_px_mean="
            << MeanOf(detector_corner_audit_stats_.point_mean_values) << '\n';
    summary << "point_mean_px_p50="
            << Percentile(detector_corner_audit_stats_.point_mean_values, 0.50) << '\n';
    summary << "point_mean_px_p95="
            << Percentile(detector_corner_audit_stats_.point_mean_values, 0.95) << '\n';
    summary << "point_mean_px_max="
            << (detector_corner_audit_stats_.point_mean_values.empty()
                    ? 0.0
                    : *std::max_element(detector_corner_audit_stats_.point_mean_values.begin(),
                                        detector_corner_audit_stats_.point_mean_values.end()))
            << '\n';
    summary << "point_rms_px_mean="
            << MeanOf(detector_corner_audit_stats_.point_rms_values) << '\n';
    summary << "point_rms_px_p50="
            << Percentile(detector_corner_audit_stats_.point_rms_values, 0.50) << '\n';
    summary << "point_rms_px_p95="
            << Percentile(detector_corner_audit_stats_.point_rms_values, 0.95) << '\n';
    summary << "point_rms_px_max="
            << (detector_corner_audit_stats_.point_rms_values.empty()
                    ? 0.0
                    : *std::max_element(detector_corner_audit_stats_.point_rms_values.begin(),
                                        detector_corner_audit_stats_.point_rms_values.end()))
            << '\n';
    summary << "point_max_px_mean="
            << MeanOf(detector_corner_audit_stats_.point_max_values) << '\n';
    summary << "point_max_px_p50="
            << Percentile(detector_corner_audit_stats_.point_max_values, 0.50) << '\n';
    summary << "point_max_px_p95="
            << Percentile(detector_corner_audit_stats_.point_max_values, 0.95) << '\n';
    summary << "point_max_px_max="
            << (detector_corner_audit_stats_.point_max_values.empty()
                    ? 0.0
                    : *std::max_element(detector_corner_audit_stats_.point_max_values.begin(),
                                        detector_corner_audit_stats_.point_max_values.end()))
            << '\n';
    summary << "center_err_px_mean="
            << MeanOf(detector_corner_audit_stats_.center_err_values) << '\n';
    summary << "center_err_px_p50="
            << Percentile(detector_corner_audit_stats_.center_err_values, 0.50) << '\n';
    summary << "center_err_px_p95="
            << Percentile(detector_corner_audit_stats_.center_err_values, 0.95) << '\n';
    summary << "center_err_px_max="
            << (detector_corner_audit_stats_.center_err_values.empty()
                    ? 0.0
                    : *std::max_element(detector_corner_audit_stats_.center_err_values.begin(),
                                        detector_corner_audit_stats_.center_err_values.end()))
            << '\n';
    summary << "shape_mean_px_mean="
            << MeanOf(detector_corner_audit_stats_.shape_mean_values) << '\n';
    summary << "shape_mean_px_p50="
            << Percentile(detector_corner_audit_stats_.shape_mean_values, 0.50) << '\n';
    summary << "shape_mean_px_p95="
            << Percentile(detector_corner_audit_stats_.shape_mean_values, 0.95) << '\n';
    summary << "shape_mean_px_max="
            << (detector_corner_audit_stats_.shape_mean_values.empty()
                    ? 0.0
                    : *std::max_element(detector_corner_audit_stats_.shape_mean_values.begin(),
                                        detector_corner_audit_stats_.shape_mean_values.end()))
            << '\n';
    summary << "shape_rms_px_mean="
            << MeanOf(detector_corner_audit_stats_.shape_rms_values) << '\n';
    summary << "shape_rms_px_p50="
            << Percentile(detector_corner_audit_stats_.shape_rms_values, 0.50) << '\n';
    summary << "shape_rms_px_p95="
            << Percentile(detector_corner_audit_stats_.shape_rms_values, 0.95) << '\n';
    summary << "shape_rms_px_max="
            << (detector_corner_audit_stats_.shape_rms_values.empty()
                    ? 0.0
                    : *std::max_element(detector_corner_audit_stats_.shape_rms_values.begin(),
                                        detector_corner_audit_stats_.shape_rms_values.end()))
            << '\n';
  }

  bool AssignTruthToDetections(const ArmorDetectionsMessage& armors,
                               const std::array<Eigen::Vector3d, 4>& gt_cam,
                               const std::array<bool, 4>& gt_visible,
                               uint64_t image_timestamp_us,
                               std::vector<DetectionTruthAssignment>& detection_truth) const
  {
    constexpr double kTruthContinuityWeight = 2.0;
    constexpr double kTruthContinuityBaseAllowanceM = 0.18;
    constexpr double kTruthContinuityVelocityAllowanceMps = 1.5;
    constexpr uint64_t kTruthContinuityTimeoutUs = 200000;

    detection_truth.assign(armors.results.size(), DetectionTruthAssignment{});
    if (armors.results.empty())
    {
      return true;
    }
    std::vector<int> candidates;
    for (int i = 0; i < 4; ++i)
    {
      if (gt_visible[i])
      {
        candidates.push_back(i);
      }
    }
    if (candidates.size() < armors.results.size())
    {
      candidates = {0, 1, 2, 3};
    }
    const std::size_t det_count = armors.results.size();
    if (candidates.size() < det_count)
    {
      return false;
    }
    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_assign(det_count, -1);
    std::vector<int> current_assign(det_count, -1);
    std::vector<bool> used(candidates.size(), false);
    auto dfs = [&](auto&& self, std::size_t det_idx, double accum_cost) -> void
    {
      if (det_idx >= det_count)
      {
        if (accum_cost < best_cost)
        {
          best_cost = accum_cost;
          best_assign = current_assign;
        }
        return;
      }
      if (accum_cost >= best_cost)
      {
        return;
      }
      const auto& armor = armors.results[det_idx];
      const Eigen::Vector3d det_pos(armor.pose.translation.x(), armor.pose.translation.y(),
                                    armor.pose.translation.z());
      for (std::size_t c = 0; c < candidates.size(); ++c)
      {
        if (used[c])
        {
          continue;
        }
        const int gt_index = candidates[c];
        const double err = (det_pos - gt_cam[gt_index]).norm();
        double continuity_err = 0.0;
        const auto& persist = truth_gt_persistent_[gt_index];
        if (persist.valid && image_timestamp_us > persist.last_ts_us)
        {
          const uint64_t dt_us = image_timestamp_us - persist.last_ts_us;
          if (dt_us <= kTruthContinuityTimeoutUs)
          {
            const double dt_s = static_cast<double>(dt_us) * 1e-6;
            const double allowance =
                kTruthContinuityBaseAllowanceM +
                kTruthContinuityVelocityAllowanceMps * dt_s;
            continuity_err =
                std::max(0.0, (det_pos - persist.last_det_pos).norm() - allowance);
          }
        }
        const double total_cost = err + kTruthContinuityWeight * continuity_err;
        used[c] = true;
        current_assign[det_idx] = gt_index;
        self(self, det_idx + 1, accum_cost + total_cost);
        current_assign[det_idx] = -1;
        used[c] = false;
      }
    };
    dfs(dfs, 0, 0.0);
    if (!std::isfinite(best_cost))
    {
      return false;
    }
    for (std::size_t i = 0; i < det_count; ++i)
    {
      if (best_assign[i] < 0)
      {
        continue;
      }
      const auto& armor = armors.results[i];
      const Eigen::Vector3d det_pos(armor.pose.translation.x(), armor.pose.translation.y(),
                                    armor.pose.translation.z());
      detection_truth[i].valid = true;
      detection_truth[i].gt_index = best_assign[i];
      detection_truth[i].error_m = (det_pos - gt_cam[best_assign[i]]).norm();
      const auto& persist = truth_gt_persistent_[best_assign[i]];
      if (persist.valid && image_timestamp_us > persist.last_ts_us)
      {
        const uint64_t dt_us = image_timestamp_us - persist.last_ts_us;
        if (dt_us <= kTruthContinuityTimeoutUs)
        {
          const double dt_s = static_cast<double>(dt_us) * 1e-6;
          const double allowance =
              kTruthContinuityBaseAllowanceM +
              kTruthContinuityVelocityAllowanceMps * dt_s;
          detection_truth[i].continuity_error_m =
              std::max(0.0, (det_pos - persist.last_det_pos).norm() - allowance);
        }
      }
      detection_truth[i].total_cost =
          detection_truth[i].error_m +
          kTruthContinuityWeight * detection_truth[i].continuity_error_m;
    }
    return true;
  }

  bool AnalyzeIndependentTruth(
      const ArmorDetectionsMessage& armors,
      const IndependentTracksSnapshot* independent_tracks,
      const std::shared_ptr<CameraBase::CameraInfo>& cam_info,
      std::vector<DetectionTruthAssignment>& detection_truth,
      std::array<TrackTruthSnapshot, kMaxIndependentTracks>& track_truth)
  {
    detection_truth.clear();
    for (auto& item : track_truth)
    {
      item = TrackTruthSnapshot{};
    }
    if (independent_tracks == nullptr || supervisor_ == nullptr || cam_info == nullptr)
    {
      return false;
    }
    std::array<Eigen::Vector3d, 4> gt_cam{};
    std::array<bool, 4> gt_visible{};
    if (!ReadTruthCamera(*cam_info, gt_cam, gt_visible) ||
        !AssignTruthToDetections(armors, gt_cam, gt_visible,
                                 independent_tracks->image_timestamp_us, detection_truth))
    {
      return false;
    }
    if (IndependentTrackAuditEnabled())
    {
      std::ostringstream truth_line;
      truth_line << "IndependentTruth labels: ts=" << independent_tracks->image_timestamp_us;
      if (detection_truth.empty())
      {
        truth_line << " none";
      }
      for (std::size_t i = 0; i < detection_truth.size(); ++i)
      {
        truth_line << " det" << i << '=';
        if (detection_truth[i].valid)
        {
          truth_line << TruthLabelString(detection_truth[i].gt_index) << '('
                     << std::fixed << std::setprecision(3)
                     << detection_truth[i].error_m << ')';
        }
        else
        {
          truth_line << "na";
        }
      }
      XR_LOG_WARN("%s", truth_line.str().c_str());

      for (std::size_t i = 0; i < armors.results.size(); ++i)
      {
        const auto& armor = armors.results[i];
        const Eigen::Vector3d det_pos(armor.pose.translation.x(), armor.pose.translation.y(),
                                      armor.pose.translation.z());
        int nearest_gt = -1;
        double nearest_err = std::numeric_limits<double>::infinity();
        std::ostringstream detail_line;
        detail_line << "IndependentTruth audit det: ts=" << independent_tracks->image_timestamp_us
                    << " det=" << i << " xyz=(" << std::fixed << std::setprecision(3)
                    << det_pos.x() << ", " << det_pos.y() << ", " << det_pos.z() << ")";
        for (int gt_index = 0; gt_index < 4; ++gt_index)
        {
          const double err = (det_pos - gt_cam[gt_index]).norm();
          if (err < nearest_err)
          {
            nearest_err = err;
            nearest_gt = gt_index;
          }
          detail_line << ' ' << TruthLabelString(gt_index) << '=' << std::fixed
                      << std::setprecision(3) << err;
          if (gt_visible[gt_index])
          {
            detail_line << 'v';
          }
        }
        detail_line << " nearest=" << TruthLabelString(nearest_gt) << '(' << std::fixed
                    << std::setprecision(3) << nearest_err << ")";
        if (i < detection_truth.size() && detection_truth[i].valid)
        {
          detail_line << " final=" << TruthLabelString(detection_truth[i].gt_index) << '('
                      << std::fixed << std::setprecision(3)
                      << detection_truth[i].error_m << ')'
                      << " cont=" << std::fixed << std::setprecision(3)
                      << detection_truth[i].continuity_error_m << " cost="
                      << detection_truth[i].total_cost;
        }
        else
        {
          detail_line << " final=na";
        }
        XR_LOG_WARN("%s", detail_line.str().c_str());
      }
    }
    std::lock_guard<std::mutex> lock(truth_lock_);
    truth_stats_.frame_total++;
    truth_stats_.detection_assignment_total += static_cast<uint32_t>(
        std::count_if(detection_truth.begin(), detection_truth.end(),
                      [](const DetectionTruthAssignment& item) { return item.valid; }));
    if (!truth_file_.is_open())
    {
      truth_file_.open(truth_path_, std::ios::out | std::ios::trunc);
      if (truth_file_)
      {
        truth_file_ << "video_frame\timage_ts_us\tsim_time_s\ttrack_id\tdetection_index\tgt_label\terr_m\tswitch_count\tswitched_this_frame\n";
      }
    }
    const double sim_time_s = supervisor_->getTime();
    bool frame_has_switch = false;
    for (std::size_t i = 0; i < independent_tracks->tracks.size(); ++i)
    {
      const auto& track = independent_tracks->tracks[i];
      if (!TrackShouldDisplay(track) || !track.matched_this_frame)
      {
        continue;
      }
      if (track.matched_armor_index == kInvalidArmorIndex ||
          track.matched_armor_index >= detection_truth.size())
      {
        continue;
      }
      const auto& assignment = detection_truth[track.matched_armor_index];
      if (!assignment.valid)
      {
        continue;
      }
      auto& persist = truth_persistent_[TrackDisplayId(track)];
      const bool switched =
          persist.last_gt_index >= 0 && persist.last_gt_index != assignment.gt_index;
      if (switched)
      {
        persist.switch_count++;
        truth_stats_.switch_total++;
        frame_has_switch = true;
        if (persist.switch_count == 1)
        {
          truth_stats_.switched_track_total++;
        }
        XR_LOG_WARN(
            "IndependentTrack truth switch: tid=%u det=%u %s->%s err=%.3f frame=%u ts=%llu",
            static_cast<unsigned>(track.track_id),
            static_cast<unsigned>(track.matched_armor_index),
            TruthLabelString(persist.last_gt_index), TruthLabelString(assignment.gt_index),
            assignment.error_m, frame_count_,
            static_cast<unsigned long long>(independent_tracks->image_timestamp_us));
      }
      persist.last_gt_index = assignment.gt_index;
      track_truth[i].valid = true;
      track_truth[i].gt_index = assignment.gt_index;
      track_truth[i].error_m = assignment.error_m;
      track_truth[i].switch_count = persist.switch_count;
      track_truth[i].switched_this_frame = switched;
      truth_stats_.track_assignment_total++;
      truth_stats_.track_error_sum += assignment.error_m;
      truth_stats_.track_error_count++;
      if (truth_file_)
      {
        truth_file_ << frame_count_ << '\t' << independent_tracks->image_timestamp_us
                    << '\t' << std::fixed << std::setprecision(6) << sim_time_s << '\t'
                    << TrackDisplayId(track) << '\t'
                    << static_cast<int>(track.matched_armor_index) << '\t'
                    << TruthLabelString(assignment.gt_index) << '\t' << assignment.error_m
                    << '\t' << persist.switch_count << '\t' << (switched ? 1 : 0)
                    << '\n';
      }
    }
    for (std::size_t i = 0; i < armors.results.size(); ++i)
    {
      if (i >= detection_truth.size() || !detection_truth[i].valid)
      {
        continue;
      }
      const auto& armor = armors.results[i];
      auto& persist = truth_gt_persistent_[detection_truth[i].gt_index];
      persist.valid = true;
      persist.last_ts_us = independent_tracks->image_timestamp_us;
      persist.last_det_pos = Eigen::Vector3d(armor.pose.translation.x(),
                                             armor.pose.translation.y(),
                                             armor.pose.translation.z());
    }
    if (frame_has_switch)
    {
      truth_stats_.switch_frame_total++;
    }
    if (truth_file_)
    {
      truth_file_.flush();
    }
    return true;
  }

  bool ProjectPoint(const CameraBase::CameraInfo& cam, const cv::Size& frame_size,
                    const Eigen::Vector3d& pc, cv::Point2d& uv) const
  {
    if (!(pc.z() > 1e-6) || !std::isfinite(pc.x()) || !std::isfinite(pc.y()) ||
        !std::isfinite(pc.z()))
    {
      return false;
    }

    cv::Mat k = (cv::Mat_<double>(3, 3) << cam.camera_matrix[0], cam.camera_matrix[1],
                 cam.camera_matrix[2], cam.camera_matrix[3], cam.camera_matrix[4],
                 cam.camera_matrix[5], cam.camera_matrix[6], cam.camera_matrix[7],
                 cam.camera_matrix[8]);
    const double sx = static_cast<double>(frame_size.width) /
                      static_cast<double>(std::max<uint32_t>(1, cam.width));
    const double sy = static_cast<double>(frame_size.height) /
                      static_cast<double>(std::max<uint32_t>(1, cam.height));
    k.at<double>(0, 0) *= sx;
    k.at<double>(1, 1) *= sy;
    k.at<double>(0, 2) *= sx;
    k.at<double>(1, 2) *= sy;

    cv::Mat d;
    if (cam.distortion_model == CameraBase::DistortionModel::PLUMB_BOB)
    {
      std::vector<double> dvec = {cam.distortion_coefficients[0],
                                  cam.distortion_coefficients[1],
                                  cam.distortion_coefficients[2],
                                  cam.distortion_coefficients[3],
                                  cam.distortion_coefficients[4]};
      d = cv::Mat(dvec).clone().reshape(1, 1);
    }

    std::vector<cv::Point3d> obj{cv::Point3d(pc.x(), pc.y(), pc.z())};
    cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64F);
    cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64F);
    std::vector<cv::Point2d> imgpts;
    cv::projectPoints(obj, rvec, tvec, k, d, imgpts);
    uv = imgpts[0];
    return uv.x >= 0 && uv.x < frame_size.width && uv.y >= 0 && uv.y < frame_size.height;
  }

  static cv::Scalar TruthQuadColor(int gt_index)
  {
    switch (gt_index)
    {
      case 0:
        return cv::Scalar(80, 240, 240);
      case 1:
        return cv::Scalar(110, 255, 110);
      case 2:
        return cv::Scalar(0, 190, 255);
      case 3:
        return cv::Scalar(240, 120, 240);
      default:
        return cv::Scalar(230, 230, 230);
    }
  }


  bool AnalyzeEkfTruth(const ArmorTracker::EkfPointsMsg& ekf_points,
                       uint64_t image_timestamp_us,
                       const std::shared_ptr<CameraBase::CameraInfo>& cam_info)
  {
    if (supervisor_ == nullptr || cam_info == nullptr)
    {
      return false;
    }

    std::array<Eigen::Vector3d, 4> gt_cam{};
    std::array<bool, 4> gt_visible{};
    if (!ReadTruthCamera(*cam_info, gt_cam, gt_visible))
    {
      return false;
    }

    std::vector<int> pred_indices;
    pred_indices.reserve(std::min<int>(ekf_points.count, 4));
    for (int i = 0; i < std::min<int>(ekf_points.count, 4); ++i)
    {
      if (!ekf_points.valid[i + 1])
      {
        continue;
      }
      const auto& p = ekf_points.armors_cam[i];
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
      {
        continue;
      }
      pred_indices.push_back(i);
    }
    if (pred_indices.empty())
    {
      return false;
    }

    double best_cost = std::numeric_limits<double>::infinity();
    std::vector<int> best_assign(pred_indices.size(), -1);
    std::vector<int> current_assign(pred_indices.size(), -1);
    std::array<bool, 4> used{};
    auto dfs = [&](auto&& self, std::size_t pred_idx, double accum_cost) -> void
    {
      if (pred_idx >= pred_indices.size())
      {
        if (accum_cost < best_cost)
        {
          best_cost = accum_cost;
          best_assign = current_assign;
        }
        return;
      }
      if (accum_cost >= best_cost)
      {
        return;
      }
      const int armor_slot = pred_indices[pred_idx];
      const auto& armor_pos = ekf_points.armors_cam[armor_slot];
      const Eigen::Vector3d pred_pos(armor_pos.x(), armor_pos.y(), armor_pos.z());
      for (int gt_index = 0; gt_index < 4; ++gt_index)
      {
        if (used[gt_index])
        {
          continue;
        }
        const double err = (pred_pos - gt_cam[gt_index]).norm();
        used[gt_index] = true;
        current_assign[pred_idx] = gt_index;
        self(self, pred_idx + 1, accum_cost + err);
        current_assign[pred_idx] = -1;
        used[gt_index] = false;
      }
    };
    dfs(dfs, 0, 0.0);
    if (!std::isfinite(best_cost))
    {
      return false;
    }

    std::vector<EkfTruthAssignment> assignments(pred_indices.size());
    double frame_err_sum = 0.0;
    double frame_err_max = 0.0;
    for (std::size_t i = 0; i < pred_indices.size(); ++i)
    {
      const int gt_index = best_assign[i];
      if (gt_index < 0 || gt_index >= 4)
      {
        continue;
      }
      const int armor_slot = pred_indices[i];
      const auto& armor_pos = ekf_points.armors_cam[armor_slot];
      const Eigen::Vector3d pred_pos(armor_pos.x(), armor_pos.y(), armor_pos.z());
      assignments[i].valid = true;
      assignments[i].gt_index = gt_index;
      assignments[i].error_m = (pred_pos - gt_cam[gt_index]).norm();
      frame_err_sum += assignments[i].error_m;
      frame_err_max = std::max(frame_err_max, assignments[i].error_m);
    }

    std::lock_guard<std::mutex> lock(truth_lock_);
    ekf_truth_stats_.frame_total++;
    if (!ekf_truth_file_.is_open())
    {
      ekf_truth_file_.open(ekf_truth_path_, std::ios::out | std::ios::trunc);
      if (ekf_truth_file_)
      {
        ekf_truth_file_ << "video_frame\timage_ts_us\tsim_time_s\tpred_index\tgt_label\t"
                           "err_m\tpred_x\tpred_y\tpred_z\tgt_x\tgt_y\tgt_z\n";
      }
    }

    const double sim_time_s = supervisor_->getTime();
    for (std::size_t i = 0; i < pred_indices.size(); ++i)
    {
      if (!assignments[i].valid)
      {
        continue;
      }
      ekf_truth_stats_.row_count++;
      ekf_truth_stats_.armor_err_values.push_back(assignments[i].error_m);
      if (ekf_truth_file_)
      {
        const int armor_slot = pred_indices[i];
        const auto& armor_pos = ekf_points.armors_cam[armor_slot];
        const Eigen::Vector3d pred_pos(armor_pos.x(), armor_pos.y(), armor_pos.z());
        const Eigen::Vector3d& gt_pos = gt_cam[assignments[i].gt_index];
        ekf_truth_file_ << frame_count_ << '\t' << image_timestamp_us << '\t'
                        << std::fixed << std::setprecision(6) << sim_time_s << '\t'
                        << pred_indices[i] << '\t'
                        << TruthLabelString(assignments[i].gt_index) << '\t'
                        << assignments[i].error_m << '\t' << pred_pos.x() << '\t'
                        << pred_pos.y() << '\t' << pred_pos.z() << '\t'
                        << gt_pos.x() << '\t' << gt_pos.y() << '\t'
                        << gt_pos.z() << '\n';
      }
    }
    ekf_truth_stats_.frame_mean_err_values.push_back(
        frame_err_sum / static_cast<double>(pred_indices.size()));
    ekf_truth_stats_.frame_max_err_values.push_back(frame_err_max);
    if (ekf_truth_file_)
    {
      ekf_truth_file_.flush();
    }
    return true;
  }

  void WriteEkfTruthSummary()
  {
    std::lock_guard<std::mutex> lock(truth_lock_);
    if (ekf_truth_file_.is_open())
    {
      ekf_truth_file_.flush();
    }

    std::ofstream summary(ekf_truth_summary_path_, std::ios::out | std::ios::trunc);
    summary << "path=" << ekf_truth_path_ << '\n';
    summary << "frame_total=" << ekf_truth_stats_.frame_total << '\n';
    summary << "row_count=" << ekf_truth_stats_.row_count << '\n';
    summary << std::fixed << std::setprecision(6);
    summary << "armor_err_m_mean=" << MeanOf(ekf_truth_stats_.armor_err_values) << '\n';
    summary << "armor_err_m_p50=" << Percentile(ekf_truth_stats_.armor_err_values, 0.50)
            << '\n';
    summary << "armor_err_m_p95=" << Percentile(ekf_truth_stats_.armor_err_values, 0.95)
            << '\n';
    summary << "armor_err_m_max="
            << (ekf_truth_stats_.armor_err_values.empty()
                    ? 0.0
                    : *std::max_element(ekf_truth_stats_.armor_err_values.begin(),
                                        ekf_truth_stats_.armor_err_values.end()))
            << '\n';
    summary << "frame_mean_err_m_mean="
            << MeanOf(ekf_truth_stats_.frame_mean_err_values) << '\n';
    summary << "frame_max_err_m_mean="
            << MeanOf(ekf_truth_stats_.frame_max_err_values) << '\n';
    summary << "frame_max_err_m_p95="
            << Percentile(ekf_truth_stats_.frame_max_err_values, 0.95) << '\n';
    summary << "frame_max_err_m_max="
            << (ekf_truth_stats_.frame_max_err_values.empty()
                    ? 0.0
                    : *std::max_element(ekf_truth_stats_.frame_max_err_values.begin(),
                                        ekf_truth_stats_.frame_max_err_values.end()))
            << '\n';
  }

  void DrawMatchedTruthOverlay(cv::Mat& frame, const ArmorDetectionsMessage& armors,
                              const std::shared_ptr<CameraBase::CameraInfo>& cam_info)
  {
    if (!TruthQuadOverlayEnabled() || !cam_info)
    {
      return;
    }

    std::array<std::array<cv::Point2f, 4>, 4> gt_points{};
    std::array<cv::Point2f, 4> gt_centers{};
    std::array<bool, 4> gt_valid{};
    if (!ReadTruthProjectedCorners(*cam_info, gt_points, gt_centers, gt_valid))
    {
      return;
    }

    std::vector<DetectorCornerTruthAssignment> assignments;
    if (!AssignTruthToDetectionsByCorners(armors, gt_points, gt_centers, gt_valid,
                                          assignments))
    {
      return;
    }

    for (std::size_t armor_index = 0; armor_index < assignments.size(); ++armor_index)
    {
      const auto& match = assignments[armor_index];
      if (!match.valid)
      {
        continue;
      }

      const auto& armor = armors.results[armor_index];
      const auto& gt = gt_points[match.gt_index];
      const cv::Scalar color = TruthQuadColor(match.gt_index);

      std::array<cv::Point, 4> polygon{};
      for (std::size_t i = 0; i < polygon.size(); ++i)
      {
        polygon[i] = gt[i];
      }
      const cv::Point* points = polygon.data();
      const int count = static_cast<int>(polygon.size());
      cv::polylines(frame, &points, &count, 1, true, color, 2, cv::LINE_AA);

      for (std::size_t k = 0; k < armor.points.size(); ++k)
      {
        cv::line(frame, armor.points[k], gt[k], cv::Scalar(255, 255, 255), 1,
                 cv::LINE_AA);
        cv::circle(frame, gt[k], 3, color, cv::FILLED, cv::LINE_AA);
        const std::string idx = std::to_string(k);
        cv::putText(frame, idx, gt[k] + cv::Point2f(4.0f, -4.0f),
                    cv::FONT_HERSHEY_DUPLEX, 0.42, color, 1, cv::LINE_AA);
      }

      cv::drawMarker(frame, gt_centers[match.gt_index], color, cv::MARKER_TILTED_CROSS,
                     12, 1, cv::LINE_AA);
      std::ostringstream label;
      label << "GT " << TruthLabelString(match.gt_index)
            << " c=" << std::fixed << std::setprecision(0) << match.center_err_px
            << " s=" << std::setprecision(0) << match.shape_rms_px;
      DrawLabel(frame, armor.center + cv::Point2f(12.0f, 20.0f), label.str(), color);
    }
  }

  void DrawDetectorArmors(
      cv::Mat& frame, const ArmorDetectionsMessage& armors,
      const ArmorTracker::CandidateDebugMsg* candidate_debug,
      const IndependentTracksSnapshot* independent_tracks,
      const std::vector<DetectionTruthAssignment>* detection_truth) const
  {
    const auto* selected_candidate = SelectedCandidate(candidate_debug);

    for (std::size_t armor_index = 0; armor_index < armors.results.size(); ++armor_index)
    {
      const auto& armor = armors.results[armor_index];
      const auto* independent_track =
          FindIndependentTrackForArmor(independent_tracks, armor_index);
      cv::Scalar outline_color = independent_track != nullptr
                                    ? TrackIdColor(TrackDisplayId(*independent_track))
                                    : ColorToScalar(armor.color);
      int line_thickness = 2;
      bool is_selected_armor = false;
      if (candidate_debug != nullptr && selected_candidate != nullptr &&
          selected_candidate->armor_index == armor_index)
      {
        is_selected_armor = true;
        line_thickness = 3;
      }

      std::array<cv::Point, 4> polygon{};
      for (std::size_t i = 0; i < armor.points.size(); ++i)
      {
        polygon[i] = armor.points[i];
      }
      const cv::Point* points = polygon.data();
      const int count = static_cast<int>(polygon.size());
      cv::polylines(frame, &points, &count, 1, true, outline_color, line_thickness,
                    cv::LINE_AA);
      cv::rectangle(frame, armor.box, outline_color, is_selected_armor ? 2 : 1,
                    cv::LINE_AA);
      for (const auto& point : armor.points)
      {
        cv::circle(frame, point, 3, outline_color, cv::FILLED, cv::LINE_AA);
      }
      cv::circle(frame, armor.center, independent_track != nullptr ? 5 : 4, outline_color,
                 2, cv::LINE_AA);

      std::ostringstream label;
      label << 'D' << armor_index;
      const int16_t tracker_image_id = CandidateDetectionTrackId(candidate_debug, armor_index);
      const bool tracker_image_confirmed =
          CandidateDetectionTrackConfirmed(candidate_debug, armor_index);
      if (tracker_image_id >= 0)
      {
        label << ' ' << TrackerImageIdString(tracker_image_id, tracker_image_confirmed);
      }
      if (independent_track != nullptr)
      {
        label << " T" << TrackDisplayId(*independent_track);
      }
      if (is_selected_armor)
      {
        label << " PICK";
      }
      label << ' ' << ArmorNumberString(armor.number) << ' ' << ArmorTypeString(armor.type)
            << ' ' << std::fixed << std::setprecision(2) << armor.confidence;
      if (detection_truth != nullptr && armor_index < detection_truth->size() &&
          (*detection_truth)[armor_index].valid)
      {
        label << " GT:" << TruthLabelString((*detection_truth)[armor_index].gt_index);
      }
      DrawLabel(frame, cv::Point(std::max(armor.box.x, 4), std::max(armor.box.y - 4, 22)),
                label.str(), outline_color);
    }
  }

  void DrawEkfOverlay(cv::Mat& frame, const ArmorTracker::EkfPointsMsg& ekf,
                      const std::shared_ptr<CameraBase::CameraInfo>& cam_info) const
  {
    if (!cam_info)
    {
      return;
    }

    if (ekf.valid[0])
    {
      cv::Point2d uv;
      const Eigen::Vector3d pc(ekf.center_cam.x(), ekf.center_cam.y(),
                               ekf.center_cam.z());
      if (ProjectPoint(*cam_info, frame.size(), pc, uv))
      {
        cv::circle(frame, uv, 6, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::putText(frame, "C", uv + cv::Point2d(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
      }
    }

    for (int i = 0; i < std::min<int>(ekf.count, 4); ++i)
    {
      if (!ekf.valid[i + 1])
      {
        continue;
      }
      cv::Point2d uv;
      const Eigen::Vector3d pc(ekf.armors_cam[i].x(), ekf.armors_cam[i].y(),
                               ekf.armors_cam[i].z());
      if (ProjectPoint(*cam_info, frame.size(), pc, uv))
      {
        cv::circle(frame, uv, 5, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
        char buf[16];
        (void)std::snprintf(buf, sizeof(buf), "A%d", i);
        cv::putText(frame, buf, uv + cv::Point2d(8, -8), cv::FONT_HERSHEY_SIMPLEX, 0.50,
                    cv::Scalar(255, 255, 0), 1, cv::LINE_AA);
      }
    }

    for (int i = 0; i < std::min<int>(ekf.count, 4); ++i)
    {
      if (!ekf.valid[0] || !ekf.valid[i + 1])
      {
        continue;
      }
      cv::Point2d uc, ua;
      const Eigen::Vector3d pc_c(ekf.center_cam.x(), ekf.center_cam.y(),
                                 ekf.center_cam.z());
      const Eigen::Vector3d pc_a(ekf.armors_cam[i].x(), ekf.armors_cam[i].y(),
                                 ekf.armors_cam[i].z());
      if (ProjectPoint(*cam_info, frame.size(), pc_c, uc) &&
          ProjectPoint(*cam_info, frame.size(), pc_a, ua))
      {
        cv::line(frame, uc, ua, cv::Scalar(80, 180, 255), 1, cv::LINE_AA);
      }
    }
  }

  void DrawPanel(cv::Mat& panel, const ArmorDetectionsMessage& armors,
                 const SolveTrajectory::Target& target,
                 const ArmorTracker::TrackerInfo* info,
                 const LibXR::EulerAngle<float>* target_eulr, const uint8_t* fire_notify,
                 const ArmorTracker::Send* send,
                 const ArmorTracker::EkfPointsMsg* ekf_points,
                 const IndependentTracksSnapshot* independent_tracks,
                 const ArmorTracker::CandidateDebugMsg* candidate_debug) const
  {
    panel.setTo(cv::Scalar(18, 22, 28));
    cv::putText(panel, "Tracker Overlay", cv::Point(16, 30), cv::FONT_HERSHEY_DUPLEX,
                0.82, cv::Scalar(244, 247, 250), 1, cv::LINE_AA);
    int y = 62;

    DrawPanelRow(panel, y, "detector_armors", std::to_string(armors.results.size()),
                 cv::Scalar(240, 244, 250));
    if (independent_tracks)
    {
      DrawPanelRow(panel, y, "id_tracks",
                   std::to_string(independent_tracks->active_count),
                   cv::Scalar(240, 244, 250));
      std::ostringstream track_match;
      track_match << static_cast<int>(independent_tracks->matched_count) << '/'
                  << static_cast<int>(independent_tracks->detection_count);
      DrawPanelRow(panel, y, "track_match", track_match.str(),
                   cv::Scalar(240, 244, 250));
      std::ostringstream track_delta;
      track_delta << static_cast<int>(independent_tracks->new_track_count) << '/'
                  << static_cast<int>(independent_tracks->removed_count);
      DrawPanelRow(panel, y, "track_new_rm", track_delta.str(),
                   cv::Scalar(240, 244, 250));
    }
    else
    {
      DrawPanelRow(panel, y, "id_tracks", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "track_match", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "track_new_rm", "n/a", cv::Scalar(170, 182, 196));
    }
    DrawPanelRow(panel, y, "tracking", target.tracking ? "true" : "false",
                 target.tracking ? cv::Scalar(100, 240, 140)
                                 : cv::Scalar(255, 210, 120));
    DrawPanelRow(panel, y, "target_id", ArmorNumberString(target.id),
                 cv::Scalar(240, 244, 250));
    DrawPanelRow(panel, y, "armors_num", std::to_string(target.armors_num),
                 cv::Scalar(240, 244, 250));

    if (candidate_debug)
    {
      DrawPanelRow(
          panel, y, "bound_face_id",
          candidate_debug->tracked_face_track_id_valid
              ? TrackerImageIdString(candidate_debug->tracked_face_track_id, true)
              : std::string("-"),
          candidate_debug->tracked_face_track_id_valid ? cv::Scalar(100, 240, 140)
                                                       : cv::Scalar(170, 182, 196));

      std::ostringstream det_ids;
      for (std::size_t i = 0; i < candidate_debug->detection_count; ++i)
      {
        if (i != 0)
        {
          det_ids << ' ';
        }
        det_ids << 'D' << i << '='
                << TrackerImageIdString(candidate_debug->detection_track_ids[i],
                                        candidate_debug->detection_track_confirmed[i] != 0);
      }
      DrawPanelRow(panel, y, "det_image_id",
                   det_ids.str().empty() ? std::string("-") : det_ids.str(),
                   cv::Scalar(240, 244, 250));

      const auto* selected_candidate = SelectedCandidate(candidate_debug);
      if (selected_candidate != nullptr)
      {
        std::ostringstream ss;
        ss << 'D' << static_cast<int>(selected_candidate->armor_index) << "/F"
           << static_cast<int>(selected_candidate->face_index) << ' '
           << TrackerImageIdString(selected_candidate->image_track_id,
                                   selected_candidate->image_track_confirmed != 0)
           << (selected_candidate->same_persistent_track ? " hold" : " switch")
           << ' ' << AcceptedModeString(candidate_debug->accepted_mode);
        DrawPanelRow(panel, y, "pick_image_id", ss.str(), cv::Scalar(240, 244, 250));
      }
      else
      {
        DrawPanelRow(panel, y, "pick_image_id", "-", cv::Scalar(170, 182, 196));
      }
    }
    else
    {
      DrawPanelRow(panel, y, "bound_face_id", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "det_image_id", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "pick_image_id", "n/a", cv::Scalar(170, 182, 196));
    }

    if (fire_notify)
    {
      DrawPanelRow(panel, y, "fire_notify", std::to_string(*fire_notify),
                   *fire_notify ? cv::Scalar(255, 110, 110)
                                : cv::Scalar(170, 182, 196));
    }
    else
    {
      DrawPanelRow(panel, y, "fire_notify", "n/a", cv::Scalar(170, 182, 196));
    }

    if (target_eulr)
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << target_eulr->Pitch() << ", "
         << target_eulr->Yaw();
      DrawPanelRow(panel, y, "target_eulr", ss.str(), cv::Scalar(240, 244, 250));
    }
    else
    {
      DrawPanelRow(panel, y, "target_eulr", "n/a", cv::Scalar(170, 182, 196));
    }

    if (info)
    {
      std::ostringstream pos_diff;
      pos_diff << std::fixed << std::setprecision(3) << info->position_diff;
      DrawPanelRow(panel, y, "position_diff", pos_diff.str(), cv::Scalar(240, 244, 250));

      std::ostringstream yaw_diff;
      yaw_diff << std::fixed << std::setprecision(3) << info->yaw_diff;
      DrawPanelRow(panel, y, "yaw_diff", yaw_diff.str(), cv::Scalar(240, 244, 250));
    }
    else
    {
      DrawPanelRow(panel, y, "position_diff", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "yaw_diff", "n/a", cv::Scalar(170, 182, 196));
    }

    auto eig_vec3_string = [](const Eigen::Matrix<double, 3, 1>& v)
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << v.x() << ", " << v.y() << ", " << v.z();
      return ss.str();
    };
    auto pos3_string = [](const LibXR::Position<double>& v)
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << v.x() << ", " << v.y() << ", " << v.z();
      return ss.str();
    };

    DrawPanelRow(panel, y, "position", eig_vec3_string(target.position),
                 cv::Scalar(240, 244, 250));
    DrawPanelRow(panel, y, "velocity", eig_vec3_string(target.velocity),
                 cv::Scalar(240, 244, 250));

    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << target.yaw;
      DrawPanelRow(panel, y, "yaw", ss.str(), cv::Scalar(240, 244, 250));
    }
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << target.v_yaw;
      DrawPanelRow(panel, y, "v_yaw", ss.str(), cv::Scalar(240, 244, 250));
    }
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << target.radius_1 << ", "
         << target.radius_2;
      DrawPanelRow(panel, y, "r1_r2", ss.str(), cv::Scalar(240, 244, 250));
    }
    {
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << target.dz;
      DrawPanelRow(panel, y, "dz", ss.str(), cv::Scalar(240, 244, 250));
    }

    if (send)
    {
      DrawPanelRow(panel, y, "send_pos", pos3_string(send->position),
                   cv::Scalar(240, 244, 250));
      std::ostringstream ss;
      ss << std::fixed << std::setprecision(3) << send->pitch << ", " << send->yaw;
      DrawPanelRow(panel, y, "send_py", ss.str(), cv::Scalar(240, 244, 250));
      std::ostringstream vv;
      vv << std::fixed << std::setprecision(3) << send->v_yaw;
      DrawPanelRow(panel, y, "send_vyaw", vv.str(), cv::Scalar(240, 244, 250));
      DrawPanelRow(panel, y, "send_fire", send->is_fire ? "true" : "false",
                   send->is_fire ? cv::Scalar(255, 110, 110)
                                 : cv::Scalar(170, 182, 196));
    }
    else
    {
      DrawPanelRow(panel, y, "send_pos", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "send_py", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "send_vyaw", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "send_fire", "n/a", cv::Scalar(170, 182, 196));
    }

    if (ekf_points)
    {
      DrawPanelRow(panel, y, "ekf_count", std::to_string(ekf_points->count),
                   cv::Scalar(240, 244, 250));
      DrawPanelRow(panel, y, "ekf_center", ekf_points->valid[0] ? "valid" : "invalid",
                   ekf_points->valid[0] ? cv::Scalar(100, 240, 140)
                                        : cv::Scalar(170, 182, 196));
    }
    else
    {
      DrawPanelRow(panel, y, "ekf_count", "n/a", cv::Scalar(170, 182, 196));
      DrawPanelRow(panel, y, "ekf_center", "n/a", cv::Scalar(170, 182, 196));
    }
  }

  void DrawIndependentTracksPanel(
      cv::Mat& panel, const IndependentTracksSnapshot* independent_tracks,
      const std::array<TrackTruthSnapshot, kMaxIndependentTracks>* track_truth) const
  {
    panel.setTo(cv::Scalar(16, 19, 25));
    cv::putText(panel, "Independent Armor Tracks", cv::Point(16, 28),
                cv::FONT_HERSHEY_DUPLEX, 0.72, cv::Scalar(244, 247, 250), 1,
                cv::LINE_AA);
    int y = 52;

    if (independent_tracks == nullptr)
    {
      DrawSmallLine(panel, y, "independent_tracks: n/a", cv::Scalar(170, 182, 196));
      return;
    }

    std::ostringstream summary;
    summary << "ts=" << independent_tracks->image_timestamp_us << " det="
            << static_cast<int>(independent_tracks->detection_count) << " active="
            << static_cast<int>(independent_tracks->active_count) << " matched="
            << static_cast<int>(independent_tracks->matched_count) << " new="
            << static_cast<int>(independent_tracks->new_track_count) << " rm="
            << static_cast<int>(independent_tracks->removed_count);
    DrawSmallLine(panel, y, summary.str(), cv::Scalar(240, 244, 250));

    DrawSmallLine(panel, y,
                  "id det hit miss score yaw xyz center label gt sw",
                  cv::Scalar(255, 210, 120));

    bool has_row = false;
    for (std::size_t track_index = 0; track_index < independent_tracks->tracks.size();
         ++track_index)
    {
      const auto& track = independent_tracks->tracks[track_index];
      if (!track.active)
      {
        continue;
      }
      has_row = true;
      const cv::Scalar row_color =
          track.matched_this_frame ? TrackIdColor(TrackDisplayId(track))
                                   : cv::Scalar(170, 182, 196);
      const int top = y - 12;
      cv::rectangle(panel, cv::Rect(10, std::max(0, top), panel.cols - 20, 15),
                    track.matched_this_frame ? cv::Scalar(28, 42, 34)
                                             : cv::Scalar(20, 24, 30),
                    cv::FILLED, cv::LINE_AA);
      std::ostringstream row;
      row << 'T' << TrackDisplayId(track) << " D";
      if (track.matched_armor_index == kInvalidArmorIndex)
      {
        row << "--";
      }
      else
      {
        row << static_cast<int>(track.matched_armor_index);
      }
      row << " hit=" << track.hit_count << " miss=" << track.miss_count
          << " sc=" << std::fixed << std::setprecision(3) << track.last_match_score
          << " yaw=" << track.yaw << " xyz=(" << std::setprecision(2)
          << track.position.x() << ',' << track.position.y() << ',' << track.position.z()
          << ") c=(" << std::lround(track.image_center.x) << ','
          << std::lround(track.image_center.y) << ") "
          << ArmorNumberString(track.number) << ' ' << ArmorTypeString(track.type);
      if (track_truth != nullptr && (*track_truth)[track_index].valid)
      {
        const auto& truth = (*track_truth)[track_index];
        row << " gt=" << TruthLabelString(truth.gt_index) << " sw=" << truth.switch_count
            << " e=" << std::setprecision(3) << truth.error_m;
        if (truth.switched_this_frame)
        {
          row << " JUMP";
        }
      }
      DrawSmallLine(panel, y, row.str(), row_color);
    }

    if (!has_row)
    {
      DrawSmallLine(panel, y, "no active tracks", cv::Scalar(170, 182, 196));
    }
  }

  void TargetCallback(SolveTrajectory::Target* target)
  {
    if (target == nullptr || Done())
    {
      return;
    }

    cv::Mat image;
    {
      std::lock_guard<std::mutex> lock(image_lock_);
      if (latest_image_.empty())
      {
        return;
      }
      image = latest_image_.clone();
    }

    ArmorDetectionsMessage armors;
    ArmorTracker::TrackerInfo info{};
    LibXR::EulerAngle<float> target_eulr{};
    uint8_t fire_notify = 0;
    ArmorTracker::Send send{};
    ArmorTracker::EkfPointsMsg ekf_points{};
    ArmorTracker::CandidateDebugMsg candidate_debug{};
    IndependentTracksSnapshot independent_tracks{};
    std::array<TrackTruthSnapshot, kMaxIndependentTracks> track_truth{};
    std::vector<DetectionTruthAssignment> detection_truth;
    std::shared_ptr<CameraBase::CameraInfo> cam_info;
    bool has_info = false;
    bool has_target_eulr = false;
    bool has_fire_notify = false;
    bool has_send = false;
    bool has_ekf_points = false;
    bool has_candidate_debug = false;
    bool has_independent_tracks = false;
    bool has_track_truth = false;

    {
      std::lock_guard<std::mutex> lock(state_lock_);
      if (has_armors_)
      {
        armors = latest_armors_;
      }
      if (has_info_)
      {
        info = latest_info_;
        has_info = true;
      }
      if (has_target_eulr_)
      {
        target_eulr = latest_target_eulr_;
        has_target_eulr = true;
      }
      if (has_fire_notify_)
      {
        fire_notify = latest_fire_notify_;
        has_fire_notify = true;
      }
      if (has_send_)
      {
        send = latest_send_;
        has_send = true;
      }
      const uint64_t ekf_sync_delta_us =
          synced_ekf_image_timestamp_us_ > armors.image_timestamp_us
              ? (synced_ekf_image_timestamp_us_ - armors.image_timestamp_us)
              : (armors.image_timestamp_us - synced_ekf_image_timestamp_us_);
      if (has_synced_ekf_points_ && ekf_sync_delta_us <= 20000)
      {
        ekf_points = synced_ekf_points_;
        has_ekf_points = true;
      }
      if (has_candidate_debug_)
      {
        candidate_debug = latest_candidate_debug_;
        has_candidate_debug = true;
      }
      if (has_independent_tracks_)
      {
        independent_tracks = latest_independent_tracks_;
        has_independent_tracks = true;
      }
      cam_info = camera_info_;
    }

    if (has_independent_tracks)
    {
      has_track_truth = AnalyzeIndependentTruth(armors, &independent_tracks, cam_info,
                                                detection_truth, track_truth);
    }
    if (has_ekf_points)
    {
      AnalyzeEkfTruth(ekf_points, armors.image_timestamp_us, cam_info);
    }
    RunDetectorCornerAudit(armors, cam_info);

    cv::Mat track_view = image.clone();
    cv::Mat full_view = image.clone();
    cv::drawMarker(track_view, cv::Point(image.cols / 2, image.rows / 2),
                   cv::Scalar(80, 220, 255), cv::MARKER_CROSS, 18, 1, cv::LINE_AA);
    cv::drawMarker(full_view, cv::Point(image.cols / 2, image.rows / 2),
                   cv::Scalar(80, 220, 255), cv::MARKER_CROSS, 18, 1, cv::LINE_AA);
    DrawPaneTitle(track_view, "Independent Armor Tracks");
    DrawPaneTitle(full_view, TruthQuadOverlayEnabled() ? "Detector vs Matched Truth"
                                                       : "Whole-Car Context");
    DrawDetectorArmors(track_view, armors, nullptr,
                       has_independent_tracks ? &independent_tracks : nullptr,
                       has_track_truth ? &detection_truth : nullptr);
    if (has_ekf_points && !TruthQuadOverlayEnabled())
    {
      DrawEkfOverlay(full_view, ekf_points, cam_info);
    }
    DrawDetectorArmors(full_view, armors, has_candidate_debug ? &candidate_debug : nullptr,
                      has_independent_tracks ? &independent_tracks : nullptr,
                      has_track_truth ? &detection_truth : nullptr);
    DrawMatchedTruthOverlay(full_view, armors, cam_info);

    cv::Mat canvas(image.rows + kCandidatePanelHeight, image.cols * 2 + kSummaryWidth,
                   CV_8UC3, cv::Scalar(18, 22, 28));
    track_view.copyTo(canvas(cv::Rect(0, 0, image.cols, image.rows)));
    full_view.copyTo(canvas(cv::Rect(image.cols, 0, image.cols, image.rows)));
    cv::Mat panel = canvas(cv::Rect(image.cols * 2, 0, kSummaryWidth, image.rows));
    DrawPanel(panel, armors, *target, has_info ? &info : nullptr,
              has_target_eulr ? &target_eulr : nullptr,
              has_fire_notify ? &fire_notify : nullptr,
              has_send ? &send : nullptr,
              has_ekf_points ? &ekf_points : nullptr,
              has_independent_tracks ? &independent_tracks : nullptr,
              has_candidate_debug ? &candidate_debug : nullptr);
    cv::Mat candidate_panel =
        canvas(cv::Rect(0, image.rows, canvas.cols, kCandidatePanelHeight));
    DrawIndependentTracksPanel(candidate_panel,
                               has_independent_tracks ? &independent_tracks : nullptr,
                               has_track_truth ? &track_truth : nullptr);

    std::lock_guard<std::mutex> lock(writer_lock_);
    if (done_.load(std::memory_order_relaxed))
    {
      return;
    }

    if (!writer_.isOpened())
    {
      writer_.open(video_path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), output_fps_,
                   canvas.size(), true);
      if (!writer_.isOpened())
      {
        done_.store(true, std::memory_order_relaxed);
        WriteSummary("open_failed");
        XR_LOG_ERROR("TrackerVideoRecorder failed to open writer: %s", video_path_.c_str());
        return;
      }
      XR_LOG_PASS("TrackerVideoRecorder opened: %s (%dx%d @ %.1ffps)", video_path_.c_str(),
                  canvas.cols, canvas.rows, output_fps_);
    }

    writer_.write(canvas);
    frame_count_++;

    if ((frame_count_ % 100U) == 0U)
    {
      XR_LOG_PASS("TrackerVideoRecorder frames=%u", frame_count_);
    }

    if (frame_count_ >= max_frames_)
    {
      writer_.release();
      done_.store(true, std::memory_order_relaxed);
      WriteSummary("done");
      XR_LOG_PASS("TrackerVideoRecorder done: frames=%u path=%s", frame_count_,
                  video_path_.c_str());
    }
  }

  void WriteSummary(const char* status)
  {
    {
      std::lock_guard<std::mutex> truth_lock(truth_lock_);
      if (truth_file_.is_open())
      {
        truth_file_.flush();
      }
      if (ekf_truth_file_.is_open())
      {
        ekf_truth_file_.flush();
      }
    }
    {
      std::lock_guard<std::mutex> detector_corner_lock(detector_corner_audit_lock_);
      if (detector_corner_audit_file_.is_open())
      {
        detector_corner_audit_file_.flush();
      }
    }
    std::ofstream f(summary_path_, std::ios::out | std::ios::trunc);
    f << "status=" << status << '\n';
    f << "video=" << video_path_ << '\n';
    f << "frames=" << frame_count_ << '\n';
    f << "fps=" << output_fps_ << '\n';
    f << "max_frames=" << max_frames_ << '\n';
    f << "independent_frames=" << stats_.frames << '\n';
    f << "independent_candidate_total=" << stats_.candidate_total << '\n';
    f << "independent_matched_total=" << stats_.matched_total << '\n';
    f << "independent_new_track_total=" << stats_.new_track_total << '\n';
    f << "independent_removed_total=" << stats_.removed_total << '\n';
    f << "independent_suppressed_spawn_total=" << stats_.suppressed_spawn_total << '\n';
    f << "independent_duplicate_pair_frames=" << stats_.duplicate_pair_frames << '\n';
    f << "independent_duplicate_pair_total=" << stats_.duplicate_pair_total << '\n';
    f << "ekf_truth_audit=" << ekf_truth_path_ << '\n';
    f << "ekf_truth_audit_summary=" << ekf_truth_summary_path_ << '\n';
    if (DetectorCornerAuditEnabled())
    {
      f << "detector_corner_audit=" << detector_corner_audit_path_ << '\n';
      f << "detector_corner_audit_summary=" << detector_corner_audit_summary_path_ << '\n';
    }
    WriteEkfTruthSummary();
    WriteDetectorCornerAuditSummary();
  }

 private:
  static constexpr int kSummaryWidth = 460;
  static constexpr int kCandidatePanelHeight = 420;
  std::string video_path_;
  std::string summary_path_;
  std::string truth_path_;
  std::string ekf_truth_path_;
  std::string ekf_truth_summary_path_;
  std::string detector_corner_audit_path_;
  std::string detector_corner_audit_summary_path_;
  std::atomic<bool> done_{false};
  uint32_t frame_count_{0};
  uint32_t max_frames_{80};
  double output_fps_{100.0};

  std::mutex image_lock_{};
  cv::Mat latest_image_{};

  std::mutex state_lock_{};
  std::shared_ptr<CameraBase::CameraInfo> camera_info_{};
  ArmorDetectionsMessage latest_armors_{};
  ArmorTracker::TrackerInfo latest_info_{};
  LibXR::EulerAngle<float> latest_target_eulr_{};
  uint8_t latest_fire_notify_{0};
  ArmorTracker::Send latest_send_{};
  ArmorTracker::EkfPointsMsg latest_ekf_points_{};
  ArmorTracker::EkfPointsMsg synced_ekf_points_{};
  ArmorTracker::CandidateDebugMsg latest_candidate_debug_{};
  uint64_t synced_ekf_image_timestamp_us_{0};
  std::array<IndependentArmorTrack, kMaxIndependentTracks> independent_tracks_{};
  IndependentTracksSnapshot latest_independent_tracks_{};
  uint16_t next_independent_track_id_{0};
  uint16_t next_independent_internal_track_id_{0};
  bool has_armors_{false};
  bool has_info_{false};
  bool has_target_eulr_{false};
  bool has_fire_notify_{false};
  bool has_send_{false};
  bool has_ekf_points_{false};
  bool has_synced_ekf_points_{false};
  bool has_candidate_debug_{false};
  bool has_independent_tracks_{false};

  std::mutex writer_lock_{};
  cv::VideoWriter writer_{};
  IndependentTrackStats stats_{};

  webots::Supervisor* supervisor_ = nullptr;
  webots::Node* target_spin_node_ = nullptr;
  webots::Node* camera_node_ = nullptr;
  webots::Node* target_robot_node_ = nullptr;
  webots::Node* self_robot_node_ = nullptr;
  std::array<webots::Node*, 4> armor_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  std::array<webots::Node*, 4> visible_face_nodes_{{nullptr, nullptr, nullptr, nullptr}};
  bool truth_resolve_failure_logged_{false};
  bool target_spin_pose_logged_{false};
  bool camera_pose_logged_{false};
  std::array<bool, 4> armor_pose_logged_{{false, false, false, false}};
  std::array<bool, 4> visible_face_pose_logged_{{false, false, false, false}};

  std::mutex truth_lock_{};
  std::ofstream truth_file_{};
  std::ofstream ekf_truth_file_{};
  std::map<uint16_t, TrackTruthPersistent> truth_persistent_{};
  std::array<TruthGtPersistent, 4> truth_gt_persistent_{};
  IndependentTruthStats truth_stats_{};
  EkfTruthStats ekf_truth_stats_{};

  std::mutex detector_corner_audit_lock_{};
  std::ofstream detector_corner_audit_file_{};
  DetectorCornerAuditStats detector_corner_audit_stats_{};
};
