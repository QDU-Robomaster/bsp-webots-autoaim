#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_set>

#include <Eigen/Dense>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "CameraFrameSync.hpp"
#include "armor.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "webots_truth_visible_plane.hpp"
#include "xrobot_constexpr.hpp"

class DetectorVideoRecorder
{
 public:
  using MainFrameSync = CameraFrameSync<ProjectConstexpr::MainCameraInfo>;
  using ImageFrame = MainFrameSync::ImageFrame;
  using SyncedFrame = MainFrameSync::SyncedFrame;

  static constexpr uint32_t kSyncFrameWaitTimeoutMs = 100;

  DetectorVideoRecorder()
  {
    const char* video_env = std::getenv("XR_DETECTOR_VIDEO_PATH");
    if (video_env != nullptr && video_env[0] != '\0')
    {
      video_path_ = video_env;
    }
    else
    {
      video_path_ = "detector_overlay_100fps.mp4";
    }

    const char* summary_env = std::getenv("XR_DETECTOR_VIDEO_SUMMARY_PATH");
    if (summary_env != nullptr && summary_env[0] != '\0')
    {
      summary_path_ = summary_env;
    }
    else
    {
      summary_path_ = video_path_ + ".summary.txt";
    }

    const char* max_frames_env = std::getenv("XR_DETECTOR_VIDEO_MAX_FRAMES");
    if (max_frames_env != nullptr && max_frames_env[0] != '\0')
    {
      char* end = nullptr;
      const unsigned long parsed = std::strtoul(max_frames_env, &end, 10);
      if (end != max_frames_env && parsed > 0UL)
      {
        max_frames_ = static_cast<uint32_t>(parsed);
      }
    }

    const char* fps_env = std::getenv("XR_DETECTOR_VIDEO_FPS");
    if (fps_env != nullptr && fps_env[0] != '\0')
    {
      char* end = nullptr;
      const double parsed = std::strtod(fps_env, &end);
      if (end != fps_env && std::isfinite(parsed) && parsed > 0.0)
      {
        output_fps_ = parsed;
      }
    }

    const char* truth_overlay_env = std::getenv("XR_DETECTOR_VIDEO_TRUTH_OVERLAY");
    truth_overlay_enabled_ =
        truth_overlay_env != nullptr && truth_overlay_env[0] != '\0' &&
        truth_overlay_env[0] != '0';
  }

  void Init(webots::Supervisor* supervisor) { supervisor_ = supervisor; }

  void InstallBlocking()
  {
    LibXR::Topic::Domain armor_domain("armor_detector");
    auto armors_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("armors_result", UINT32_MAX, &armor_domain));
    auto armors_cb = LibXR::Topic::Callback::Create(
        [](bool, DetectorVideoRecorder* self, LibXR::RawData& data)
        {
          auto* armors = reinterpret_cast<ArmorDetectionsMessage*>(data.addr_);
          self->ArmorsCallback(armors);
        },
        this);
    armors_topic.RegisterCallback(armors_cb);

    XR_LOG_PASS(
        "DetectorVideoRecorder subscribed: image=%s imu=%s + armor_detector/armors_result -> %s",
        ProjectConstexpr::MainImageTopicName, ProjectConstexpr::MainImuTopicName,
        video_path_.c_str());

    while (!Done())
    {
      MainFrameSync::Subscriber subscriber(ProjectConstexpr::MainImageTopicName,
                                           ProjectConstexpr::MainImuTopicName);
      if (!subscriber.Valid())
      {
        LibXR::Thread::Sleep(200);
        continue;
      }

      SyncedFrame synced_frame;
      while (!Done())
      {
        const auto wait_ans = subscriber.Wait(synced_frame, kSyncFrameWaitTimeoutMs);
        if (wait_ans == LibXR::ErrorCode::TIMEOUT)
        {
          continue;
        }
        if (wait_ans != LibXR::ErrorCode::OK)
        {
          break;
        }

        const ImageFrame* frame = synced_frame.GetImageFrame();
        if (frame != nullptr)
        {
          ImageCallback(*frame);
        }
      }
    }
  }

  bool Done() const { return done_.load(std::memory_order_relaxed); }

 private:
  struct CachedFrame
  {
    uint64_t timestamp_us{0};
    cv::Mat image{};
  };

  struct CachedArmors
  {
    uint64_t timestamp_us{0};
    ArmorDetectionsMessage msg{};
  };

  struct Pose3d
  {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  };

  static constexpr std::size_t kFrameCacheSize = 32;
  static constexpr std::size_t kArmorsCacheSize = 64;
  static constexpr std::array<const char*, 4> kTruthLabels = {
      "armor_front", "armor_right", "armor_back", "armor_left"};
  static constexpr const char* kTruthVisibleFaceProtoDef = "XR_VISIBLE_FACE_POSE";
  static constexpr const char* kTruthLeftLightbarProtoDef = "XR_LEFT_LIGHTBAR_POSE";
  static constexpr const char* kTruthRightLightbarProtoDef = "XR_RIGHT_LIGHTBAR_POSE";

  static int SyncFrameCvType(CameraTypes::Encoding encoding)
  {
    switch (encoding)
    {
      case CameraTypes::Encoding::RGB8:
      case CameraTypes::Encoding::BGR8:
        return CV_8UC3;
      case CameraTypes::Encoding::RGBA8:
      case CameraTypes::Encoding::BGRA8:
        return CV_8UC4;
      case CameraTypes::Encoding::MONO8:
        return CV_8UC1;
      default:
        return -1;
    }
  }

  static cv::Mat SyncFrameToBgr(const ImageFrame& frame)
  {
    const auto encoding = ProjectConstexpr::MainCameraInfo.encoding;
    const int cv_type = SyncFrameCvType(encoding);
    if (cv_type < 0)
    {
      return {};
    }

    cv::Mat input(static_cast<int>(ProjectConstexpr::MainCameraInfo.height),
                  static_cast<int>(ProjectConstexpr::MainCameraInfo.width), cv_type,
                  const_cast<uint8_t*>(frame.data.data()),
                  static_cast<size_t>(ProjectConstexpr::MainCameraInfo.step));
    switch (encoding)
    {
      case CameraTypes::Encoding::RGB8:
      {
        cv::Mat output;
        cv::cvtColor(input, output, cv::COLOR_RGB2BGR);
        return output;
      }
      case CameraTypes::Encoding::BGRA8:
      {
        cv::Mat output;
        cv::cvtColor(input, output, cv::COLOR_BGRA2BGR);
        return output;
      }
      case CameraTypes::Encoding::RGBA8:
      {
        cv::Mat output;
        cv::cvtColor(input, output, cv::COLOR_RGBA2BGR);
        return output;
      }
      case CameraTypes::Encoding::MONO8:
      {
        cv::Mat output;
        cv::cvtColor(input, output, cv::COLOR_GRAY2BGR);
        return output;
      }
      default:
        return input.clone();
    }
  }

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

  static void DrawLabel(cv::Mat& frame, const cv::Point& origin, const std::string& text,
                        const cv::Scalar& color)
  {
    constexpr int kFont = cv::FONT_HERSHEY_DUPLEX;
    constexpr double kScale = 0.50;
    constexpr int kThickness = 1;
    int baseline = 0;
    const cv::Size size = cv::getTextSize(text, kFont, kScale, kThickness, &baseline);
    const cv::Rect bg(origin.x, std::max(0, origin.y - size.height - 8),
                      size.width + 10, size.height + 10);
    cv::rectangle(frame, bg, color, cv::FILLED, cv::LINE_AA);
    cv::putText(frame, text, cv::Point(bg.x + 5, bg.y + size.height + 1), kFont, kScale,
                cv::Scalar(12, 16, 24), kThickness, cv::LINE_AA);
  }

  static bool ReadRelativePose(webots::Node* node, const webots::Node* from_node,
                               Pose3d& pose)
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
    return true;
  }

  static webots::Node* FindNamedNodeRecursiveImpl(webots::Node* node,
                                                  const char* wanted_name,
                                                  std::unordered_set<const webots::Node*>& visited)
  {
    if (node == nullptr || wanted_name == nullptr)
    {
      return nullptr;
    }
    if (!visited.insert(node).second)
    {
      return nullptr;
    }

    const std::string current_name = node->getField("name") != nullptr
                                         ? node->getField("name")->getSFString()
                                         : std::string();
    if (current_name == wanted_name)
    {
      return node;
    }

    webots::Field* children = node->getField("children");
    if (children == nullptr)
    {
      return nullptr;
    }

    const int count = children->getCount();
    for (int i = 0; i < count; ++i)
    {
      webots::Node* child = children->getMFNode(i);
      if (webots::Node* found = FindNamedNodeRecursiveImpl(child, wanted_name, visited);
          found != nullptr)
      {
        return found;
      }
    }
    return nullptr;
  }

  static webots::Node* FindNamedNodeRecursive(webots::Node* node, const char* wanted_name)
  {
    std::unordered_set<const webots::Node*> visited;
    return FindNamedNodeRecursiveImpl(node, wanted_name, visited);
  }

  bool ResolveTruthNodes() const
  {
    bool all_armors_resolved = true;
    for (auto* node : armor_nodes_)
    {
      all_armors_resolved = all_armors_resolved && (node != nullptr);
    }
    if (camera_node_ != nullptr && all_armors_resolved)
    {
      return true;
    }
    if (supervisor_ == nullptr)
    {
      return false;
    }

    camera_node_ = supervisor_->getFromDef("camera");
    webots::Node* root = supervisor_->getRoot();
    if (camera_node_ == nullptr || root == nullptr)
    {
      return false;
    }

    for (std::size_t i = 0; i < armor_nodes_.size(); ++i)
    {
      if (armor_nodes_[i] == nullptr)
      {
        armor_nodes_[i] = FindNamedNodeRecursive(root, kTruthLabels[i]);
      }
      if (armor_nodes_[i] == nullptr)
      {
        return false;
      }
      if (visible_face_nodes_[i] == nullptr && armor_nodes_[i]->isProto())
      {
        visible_face_nodes_[i] = armor_nodes_[i]->getFromProtoDef(kTruthVisibleFaceProtoDef);
      }
      if (left_lightbar_nodes_[i] == nullptr && armor_nodes_[i]->isProto())
      {
        left_lightbar_nodes_[i] = armor_nodes_[i]->getFromProtoDef(kTruthLeftLightbarProtoDef);
      }
      if (right_lightbar_nodes_[i] == nullptr && armor_nodes_[i]->isProto())
      {
        right_lightbar_nodes_[i] =
            armor_nodes_[i]->getFromProtoDef(kTruthRightLightbarProtoDef);
      }
    }

    XR_LOG_PASS("DetectorVideoRecorder resolved truth overlay nodes");
    return true;
  }

  static void DrawTruthFace(cv::Mat& frame, const WebotsTruthVisibleFace& face,
                            const cv::Scalar& color, const std::string& label)
  {
    if (!face.valid)
    {
      return;
    }

    std::array<cv::Point, 4> polygon{};
    for (std::size_t i = 0; i < face.points.size(); ++i)
    {
      polygon[i] = face.points[i];
    }
    const cv::Point* points = polygon.data();
    const int count = static_cast<int>(polygon.size());
    cv::polylines(frame, &points, &count, 1, true, color, 2, cv::LINE_AA);
    for (const auto& point : polygon)
    {
      cv::drawMarker(frame, point, color, cv::MARKER_TILTED_CROSS, 8, 1, cv::LINE_AA);
    }
    DrawLabel(frame, cv::Point(std::max(4, static_cast<int>(face.center.x) + 6),
                               std::max(20, static_cast<int>(face.center.y) - 6)),
              label, color);
  }

  void DrawTruthOverlay(cv::Mat& frame) const
  {
    if (!truth_overlay_enabled_ || !ResolveTruthNodes())
    {
      return;
    }

    const auto& camera_info = ProjectConstexpr::MainCameraInfo;
    for (std::size_t i = 0; i < armor_nodes_.size(); ++i)
    {
      Pose3d armor_in_camera_node;
      if (!ReadRelativePose(armor_nodes_[i], camera_node_, armor_in_camera_node))
      {
        continue;
      }

      WebotsTruthVisibleFace sp_face;
      WebotsTruthVisibleFace fit_face;
      bool sp_ok = false;
      bool fit_ok = false;

      if (left_lightbar_nodes_[i] != nullptr && right_lightbar_nodes_[i] != nullptr)
      {
        Pose3d left_lightbar_in_camera_node;
        Pose3d right_lightbar_in_camera_node;
        if (ReadRelativePose(left_lightbar_nodes_[i], camera_node_,
                             left_lightbar_in_camera_node) &&
            ReadRelativePose(right_lightbar_nodes_[i], camera_node_,
                             right_lightbar_in_camera_node))
        {
          sp_ok = ProjectWebotsTruthLightbarCenterlines(
              camera_info,
              WebotsCameraNodeToOpticalRotation() * left_lightbar_in_camera_node.rotation,
              WebotsCameraNodeToOpticalRotation() *
                  left_lightbar_in_camera_node.translation,
              WebotsCameraNodeToOpticalRotation() *
                  right_lightbar_in_camera_node.rotation,
              WebotsCameraNodeToOpticalRotation() *
                  right_lightbar_in_camera_node.translation,
              sp_face);
        }
      }

      if (!sp_ok && visible_face_nodes_[i] != nullptr)
      {
        Pose3d visible_face_in_camera_node;
        if (ReadRelativePose(visible_face_nodes_[i], camera_node_,
                             visible_face_in_camera_node))
        {
          const Eigen::Matrix3d r_optical_armor_root =
              WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.rotation;
          const Eigen::Matrix3d r_optical_box =
              WebotsCameraNodeToOpticalRotation() * visible_face_in_camera_node.rotation;
          const Eigen::Vector3d t_optical_box =
              WebotsCameraNodeToOpticalRotation() * visible_face_in_camera_node.translation +
              r_optical_armor_root * WebotsTruthVisiblePlaneDelta();
          sp_ok = ProjectWebotsTruthFaceBox(
              camera_info, r_optical_box, t_optical_box, 0.135, 0.056,
              kWebotsTruthSpArmorDepth, sp_face);
          fit_ok = ProjectWebotsTruthFaceBox(
              camera_info, r_optical_box, t_optical_box, 0.135, 0.0525,
              kWebotsTruthSpArmorDepth, fit_face);
        }
      }

      if (!sp_ok || !fit_ok)
      {
        const Eigen::Matrix3d r_optical_armor_root =
            WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.rotation;
        const Eigen::Vector3d t_optical_armor_root =
            WebotsCameraNodeToOpticalRotation() * armor_in_camera_node.translation;
        if (!sp_ok)
        {
          sp_ok = ProjectWebotsTruthVisibleFace(camera_info, r_optical_armor_root,
                                                t_optical_armor_root, sp_face);
        }
        if (!fit_ok)
        {
          fit_ok = ProjectWebotsTruthFaceBox(
              camera_info, r_optical_armor_root *
                               Eigen::AngleAxisd(-CV_PI * 0.5, Eigen::Vector3d::UnitX())
                                   .toRotationMatrix(),
              t_optical_armor_root +
                  r_optical_armor_root * Eigen::Vector3d(0.0, -0.009, -0.0008),
              0.135, 0.0525, kWebotsTruthSpArmorDepth, fit_face);
        }
      }

      if (sp_ok)
      {
        DrawTruthFace(frame, sp_face, cv::Scalar(0, 255, 255),
                      std::string("T56 ") + kTruthLabels[i]);
      }
      if (fit_ok)
      {
        DrawTruthFace(frame, fit_face, cv::Scalar(0, 165, 255),
                      std::string("T52.5 ") + kTruthLabels[i]);
      }
    }
  }

  void ImageCallback(const ImageFrame& frame)
  {
    if (Done())
    {
      return;
    }

    cv::Mat image = SyncFrameToBgr(frame);
    if (image.empty())
    {
      return;
    }

    ArmorDetectionsMessage armors;
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(cache_lock_);
      const auto it =
          std::find_if(armors_cache_.begin(), armors_cache_.end(),
                       [&](const CachedArmors& cached)
                       { return cached.timestamp_us == frame.timestamp_us; });
      if (it != armors_cache_.end())
      {
        armors = it->msg;
        armors_cache_.erase(it);
        matched = true;
      }
      else
      {
        frame_cache_.push_back({frame.timestamp_us, std::move(image)});
        while (frame_cache_.size() > kFrameCacheSize)
        {
          frame_cache_.pop_front();
        }
      }
    }

    if (matched)
    {
      EmitFrame(std::move(image), armors);
    }
  }

  void ArmorsCallback(ArmorDetectionsMessage* msg)
  {
    if (msg == nullptr || Done())
    {
      return;
    }

    cv::Mat image;
    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(cache_lock_);
      const auto it =
          std::find_if(frame_cache_.begin(), frame_cache_.end(),
                       [&](const CachedFrame& cached)
                       { return cached.timestamp_us == msg->image_timestamp_us; });
      if (it != frame_cache_.end())
      {
        image = it->image.clone();
        frame_cache_.erase(it);
        matched = true;
      }
      else
      {
        armors_cache_.push_back({msg->image_timestamp_us, *msg});
        while (armors_cache_.size() > kArmorsCacheSize)
        {
          armors_cache_.pop_front();
        }
      }
    }

    if (matched)
    {
      EmitFrame(std::move(image), *msg);
    }
  }

  void EmitFrame(cv::Mat frame, const ArmorDetectionsMessage& msg)
  {
    if (frame.empty())
    {
      return;
    }

    DrawOverlay(frame, msg);

    std::lock_guard<std::mutex> lock(writer_lock_);
    if (done_.load(std::memory_order_relaxed))
    {
      return;
    }

    if (!writer_.isOpened())
    {
      writer_.open(video_path_, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), output_fps_,
                   frame.size(), true);
      if (!writer_.isOpened())
      {
        done_.store(true, std::memory_order_relaxed);
        WriteSummary("open_failed");
        XR_LOG_ERROR("DetectorVideoRecorder failed to open writer: %s",
                     video_path_.c_str());
        return;
      }
      XR_LOG_PASS("DetectorVideoRecorder opened: %s (%dx%d @ %.1ffps)",
                  video_path_.c_str(), frame.cols, frame.rows, output_fps_);
    }

    writer_.write(frame);
    frame_count_++;

    if ((frame_count_ % 100U) == 0U)
    {
      XR_LOG_PASS("DetectorVideoRecorder frames=%u", frame_count_);
    }

    if (frame_count_ >= max_frames_)
    {
      writer_.release();
      done_.store(true, std::memory_order_relaxed);
      WriteSummary("done");
      XR_LOG_PASS("DetectorVideoRecorder done: frames=%u path=%s", frame_count_,
                  video_path_.c_str());
    }
  }

  void DrawOverlay(cv::Mat& frame, const ArmorDetectionsMessage& msg) const
  {
    const cv::Point image_center(frame.cols / 2, frame.rows / 2);
    cv::drawMarker(frame, image_center, cv::Scalar(80, 220, 255), cv::MARKER_CROSS, 18, 1,
                   cv::LINE_AA);

    cv::rectangle(frame, cv::Rect(0, 0, frame.cols, 34), cv::Scalar(18, 22, 28),
                  cv::FILLED, cv::LINE_AA);
    std::ostringstream header;
    header << "detector armors=" << msg.results.size() << " ts="
           << static_cast<unsigned long long>(msg.image_timestamp_us);
    cv::putText(frame, header.str(), cv::Point(12, 23), cv::FONT_HERSHEY_DUPLEX, 0.62,
                cv::Scalar(240, 244, 250), 1, cv::LINE_AA);

    for (const auto& armor : msg.results)
    {
      const cv::Scalar color = ColorToScalar(armor.color);
      std::array<cv::Point, 4> polygon{};
      for (std::size_t i = 0; i < armor.points.size(); ++i)
      {
        polygon[i] = armor.points[i];
      }
      const cv::Point* points = polygon.data();
      const int count = static_cast<int>(polygon.size());
      cv::polylines(frame, &points, &count, 1, true, color, 2, cv::LINE_AA);
      cv::rectangle(frame, armor.box, color, 1, cv::LINE_AA);
      for (const auto& point : armor.points)
      {
        cv::circle(frame, point, 3, color, cv::FILLED, cv::LINE_AA);
      }
      cv::circle(frame, armor.center, 4, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

      std::ostringstream label;
      label << ArmorNumberString(armor.number) << ' ' << ArmorTypeString(armor.type) << ' '
            << std::fixed << std::setprecision(2) << armor.confidence;
      DrawLabel(frame, cv::Point(std::max(armor.box.x, 4), std::max(armor.box.y - 4, 20)),
                label.str(), color);
    }

    DrawTruthOverlay(frame);
  }

  void WriteSummary(const char* status)
  {
    std::ofstream f(summary_path_, std::ios::out | std::ios::trunc);
    f << "status=" << status << '\n';
    f << "video=" << video_path_ << '\n';
    f << "frames=" << frame_count_ << '\n';
    f << "fps=" << output_fps_ << '\n';
  }

 private:
  std::string video_path_{};
  std::string summary_path_{};
  double output_fps_{100.0};
  uint32_t max_frames_{1000};
  std::atomic<bool> done_{false};
  uint32_t frame_count_{0};
  std::mutex cache_lock_{};
  std::mutex writer_lock_{};
  std::deque<CachedFrame> frame_cache_{};
  std::deque<CachedArmors> armors_cache_{};
  bool truth_overlay_enabled_{false};
  webots::Supervisor* supervisor_{nullptr};
  mutable webots::Node* camera_node_{nullptr};
  mutable std::array<webots::Node*, 4> armor_nodes_{};
  mutable std::array<webots::Node*, 4> visible_face_nodes_{};
  mutable std::array<webots::Node*, 4> left_lightbar_nodes_{};
  mutable std::array<webots::Node*, 4> right_lightbar_nodes_{};
  cv::VideoWriter writer_{};
};
