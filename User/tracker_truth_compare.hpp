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
#include <limits>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include "ArmorTracker.hpp"
#include "CameraFrameSync.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "transform.hpp"
#include "webots_truth_visible_plane.hpp"
#include "xrobot_constexpr.hpp"

class TrackerTruthCompare
{
 public:
  using MainFrameSync = CameraFrameSync<ProjectConstexpr::MainCameraInfo>;
  using MainArmorTracker = ArmorTracker<ProjectConstexpr::MainCameraInfo>;
  using SyncedFrame = MainFrameSync::SyncedFrame;

  TrackerTruthCompare()
      : gimbal_to_camera_transform_static_(
            LibXR::Quaternion<double>(0.5, -0.5, 0.5, -0.5),
            LibXR::Position<double>(0.0, 0.0, 0.0))
  {
    const char *pairs_env = std::getenv("XR_TRACKER_TRUTH_COMPARE_PATH");
    if (pairs_env != nullptr && pairs_env[0] != '\0')
    {
      pairs_path_ = pairs_env;
    }
    else
    {
      pairs_path_ = "tracker_truth_compare_pairs.tsv";
    }

    frames_path_ = pairs_path_ + ".frames.tsv";
    summary_path_ = pairs_path_ + ".summary.txt";

    if (const char *max_frames_env = std::getenv("XR_TRACKER_TRUTH_COMPARE_MAX_FRAMES"))
    {
      char *end = nullptr;
      const unsigned long parsed = std::strtoul(max_frames_env, &end, 10);
      if (end != max_frames_env && parsed > 0UL)
      {
        max_frames_ = static_cast<uint32_t>(
            std::min<unsigned long>(parsed, std::numeric_limits<uint32_t>::max()));
      }
    }

    if (const char *cache_env = std::getenv("XR_TRACKER_TRUTH_COMPARE_CACHE_SIZE"))
    {
      char *end = nullptr;
      const unsigned long parsed = std::strtoul(cache_env, &end, 10);
      if (end != cache_env && parsed > 0UL)
      {
        truth_frame_cache_size_ = static_cast<std::size_t>(
            std::min<unsigned long>(parsed, 16384UL));
      }
    }
  }

  ~TrackerTruthCompare()
  {
    std::lock_guard<std::mutex> lock(file_lock_);
    CloseFilesLocked();
    WriteSummaryLocked(done_.load(std::memory_order_relaxed) ? "done" : "exit");
  }

  void Init(webots::Supervisor *supervisor) { supervisor_ = supervisor; }

  void InstallBlocking()
  {
    LibXR::Topic::Domain tracker_domain("tracker");
    auto ekf_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("ekf_points", UINT32_MAX, &tracker_domain));
    auto ekf_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerTruthCompare *self, LibXR::RawData &data)
        {
          auto *msg = reinterpret_cast<MainArmorTracker::EkfPointsMsg *>(data.addr_);
          self->EkfPointsCallback(msg);
        },
        this);
    ekf_topic.RegisterCallback(ekf_cb);

    XR_LOG_PASS(
        "TrackerTruthCompare subscribed: image=%s imu=%s tracker/ekf_points -> %s",
        ProjectConstexpr::MainImageTopicName, ProjectConstexpr::MainImuTopicName,
        pairs_path_.c_str());

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

        SyncFrameCallback(synced_frame);
      }
    }
  }

  bool Done() const { return done_.load(std::memory_order_relaxed); }

 private:
  static constexpr std::array<const char *, 4> kTruthLabels = {
      "front", "right", "back", "left"};
  static constexpr std::array<const char *, 4> kTruthNodeLabels = {
      "armor_front", "armor_right", "armor_back", "armor_left"};
  static constexpr std::array<std::array<double, 3>, 4> kArmorLocalPosSpin = {
      std::array<double, 3>{0.0, 0.205, -0.06},
      std::array<double, 3>{0.205, 0.0, -0.11},
      std::array<double, 3>{0.0, -0.205, -0.06},
      std::array<double, 3>{-0.205, 0.0, -0.11}};
  static constexpr uint32_t kSyncFrameWaitTimeoutMs = 100;
  static constexpr std::size_t kTruthFrameCacheSize = 64;

  struct TruthFrameSnapshot
  {
    uint64_t image_timestamp_us{0};
    double sim_time_s{0.0};
    LibXR::Transform<double> camera_pose_world{};
    LibXR::Transform<double> tracker_camera_pose_world{};
    std::array<Eigen::Vector3d, 4> gt_world{};
    std::array<Eigen::Vector3d, 4> gt_cam{};
    std::array<Eigen::Vector3d, 4> gt_tracker_cam{};
    std::array<bool, 4> gt_visible{};
  };

  struct AssignmentResult
  {
    std::vector<int> pred_indices{};
    std::vector<int> gt_index_for_pred{};
    std::vector<double> pair_error_m{};
    double total_error_m = std::numeric_limits<double>::infinity();
    bool valid = false;
  };

  static bool IsFiniteVec(const Eigen::Vector3d &v)
  {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  static double DistanceMeters(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
  {
    return (a - b).norm();
  }

  static Eigen::Vector3d MeanPoint(const std::array<Eigen::Vector3d, 4> &points,
                                   const std::vector<int> &indices)
  {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    if (indices.empty())
    {
      return mean;
    }
    for (const int index : indices)
    {
      mean += points[index];
    }
    return mean / static_cast<double>(indices.size());
  }

  void SyncFrameCallback(const SyncedFrame &synced_frame)
  {
    const auto *image_frame = synced_frame.GetImageFrame();
    if (image_frame == nullptr)
    {
      return;
    }

    if (supervisor_ == nullptr)
    {
      skipped_no_supervisor_++;
      return;
    }

    if (!ResolveNodes())
    {
      skipped_no_nodes_++;
      return;
    }

    TruthFrameSnapshot snapshot;
    snapshot.image_timestamp_us = image_frame->timestamp_us;
    snapshot.sim_time_s = supervisor_->getTime();
    const LibXR::Quaternion<double> camera_rotation =
        armor_tracker_detail::PackedCameraRotation(synced_frame.imu.rotation_wxyz);
    snapshot.camera_pose_world = LibXR::Transform<double>(
        camera_rotation,
        LibXR::Position<double>(
            static_cast<double>(synced_frame.imu.translation_xyz[0]),
            static_cast<double>(synced_frame.imu.translation_xyz[1]),
            static_cast<double>(synced_frame.imu.translation_xyz[2])));
    snapshot.tracker_camera_pose_world =
        armor_tracker_detail::ArmorTrackerCameraRotationToTrackerWorldPose(
            camera_rotation,
            armor_tracker_detail::PackedCameraTranslation(
                synced_frame.imu.translation_xyz),
            gimbal_to_camera_transform_static_);

    if (!ReadTruthWorld(snapshot.gt_world) ||
        !ReadTruthCam(snapshot.gt_cam, snapshot.gt_visible))
    {
      skipped_bad_truth_++;
      return;
    }

    const auto r_tracker_wc =
        snapshot.tracker_camera_pose_world.rotation.ToRotationMatrix();
    const Eigen::Vector3d tracker_twc(
        snapshot.tracker_camera_pose_world.translation.x(),
        snapshot.tracker_camera_pose_world.translation.y(),
        snapshot.tracker_camera_pose_world.translation.z());
    const Eigen::Matrix3d r_tracker_cw = r_tracker_wc.transpose();
    for (int i = 0; i < 4; ++i)
    {
      snapshot.gt_tracker_cam[i] =
          r_tracker_cw * (snapshot.gt_world[i] - tracker_twc);
    }

    std::lock_guard<std::mutex> lock(state_lock_);
    truth_frame_cache_.push_back(snapshot);
    while (truth_frame_cache_.size() > truth_frame_cache_size_)
    {
      truth_frame_cache_.pop_front();
    }
  }

  bool LookupTruthFrame(uint64_t image_timestamp_us, TruthFrameSnapshot &snapshot_out)
  {
    std::lock_guard<std::mutex> lock(state_lock_);
    for (auto it = truth_frame_cache_.rbegin(); it != truth_frame_cache_.rend(); ++it)
    {
      if (it->image_timestamp_us == image_timestamp_us)
      {
        snapshot_out = *it;
        return true;
      }
    }
    return false;
  }

  std::size_t TruthFrameCacheSize()
  {
    std::lock_guard<std::mutex> lock(state_lock_);
    return truth_frame_cache_.size();
  }

  void EkfPointsCallback(MainArmorTracker::EkfPointsMsg *msg)
  {
    if (msg == nullptr || Done())
    {
      return;
    }

    if (msg->image_timestamp_us == 0)
    {
      skipped_no_truth_frame_++;
      return;
    }

    TruthFrameSnapshot truth_frame;
    if (!LookupTruthFrame(msg->image_timestamp_us, truth_frame))
    {
      skipped_no_truth_frame_++;
      if (skipped_no_truth_frame_ == 1 || (skipped_no_truth_frame_ % 100U) == 0U)
      {
        XR_LOG_WARN(
            "TrackerTruthCompare missing cached truth for ts=%llu (miss=%u, cache=%zu)",
            static_cast<unsigned long long>(msg->image_timestamp_us),
            skipped_no_truth_frame_, TruthFrameCacheSize());
      }
      return;
    }

    if (msg->count == 0)
    {
      skipped_bad_truth_++;
      return;
    }

    const auto r_wc = truth_frame.camera_pose_world.rotation.ToRotationMatrix();
    const Eigen::Vector3d twc(truth_frame.camera_pose_world.translation.x(),
                              truth_frame.camera_pose_world.translation.y(),
                              truth_frame.camera_pose_world.translation.z());

    std::array<Eigen::Vector3d, 4> ekf_cam{};
    std::array<Eigen::Vector3d, 4> ekf_world{};
    std::array<bool, 4> ekf_visible{};
    std::vector<int> pred_indices;
    pred_indices.reserve(std::min<int>(msg->count, 4));
    for (int i = 0; i < 4; ++i)
    {
      ekf_cam[i] = Eigen::Vector3d(msg->armors_cam[i].x(), msg->armors_cam[i].y(),
                                   msg->armors_cam[i].z());
      ekf_world[i] = r_wc * ekf_cam[i] + twc;
      ekf_visible[i] = msg->valid[i + 1];
      if (i < std::min<int>(msg->count, 4) && ekf_visible[i] && IsFiniteVec(ekf_cam[i]))
      {
        pred_indices.push_back(i);
      }
    }
    if (pred_indices.empty())
    {
      skipped_bad_truth_++;
      return;
    }

    const AssignmentResult assignment =
        SolveBestAssignment(truth_frame.gt_cam, ekf_cam, pred_indices);
    const AssignmentResult pseudo_assignment =
        SolveBestAssignment(truth_frame.gt_tracker_cam, ekf_cam, pred_indices);
    if (!assignment.valid || !pseudo_assignment.valid)
    {
      skipped_bad_truth_++;
      return;
    }
    const double sim_time_s = truth_frame.sim_time_s;
    const uint32_t frame_index = frame_count_;
    const double assignment_count = static_cast<double>(assignment.pred_indices.size());
    const double mean_error_m = assignment.total_error_m / assignment_count;
    const double max_error_m =
        *std::max_element(assignment.pair_error_m.begin(), assignment.pair_error_m.end());
    const Eigen::Vector3d gt_center_cam =
        MeanPoint(truth_frame.gt_cam, assignment.gt_index_for_pred);
    const Eigen::Vector3d ekf_center_cam =
        MeanPoint(ekf_cam, assignment.pred_indices);
    const double center_error_m = DistanceMeters(gt_center_cam, ekf_center_cam);
    double shape_error_sum_m = 0.0;
    for (std::size_t i = 0; i < assignment.pred_indices.size(); ++i)
    {
      const int pred_index = assignment.pred_indices[i];
      const int gt_index = assignment.gt_index_for_pred[i];
      shape_error_sum_m += DistanceMeters(
          truth_frame.gt_cam[gt_index] - gt_center_cam,
          ekf_cam[pred_index] - ekf_center_cam);
    }
    const double shape_mean_error_m = shape_error_sum_m / assignment_count;
    const double pseudo_mean_error_m =
        pseudo_assignment.total_error_m /
        static_cast<double>(pseudo_assignment.pred_indices.size());
    const double pseudo_max_error_m = *std::max_element(
        pseudo_assignment.pair_error_m.begin(), pseudo_assignment.pair_error_m.end());
    const Eigen::Vector3d gt_pseudo_center_cam =
        MeanPoint(truth_frame.gt_tracker_cam, pseudo_assignment.gt_index_for_pred);
    const double pseudo_center_error_m =
        DistanceMeters(gt_pseudo_center_cam, ekf_center_cam);
    double pseudo_shape_error_sum_m = 0.0;
    for (std::size_t i = 0; i < pseudo_assignment.pred_indices.size(); ++i)
    {
      const int pred_index = pseudo_assignment.pred_indices[i];
      const int gt_index = pseudo_assignment.gt_index_for_pred[i];
      pseudo_shape_error_sum_m += DistanceMeters(
          truth_frame.gt_tracker_cam[gt_index] - gt_pseudo_center_cam,
          ekf_cam[pred_index] - ekf_center_cam);
    }
    const double pseudo_shape_mean_error_m =
        pseudo_shape_error_sum_m /
        static_cast<double>(pseudo_assignment.pred_indices.size());
    const std::string assignment_str = AssignmentString(assignment);
    const std::string pseudo_assignment_str = AssignmentString(pseudo_assignment);

    {
      std::lock_guard<std::mutex> lock(file_lock_);
      OpenFilesLocked();
      for (std::size_t i = 0; i < assignment.pred_indices.size(); ++i)
      {
        const int pred_index = assignment.pred_indices[i];
        const int gt_index = assignment.gt_index_for_pred[i];
        pairs_file_ << frame_index << '\t' << msg->image_timestamp_us << '\t'
                    << std::fixed << std::setprecision(6) << sim_time_s << '\t'
                    << "A" << pred_index << '\t'
                    << kTruthLabels[gt_index] << '\t'
                    << assignment.pair_error_m[i] << '\t'
                    << (truth_frame.gt_visible[gt_index] ? 1 : 0) << '\t'
                    << (ekf_visible[pred_index] ? 1 : 0) << '\t'
                    << truth_frame.gt_world[gt_index].x() << '\t'
                    << truth_frame.gt_world[gt_index].y() << '\t'
                    << truth_frame.gt_world[gt_index].z() << '\t'
                    << truth_frame.gt_cam[gt_index].x() << '\t'
                    << truth_frame.gt_cam[gt_index].y() << '\t'
                    << truth_frame.gt_cam[gt_index].z() << '\t'
                    << ekf_cam[pred_index].x() << '\t'
                    << ekf_cam[pred_index].y() << '\t'
                    << ekf_cam[pred_index].z() << '\t'
                    << ekf_world[pred_index].x() << '\t'
                    << ekf_world[pred_index].y() << '\t'
                    << ekf_world[pred_index].z() << '\n';
      }

      frames_file_ << frame_index << '\t' << msg->image_timestamp_us << '\t'
                   << std::fixed << std::setprecision(6) << sim_time_s << '\t'
                   << static_cast<int>(assignment.pred_indices.size()) << '\t'
                   << mean_error_m << '\t'
                   << max_error_m << '\t' << center_error_m << '\t'
                   << shape_mean_error_m << '\t' << pseudo_mean_error_m << '\t'
                   << pseudo_max_error_m << '\t' << pseudo_center_error_m << '\t'
                   << pseudo_shape_mean_error_m << '\t' << assignment_str << '\t'
                   << pseudo_assignment_str << '\n';

      pairs_file_.flush();
      frames_file_.flush();
    }

    frame_mean_errors_.push_back(mean_error_m);
    frame_center_errors_.push_back(center_error_m);
    frame_shape_mean_errors_.push_back(shape_mean_error_m);
    frame_pseudo_mean_errors_.push_back(pseudo_mean_error_m);
    frame_pseudo_center_errors_.push_back(pseudo_center_error_m);
    frame_pseudo_shape_mean_errors_.push_back(pseudo_shape_mean_error_m);
    for (double err : assignment.pair_error_m)
    {
      pair_errors_.push_back(err);
      if (err > 0.05)
      {
        pair_err_gt_5cm_++;
      }
      if (err > 0.10)
      {
        pair_err_gt_10cm_++;
      }
      if (err > 0.20)
      {
        pair_err_gt_20cm_++;
      }
    }

    frame_count_++;
    if (((frame_count_ % 50U) == 0U) || frame_count_ == 1U)
    {
      XR_LOG_PASS(
          "TrackerTruthCompare frames=%u sim_t=%.3f mean_err=%.4f center_err=%.4f shape_err=%.4f pseudo_mean=%.4f pseudo_center=%.4f pseudo_shape=%.4f max_err=%.4f assign=%s",
          frame_count_, sim_time_s, mean_error_m, center_error_m,
          shape_mean_error_m, pseudo_mean_error_m, pseudo_center_error_m,
          pseudo_shape_mean_error_m, max_error_m, assignment_str.c_str());
    }

    if (frame_count_ >= max_frames_)
    {
      done_.store(true, std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(file_lock_);
      CloseFilesLocked();
      WriteSummaryLocked("done");
      XR_LOG_PASS("TrackerTruthCompare done: frames=%u path=%s", frame_count_,
                  pairs_path_.c_str());
    }
  }

  struct Pose3d
  {
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  };

  static Eigen::Matrix3d AxisAngleToRotation(const double *rot)
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

  static bool ReadTopLevelRobotPose(webots::Node *node, Pose3d &pose)
  {
    if (node == nullptr)
    {
      return false;
    }

    webots::Field *translation_field = node->getField("translation");
    webots::Field *rotation_field = node->getField("rotation");
    if (translation_field == nullptr || rotation_field == nullptr)
    {
      return false;
    }

    const double *t = translation_field->getSFVec3f();
    const double *r = rotation_field->getSFRotation();
    if (t == nullptr || r == nullptr)
    {
      return false;
    }

    pose.translation = Eigen::Vector3d(t[0], t[1], t[2]);
    pose.rotation = AxisAngleToRotation(r);
    return true;
  }

  static bool ReadRelativePose(webots::Node *node, const webots::Node *from_node,
                               Pose3d &pose, bool &logged, const char *label)
  {
    if (node == nullptr || from_node == nullptr)
    {
      return false;
    }

    const double *m = node->getPose(from_node);
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
          "TrackerTruthCompare %s row_t=(%.3f, %.3f, %.3f) col_t=(%.3f, %.3f, %.3f) choose=%s",
          label, t_row.x(), t_row.y(), t_row.z(), t_col.x(), t_col.y(), t_col.z(),
          use_col_major ? "col" : "row");
      logged = true;
    }
    return true;
  }

  bool ResolveNamedRobotNode(const char *name, webots::Node *&out)
  {
    if (out != nullptr)
    {
      return true;
    }
    if (supervisor_ == nullptr)
    {
      return false;
    }

    webots::Node *root = supervisor_->getRoot();
    if (root == nullptr)
    {
      return false;
    }

    webots::Field *children = root->getField("children");
    if (children == nullptr)
    {
      return false;
    }

    for (int i = 0; i < children->getCount(); ++i)
    {
      webots::Node *node = children->getMFNode(i);
      if (node == nullptr)
      {
        continue;
      }
      webots::Field *name_field = node->getField("name");
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

  bool ResolveNodes()
  {
    bool all_armors_resolved = true;
    for (auto *node : armor_nodes_)
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
      return false;
    }
    if (!ResolveNamedRobotNode("target", target_robot_node_))
    {
      return false;
    }
    if (!ResolveNamedRobotNode("self", self_robot_node_))
    {
      return false;
    }

    webots::Node *root = supervisor_->getRoot();
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
        return false;
      }
    }

    XR_LOG_PASS("TrackerTruthCompare resolved target/self/camera/armor nodes");
    return true;
  }

  static webots::Node *FindNamedNodeRecursiveImpl(
      webots::Node *node, const char *wanted_name,
      std::unordered_set<const webots::Node *> &visited)
  {
    if (node == nullptr || wanted_name == nullptr)
    {
      return nullptr;
    }

    webots::Field *name_field = node->getField("name");
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

    auto search_field = [&](webots::Field *field) -> webots::Node *
    {
      if (field == nullptr)
      {
        return nullptr;
      }
      if (field->getType() == webots::Field::SF_NODE)
      {
        if (webots::Node *found =
                FindNamedNodeRecursiveImpl(field->getSFNode(), wanted_name, visited))
        {
          return found;
        }
      }
      else if (field->getType() == webots::Field::MF_NODE)
      {
        for (int i = 0; i < field->getCount(); ++i)
        {
          if (webots::Node *found =
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
      if (webots::Node *found = search_field(node->getFieldByIndex(i)))
      {
        return found;
      }
    }
    if (node->isProto())
    {
      for (int i = 0; i < node->getNumberOfBaseNodeFields(); ++i)
      {
        if (webots::Node *found = search_field(node->getBaseNodeFieldByIndex(i)))
        {
          return found;
        }
      }
    }
    return nullptr;
  }

  static webots::Node *FindNamedNodeRecursive(webots::Node *node,
                                              const char *wanted_name)
  {
    std::unordered_set<const webots::Node *> visited;
    return FindNamedNodeRecursiveImpl(node, wanted_name, visited);
  }

  bool ReadTruthWorld(std::array<Eigen::Vector3d, 4> &gt_world)
  {
    if (target_spin_node_ == nullptr || target_robot_node_ == nullptr)
    {
      return false;
    }

    Pose3d target_root_pose;
    Pose3d target_spin_local_pose;
    if (!ReadTopLevelRobotPose(target_robot_node_, target_root_pose) ||
        !ReadRelativePose(target_spin_node_, target_robot_node_, target_spin_local_pose,
                          target_spin_pose_logged_, "target_spin_local"))
    {
      return false;
    }

    const Eigen::Vector3d p_target_spin_webots = target_root_pose.translation +
                                                 target_root_pose.rotation *
                                                     target_spin_local_pose.translation;
    const Eigen::Matrix3d r_target_spin_webots =
        target_root_pose.rotation * target_spin_local_pose.rotation;

    for (int i = 0; i < 4; ++i)
    {
      const Eigen::Vector3d local(kArmorLocalPosSpin[i][0], kArmorLocalPosSpin[i][1],
                                  kArmorLocalPosSpin[i][2]);
      const Eigen::Vector3d armor_webots =
          p_target_spin_webots + r_target_spin_webots * local;
      gt_world[i] = armor_webots;
      if (!IsFiniteVec(gt_world[i]))
      {
        return false;
      }
    }
    return true;
  }

  bool ReadTruthCam(std::array<Eigen::Vector3d, 4> &gt_cam,
                    std::array<bool, 4> &gt_visible)
  {
    if (!ResolveNodes())
    {
      return false;
    }
    const Eigen::Matrix3d &camera_node_to_optical =
        WebotsCameraNodeToOpticalRotation();

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

      gt_cam[i] = camera_node_to_optical * armor_in_camera_node.translation;
      gt_visible[i] = std::isfinite(gt_cam[i].x()) && std::isfinite(gt_cam[i].y()) &&
                      std::isfinite(gt_cam[i].z()) && gt_cam[i].z() > 1e-6;
      if (!IsFiniteVec(gt_cam[i]))
      {
        return false;
      }
    }
    return true;
  }

  static AssignmentResult SolveBestAssignment(
      const std::array<Eigen::Vector3d, 4> &gt_cam,
      const std::array<Eigen::Vector3d, 4> &ekf_cam,
      const std::vector<int> &pred_indices)
  {
    AssignmentResult best{};
    if (pred_indices.empty())
    {
      return best;
    }

    best.pred_indices = pred_indices;
    const std::size_t pred_count = pred_indices.size();
    std::array<int, 4> gt_order = {0, 1, 2, 3};
    do
    {
      std::vector<int> current_gt(pred_count, -1);
      std::vector<double> current_pair_errors(pred_count, 0.0);
      double total_error_m = 0.0;
      bool valid = true;
      for (std::size_t pred_slot = 0; pred_slot < pred_count; ++pred_slot)
      {
        const int pred_index = pred_indices[pred_slot];
        const int gt_index = gt_order[pred_slot];
        if (!IsFiniteVec(ekf_cam[pred_index]) || !IsFiniteVec(gt_cam[gt_index]))
        {
          valid = false;
          break;
        }
        const double err = DistanceMeters(ekf_cam[pred_index], gt_cam[gt_index]);
        current_gt[pred_slot] = gt_index;
        current_pair_errors[pred_slot] = err;
        total_error_m += err;
        if (best.valid && total_error_m >= best.total_error_m)
        {
          valid = false;
          break;
        }
      }
      if (!valid)
      {
        continue;
      }
      best.valid = true;
      best.total_error_m = total_error_m;
      best.gt_index_for_pred = current_gt;
      best.pair_error_m = current_pair_errors;
    } while (std::next_permutation(gt_order.begin(), gt_order.end()));

    return best;
  }

  static std::string AssignmentString(const AssignmentResult &assignment)
  {
    if (!assignment.valid)
    {
      return "invalid";
    }
    std::ostringstream ss;
    for (std::size_t i = 0; i < assignment.pred_indices.size(); ++i)
    {
      if (i > 0)
      {
        ss << ',';
      }
      ss << "A" << assignment.pred_indices[i] << "->"
         << kTruthLabels[assignment.gt_index_for_pred[i]];
    }
    return ss.str();
  }

  void OpenFilesLocked()
  {
    if (!pairs_file_.is_open())
    {
      pairs_file_.open(pairs_path_, std::ios::out | std::ios::trunc);
      if (pairs_file_)
      {
        pairs_file_
            << "frame\timage_ts_us\tsim_time_s\tekf_label\tgt_label\terr_m\tgt_visible\tekf_visible\t"
               "gt_world_x\tgt_world_y\tgt_world_z\tgt_cam_x\tgt_cam_y\tgt_cam_z\t"
               "ekf_cam_x\tekf_cam_y\tekf_cam_z\tekf_world_x\tekf_world_y\tekf_world_z\n";
      }
    }

    if (!frames_file_.is_open())
    {
      frames_file_.open(frames_path_, std::ios::out | std::ios::trunc);
      if (frames_file_)
      {
        frames_file_
            << "frame\timage_ts_us\tsim_time_s\tekf_count\treal_mean_err_m\treal_max_err_m\treal_center_err_m\treal_shape_mean_err_m\tpseudo_mean_err_m\tpseudo_max_err_m\tpseudo_center_err_m\tpseudo_shape_mean_err_m\treal_assignment\tpseudo_assignment\n";
      }
    }
  }

  void CloseFilesLocked()
  {
    if (pairs_file_.is_open())
    {
      pairs_file_.flush();
      pairs_file_.close();
    }
    if (frames_file_.is_open())
    {
      frames_file_.flush();
      frames_file_.close();
    }
  }

  static double MeanOf(const std::vector<double> &values)
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

  void WriteSummaryLocked(const char *status)
  {
    if (summary_written_ && done_.load(std::memory_order_relaxed))
    {
      return;
    }

    std::ofstream summary(summary_path_, std::ios::out | std::ios::trunc);
    summary << "status=" << status << '\n';
    summary << "pairs=" << pairs_path_ << '\n';
    summary << "frames=" << frames_path_ << '\n';
    summary << "frame_count=" << frame_count_ << '\n';
    summary << "max_frames=" << max_frames_ << '\n';
    summary << "skipped_no_supervisor=" << skipped_no_supervisor_ << '\n';
    summary << "skipped_no_nodes=" << skipped_no_nodes_ << '\n';
    summary << "skipped_no_truth_frame=" << skipped_no_truth_frame_ << '\n';
    summary << "skipped_bad_truth=" << skipped_bad_truth_ << '\n';
    summary << std::fixed << std::setprecision(6);
    summary << "frame_mean_err_mean=" << MeanOf(frame_mean_errors_) << '\n';
    summary << "frame_mean_err_p50=" << Percentile(frame_mean_errors_, 0.50) << '\n';
    summary << "frame_mean_err_p95=" << Percentile(frame_mean_errors_, 0.95) << '\n';
    summary << "frame_mean_err_max="
            << (frame_mean_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_mean_errors_.begin(),
                                        frame_mean_errors_.end()))
            << '\n';
    summary << "frame_center_err_mean=" << MeanOf(frame_center_errors_) << '\n';
    summary << "frame_center_err_p50=" << Percentile(frame_center_errors_, 0.50) << '\n';
    summary << "frame_center_err_p95=" << Percentile(frame_center_errors_, 0.95) << '\n';
    summary << "frame_center_err_max="
            << (frame_center_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_center_errors_.begin(),
                                        frame_center_errors_.end()))
            << '\n';
    summary << "frame_shape_err_mean=" << MeanOf(frame_shape_mean_errors_) << '\n';
    summary << "frame_shape_err_p50=" << Percentile(frame_shape_mean_errors_, 0.50)
            << '\n';
    summary << "frame_shape_err_p95=" << Percentile(frame_shape_mean_errors_, 0.95)
            << '\n';
    summary << "frame_shape_err_max="
            << (frame_shape_mean_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_shape_mean_errors_.begin(),
                                        frame_shape_mean_errors_.end()))
            << '\n';
    summary << "frame_pseudo_mean_err_mean=" << MeanOf(frame_pseudo_mean_errors_) << '\n';
    summary << "frame_pseudo_mean_err_p50="
            << Percentile(frame_pseudo_mean_errors_, 0.50) << '\n';
    summary << "frame_pseudo_mean_err_p95="
            << Percentile(frame_pseudo_mean_errors_, 0.95) << '\n';
    summary << "frame_pseudo_mean_err_max="
            << (frame_pseudo_mean_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_pseudo_mean_errors_.begin(),
                                        frame_pseudo_mean_errors_.end()))
            << '\n';
    summary << "frame_pseudo_center_err_mean=" << MeanOf(frame_pseudo_center_errors_)
            << '\n';
    summary << "frame_pseudo_center_err_p50="
            << Percentile(frame_pseudo_center_errors_, 0.50) << '\n';
    summary << "frame_pseudo_center_err_p95="
            << Percentile(frame_pseudo_center_errors_, 0.95) << '\n';
    summary << "frame_pseudo_center_err_max="
            << (frame_pseudo_center_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_pseudo_center_errors_.begin(),
                                        frame_pseudo_center_errors_.end()))
            << '\n';
    summary << "frame_pseudo_shape_err_mean="
            << MeanOf(frame_pseudo_shape_mean_errors_) << '\n';
    summary << "frame_pseudo_shape_err_p50="
            << Percentile(frame_pseudo_shape_mean_errors_, 0.50) << '\n';
    summary << "frame_pseudo_shape_err_p95="
            << Percentile(frame_pseudo_shape_mean_errors_, 0.95) << '\n';
    summary << "frame_pseudo_shape_err_max="
            << (frame_pseudo_shape_mean_errors_.empty()
                    ? 0.0
                    : *std::max_element(frame_pseudo_shape_mean_errors_.begin(),
                                        frame_pseudo_shape_mean_errors_.end()))
            << '\n';
    summary << "pair_err_mean=" << MeanOf(pair_errors_) << '\n';
    summary << "pair_err_p50=" << Percentile(pair_errors_, 0.50) << '\n';
    summary << "pair_err_p95=" << Percentile(pair_errors_, 0.95) << '\n';
    summary << "pair_err_max="
            << (pair_errors_.empty()
                    ? 0.0
                    : *std::max_element(pair_errors_.begin(), pair_errors_.end()))
            << '\n';
    summary << "pair_err_gt_0.05=" << pair_err_gt_5cm_ << '\n';
    summary << "pair_err_gt_0.10=" << pair_err_gt_10cm_ << '\n';
    summary << "pair_err_gt_0.20=" << pair_err_gt_20cm_ << '\n';
    summary_written_ = true;
  }

 private:
  webots::Supervisor *supervisor_ = nullptr;
  webots::Node *target_spin_node_ = nullptr;
  webots::Node *camera_node_ = nullptr;
  webots::Node *target_robot_node_ = nullptr;
  webots::Node *self_robot_node_ = nullptr;
  std::array<webots::Node *, 4> armor_nodes_ = {nullptr, nullptr, nullptr, nullptr};
  const LibXR::Transform<double> gimbal_to_camera_transform_static_;

  std::mutex state_lock_{};
  std::deque<TruthFrameSnapshot> truth_frame_cache_{};

  std::mutex file_lock_{};
  std::ofstream pairs_file_{};
  std::ofstream frames_file_{};
  std::string pairs_path_;
  std::string frames_path_;
  std::string summary_path_;
  uint32_t max_frames_{1000};
  std::size_t truth_frame_cache_size_{kTruthFrameCacheSize};
  bool summary_written_{false};

  std::atomic<bool> done_{false};
  uint32_t frame_count_{0};
  uint32_t skipped_no_supervisor_{0};
  uint32_t skipped_no_nodes_{0};
  uint32_t skipped_no_truth_frame_{0};
  uint32_t skipped_bad_truth_{0};
  uint32_t pair_err_gt_5cm_{0};
  uint32_t pair_err_gt_10cm_{0};
  uint32_t pair_err_gt_20cm_{0};
  bool target_spin_pose_logged_{false};
  std::array<bool, 4> armor_pose_logged_ = {false, false, false, false};
  std::vector<double> frame_mean_errors_{};
  std::vector<double> frame_center_errors_{};
  std::vector<double> frame_shape_mean_errors_{};
  std::vector<double> frame_pseudo_mean_errors_{};
  std::vector<double> frame_pseudo_center_errors_{};
  std::vector<double> frame_pseudo_shape_mean_errors_{};
  std::vector<double> pair_errors_{};
};
