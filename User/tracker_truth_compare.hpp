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
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include "ArmorTracker.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "transform.hpp"

class TrackerTruthCompare
{
 public:
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
    LibXR::Topic::Domain gimbal_domain("gimbal");
    auto gimbal_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("rotation", UINT32_MAX, &gimbal_domain));
    auto gimbal_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerTruthCompare *self, LibXR::RawData &data)
        {
          auto *msg = reinterpret_cast<LibXR::Quaternion<float> *>(data.addr_);
          self->GimbalRotationCallback(msg);
        },
        this);
    gimbal_topic.RegisterCallback(gimbal_cb);

    LibXR::Topic::Domain tracker_domain("tracker");
    auto ekf_topic =
        LibXR::Topic(LibXR::Topic::WaitTopic("ekf_points", UINT32_MAX, &tracker_domain));
    auto ekf_cb = LibXR::Topic::Callback::Create(
        [](bool, TrackerTruthCompare *self, LibXR::RawData &data)
        {
          auto *msg = reinterpret_cast<ArmorTracker::EkfPointsMsg *>(data.addr_);
          self->EkfPointsCallback(msg);
        },
        this);
    ekf_topic.RegisterCallback(ekf_cb);

    XR_LOG_PASS(
        "TrackerTruthCompare subscribed: gimbal/rotation + tracker/ekf_points -> %s",
        pairs_path_.c_str());
  }

  bool Done() const { return done_.load(std::memory_order_relaxed); }

 private:
  static constexpr std::array<const char *, 4> kTruthLabels = {
      "front", "right", "back", "left"};
  static constexpr std::array<std::array<double, 3>, 4> kArmorLocalPosSpin = {
      std::array<double, 3>{0.0, 0.205, -0.06},
      std::array<double, 3>{0.205, 0.0, -0.11},
      std::array<double, 3>{0.0, -0.205, -0.06},
      std::array<double, 3>{-0.205, 0.0, -0.11}};

  struct AssignmentResult
  {
    std::array<int, 4> gt_index_for_ekf = {0, 1, 2, 3};
    std::array<double, 4> pair_error_m = {0.0, 0.0, 0.0, 0.0};
    double total_error_m = 0.0;
  };

  static bool IsFiniteVec(const Eigen::Vector3d &v)
  {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
  }

  static double DistanceMeters(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
  {
    return (a - b).norm();
  }

  void GimbalRotationCallback(LibXR::Quaternion<float> *msg)
  {
    if (msg == nullptr)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(state_lock_);
    gimbal_rotation_ =
        LibXR::Quaternion<double>(msg->w(), msg->x(), msg->y(), msg->z());
    has_gimbal_rotation_ = true;
  }

  void EkfPointsCallback(ArmorTracker::EkfPointsMsg *msg)
  {
    if (msg == nullptr || Done())
    {
      return;
    }

    LibXR::Quaternion<double> gimbal_rotation;
    {
      std::lock_guard<std::mutex> lock(state_lock_);
      if (!has_gimbal_rotation_)
      {
        skipped_no_rotation_++;
        return;
      }
      gimbal_rotation = gimbal_rotation_;
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

    std::array<Eigen::Vector3d, 4> gt_world{};
    if (!ReadTruthWorld(gt_world))
    {
      skipped_bad_truth_++;
      return;
    }

    if (msg->count == 0)
    {
      skipped_bad_truth_++;
      return;
    }

    const LibXR::Transform<double> t_wg(gimbal_rotation, {0.0, 0.0, 0.0});
    const LibXR::Transform<double> t_wc =
        t_wg + gimbal_to_camera_transform_static_;
    const auto r_wc = t_wc.rotation.ToRotationMatrix();
    const Eigen::Vector3d twc(t_wc.translation.x(), t_wc.translation.y(),
                              t_wc.translation.z());

    std::array<Eigen::Vector3d, 4> gt_cam{};
    std::array<bool, 4> gt_visible{};
    if (!ReadTruthCam(gt_cam, gt_visible))
    {
      skipped_bad_truth_++;
      return;
    }

    std::array<Eigen::Vector3d, 4> ekf_cam{};
    std::array<Eigen::Vector3d, 4> ekf_world{};
    std::array<bool, 4> ekf_visible{};
    for (int i = 0; i < 4; ++i)
    {
      ekf_cam[i] = Eigen::Vector3d(msg->armors_cam[i].x(), msg->armors_cam[i].y(),
                                   msg->armors_cam[i].z());
      ekf_world[i] = r_wc * ekf_cam[i] + twc;
      ekf_visible[i] = msg->valid[i + 1];
    }

    const AssignmentResult assignment = SolveBestAssignment(gt_cam, ekf_cam);
    const double sim_time_s = supervisor_->getTime();
    const uint32_t frame_index = frame_count_;
    const double mean_error_m = assignment.total_error_m / 4.0;
    const double max_error_m =
        *std::max_element(assignment.pair_error_m.begin(), assignment.pair_error_m.end());
    const std::string assignment_str = AssignmentString(assignment);

    {
      std::lock_guard<std::mutex> lock(file_lock_);
      OpenFilesLocked();
      for (int ekf_index = 0; ekf_index < 4; ++ekf_index)
      {
        const int gt_index = assignment.gt_index_for_ekf[ekf_index];
        pairs_file_ << frame_index << '\t' << std::fixed << std::setprecision(6)
                    << sim_time_s << '\t' << "A" << ekf_index << '\t'
                    << kTruthLabels[gt_index] << '\t'
                    << assignment.pair_error_m[ekf_index] << '\t'
                    << (gt_visible[gt_index] ? 1 : 0) << '\t'
                    << (ekf_visible[ekf_index] ? 1 : 0) << '\t'
                    << gt_world[gt_index].x() << '\t' << gt_world[gt_index].y() << '\t'
                    << gt_world[gt_index].z() << '\t' << gt_cam[gt_index].x() << '\t'
                    << gt_cam[gt_index].y() << '\t' << gt_cam[gt_index].z() << '\t'
                    << ekf_cam[ekf_index].x() << '\t' << ekf_cam[ekf_index].y() << '\t'
                    << ekf_cam[ekf_index].z() << '\t' << ekf_world[ekf_index].x() << '\t'
                    << ekf_world[ekf_index].y() << '\t' << ekf_world[ekf_index].z()
                    << '\n';
      }

      frames_file_ << frame_index << '\t' << std::fixed << std::setprecision(6)
                   << sim_time_s << '\t' << static_cast<int>(msg->count) << '\t'
                   << mean_error_m << '\t' << max_error_m << '\t'
                   << assignment_str << '\n';

      pairs_file_.flush();
      frames_file_.flush();
    }

    frame_mean_errors_.push_back(mean_error_m);
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
          "TrackerTruthCompare frames=%u sim_t=%.3f mean_err=%.4f max_err=%.4f assign=%s",
          frame_count_, sim_time_s, mean_error_m, max_error_m, assignment_str.c_str());
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
    if (target_spin_node_ != nullptr && camera_node_ != nullptr &&
        target_robot_node_ != nullptr && self_robot_node_ != nullptr)
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

    XR_LOG_PASS("TrackerTruthCompare resolved target/self/camera nodes");
    return true;
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
    if (target_spin_node_ == nullptr || camera_node_ == nullptr)
    {
      return false;
    }

    Pose3d target_spin_in_camera_node;
    if (!ReadRelativePose(target_spin_node_, camera_node_, target_spin_in_camera_node,
                          target_spin_pose_logged_, "target_spin_in_camera"))
    {
      return false;
    }

    static const Eigen::Matrix3d kCameraNodeToOptical =
        (Eigen::Matrix3d() << 0.0, 1.0, 0.0,
         0.0, 0.0, -1.0,
         1.0, 0.0, 0.0)
            .finished();

    for (int i = 0; i < 4; ++i)
    {
      const Eigen::Vector3d local(kArmorLocalPosSpin[i][0], kArmorLocalPosSpin[i][1],
                                  kArmorLocalPosSpin[i][2]);
      const Eigen::Vector3d armor_camera_node =
          target_spin_in_camera_node.translation +
          target_spin_in_camera_node.rotation * local;
      gt_cam[i] = kCameraNodeToOptical * armor_camera_node;
      gt_visible[i] = gt_cam[i].z() > 1e-6;
      if (!IsFiniteVec(gt_cam[i]))
      {
        return false;
      }
    }
    return true;
  }

  static AssignmentResult SolveBestAssignment(
      const std::array<Eigen::Vector3d, 4> &gt_cam,
      const std::array<Eigen::Vector3d, 4> &ekf_cam)
  {
    AssignmentResult best{};
    best.total_error_m = std::numeric_limits<double>::infinity();

    std::array<int, 4> perm = {0, 1, 2, 3};
    do
    {
      double total = 0.0;
      std::array<double, 4> pair_errors{};
      bool valid = true;
      for (int ekf_index = 0; ekf_index < 4; ++ekf_index)
      {
        if (!IsFiniteVec(ekf_cam[ekf_index]) || !IsFiniteVec(gt_cam[perm[ekf_index]]))
        {
          valid = false;
          break;
        }
        const double err = DistanceMeters(ekf_cam[ekf_index], gt_cam[perm[ekf_index]]);
        pair_errors[ekf_index] = err;
        total += err;
      }
      if (valid && total < best.total_error_m)
      {
        best.total_error_m = total;
        best.gt_index_for_ekf = perm;
        best.pair_error_m = pair_errors;
      }
    } while (std::next_permutation(perm.begin(), perm.end()));

    return best;
  }

  static std::string AssignmentString(const AssignmentResult &assignment)
  {
    std::ostringstream ss;
    for (int i = 0; i < 4; ++i)
    {
      if (i > 0)
      {
        ss << ',';
      }
      ss << "A" << i << "->" << kTruthLabels[assignment.gt_index_for_ekf[i]];
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
            << "frame\tsim_time_s\tekf_label\tgt_label\terr_m\tgt_visible\tekf_visible\t"
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
            << "frame\tsim_time_s\tekf_count\tmean_err_m\tmax_err_m\tassignment\n";
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
    summary << "skipped_no_rotation=" << skipped_no_rotation_ << '\n';
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
  const LibXR::Transform<double> gimbal_to_camera_transform_static_;

  std::mutex state_lock_{};
  LibXR::Quaternion<double> gimbal_rotation_{1.0, 0.0, 0.0, 0.0};
  bool has_gimbal_rotation_{false};

  std::mutex file_lock_{};
  std::ofstream pairs_file_{};
  std::ofstream frames_file_{};
  std::string pairs_path_;
  std::string frames_path_;
  std::string summary_path_;
  uint32_t max_frames_{1000};
  bool summary_written_{false};

  std::atomic<bool> done_{false};
  uint32_t frame_count_{0};
  uint32_t skipped_no_supervisor_{0};
  uint32_t skipped_no_nodes_{0};
  uint32_t skipped_no_rotation_{0};
  uint32_t skipped_bad_truth_{0};
  uint32_t pair_err_gt_5cm_{0};
  uint32_t pair_err_gt_10cm_{0};
  uint32_t pair_err_gt_20cm_{0};
  bool target_spin_pose_logged_{false};
  bool camera_pose_logged_{false};
  std::vector<double> frame_mean_errors_{};
  std::vector<double> pair_errors_{};
};
