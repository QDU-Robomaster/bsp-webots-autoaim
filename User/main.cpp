// Webots 入口：外部控制器连接仿真，按 WEBOTS_SIM_FLOW_RATE 设定仿真速度，装配自瞄链路，
// 并每秒把各层计数写进运行摘要（供启动器判定，不靠解析日志）。
//
// Webots entry: an extern controller attached to the simulation; WEBOTS_SIM_FLOW_RATE sets
// the simulation speed. It assembles the auto-aim chain and writes the per-stage counts
// to a run summary every second, which the launcher judges instead of parsing logs.

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <thread>
#include <webots/Supervisor.hpp>

#include "AutoAimTypes.hpp"
#include "WebotsRefereeTypes.hpp"
#include "libxr.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "xrobot_main.hpp"

namespace
{
/**
 * @brief 运行摘要：各层收到的帧数与结果计数、错误日志数，每秒写成 JSON。
 *        Run summary: frames and result counts per stage and the number of error logs,
 *        written as JSON every second.
 *
 * 在模块构造前按类型创建各层 Topic 并订阅；模块随后创建同名 Topic 时得到的是同一个。
 * The stage Topics are created with their types and subscribed before the Modules are
 * constructed; the Modules then get the same Topics when they create them.
 */
class RunSummary
{
 public:
  RunSummary(const std::string& camera, std::string path, const std::string& shots_path)
      : path_(std::move(path))
  {
    Count<AutoAim::SyncedFrame>(camera + "_synced",
                                [](RunSummary* s, const AutoAim::SyncedFrame& f)
                                {
                                  s->synced_.fetch_add(1);
                                  s->sim_us_.store(static_cast<uint64_t>(f.imu.timestamp_us));
                                });
    Count<AutoAim::DetectedFrame>(camera + "_detected",
                                  [](RunSummary* s, const AutoAim::DetectedFrame& f)
                                  {
                                    s->detected_.fetch_add(1);
                                    s->armors_.fetch_add(f.armors.size());
                                  });
    Count<AutoAim::TrackedFrame>(camera + "_tracked",
                                 [](RunSummary* s, const AutoAim::TrackedFrame& f)
                                 {
                                   s->tracked_.fetch_add(1);
                                   s->tracking_.fetch_add(f.target.tracking ? 1 : 0);
                                 });
    Count<AutoAim::AimedFrame>(camera + "_aimed",
                               [](RunSummary* s, const AutoAim::AimedFrame& f)
                               {
                                 s->aimed_.fetch_add(1);
                                 s->control_.fetch_add(f.aim.control ? 1 : 0);
                                 s->fire_.fetch_add(f.aim.fire ? 1 : 0);
                               });
    // 每发弹丸（WebotsReferee 的出弹事件）；设置了路径时逐发写 TSV，供离线按真值判命中。
    // Every shot (WebotsReferee's shot events); with a path set each one is written to a
    // TSV for offline hit judgement against the truth.
    if (!shots_path.empty())
    {
      shots_file_ = std::fopen(shots_path.c_str(), "w");
      if (shots_file_ != nullptr)
      {
        std::fputs("shot_id\trequest_time_us\tfire_time_us\tbullet_speed\n", shots_file_);
      }
    }
    auto on_shot = LibXR::Topic::Callback::Create(
        [](bool, RunSummary* s, const WebotsRefereeTypes::WebotsLauncherShotEvent& e)
        {
          s->shots_.fetch_add(1);
          if (s->shots_file_ != nullptr)
          {
            std::fprintf(s->shots_file_, "%llu\t%llu\t%llu\t%.3f\n",
                         static_cast<unsigned long long>(e.shot_id),
                         static_cast<unsigned long long>(e.request_time_us),
                         static_cast<unsigned long long>(e.fire_time_us),
                         static_cast<double>(e.bullet_speed));
            std::fflush(s->shots_file_);
          }
        },
        this);
    LibXR::Topic::CreateTopic<WebotsRefereeTypes::WebotsLauncherShotEvent>(
        "shot_event", &launcher_domain_, true)
        .RegisterCallback(on_shot);
    auto on_log = LibXR::Topic::Callback::Create(
        [](bool, RunSummary* s, const LibXR::ConstRawData& data)
        {
          const auto* log = static_cast<const LibXR::LogData*>(data.addr_);
          if (log != nullptr && log->level == LibXR::LogLevel::XR_LOG_LEVEL_ERROR)
          {
            s->errors_.fetch_add(1);
          }
        },
        this);
    LibXR::Topic(LibXR::Topic::Find("/xr/log")).RegisterCallback(on_log);
    writer_ = std::thread(
        [this]()
        {
          while (true)
          {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            Write();
          }
        });
    writer_.detach();
  }

 private:
  template <typename Frame>
  void Count(const std::string& name, void (*fun)(RunSummary*, const Frame&))
  {
    struct Binding
    {
      RunSummary* self;
      void (*fun)(RunSummary*, const Frame&);
    };
    auto* binding = new Binding{this, fun};  // 与进程同寿命 / Lives as long as the process
    auto callback = LibXR::Topic::Callback::Create(
        [](bool, Binding* b, const Frame* frame) { b->fun(b->self, *frame); }, binding);
    LibXR::Topic::CreateTopic<const Frame*>(name.c_str()).RegisterCallback(callback);
  }

  void Write()
  {
    const std::string tmp = path_ + ".tmp";
    std::FILE* f = std::fopen(tmp.c_str(), "w");
    if (f == nullptr)
    {
      return;
    }
    std::fprintf(f,
                 "{\"sim_time_s\": %.3f, \"synced\": %u, \"detected\": %u, \"armors\": %u, "
                 "\"tracked\": %u, \"tracking\": %u, \"aimed\": %u, \"control\": %u, "
                 "\"fire\": %u, \"shots\": %u, \"errors\": %u}\n",
                 static_cast<double>(sim_us_.load()) * 1e-6, synced_.load(), detected_.load(),
                 armors_.load(), tracked_.load(), tracking_.load(), aimed_.load(),
                 control_.load(), fire_.load(), shots_.load(), errors_.load());
    std::fclose(f);
    std::rename(tmp.c_str(), path_.c_str());
  }

  const std::string path_;
  std::atomic<uint64_t> sim_us_{0};
  std::atomic<uint32_t> synced_{0}, detected_{0}, armors_{0}, tracked_{0}, tracking_{0},
      aimed_{0}, control_{0}, fire_{0}, shots_{0}, errors_{0};
  LibXR::Topic::Domain launcher_domain_{"webots_launcher"};
  std::FILE* shots_file_ = nullptr;
  std::thread writer_;
};
}  // namespace

int main(int, char**)
{
  double sim_flow_rate = 1.0;
  if (const char* value = std::getenv("WEBOTS_SIM_FLOW_RATE"); value != nullptr)
  {
    char* end = nullptr;
    errno = 0;
    sim_flow_rate = std::strtod(value, &end);
    if (errno == ERANGE || end == value || *end != '\0' || !std::isfinite(sim_flow_rate) ||
        sim_flow_rate <= 0.0)
    {
      std::fprintf(stderr, "Invalid WEBOTS_SIM_FLOW_RATE: expected a finite positive number\n");
      return 2;
    }
  }
  std::printf("Webots sim_flow_rate=%.9g\n", sim_flow_rate);
  std::fflush(stdout);

  webots::Supervisor supervisor;
  const double basic_time_step_ms = supervisor.getBasicTimeStep();
  const double poll_period_ms = std::round(basic_time_step_ms / sim_flow_rate);
  const double step_interval_ns = std::round(basic_time_step_ms * 1000000.0 / sim_flow_rate);
  // LibXR 取整与转换前先检查范围 / Check the ranges used by LibXR before its conversions.
  if (!std::isfinite(poll_period_ms) || !std::isfinite(step_interval_ns) ||
      poll_period_ms > static_cast<double>(std::numeric_limits<uint32_t>::max()) ||
      step_interval_ns >= static_cast<double>(std::numeric_limits<long long>::max()))
  {
    std::fprintf(stderr, "Invalid WEBOTS_SIM_FLOW_RATE: platform timing interval out of range\n");
    return 2;
  }
  LibXR::PlatformInit(&supervisor, 2, 65536, sim_flow_rate);

  const char* summary_path = std::getenv("XR_RUN_SUMMARY");
  const char* shots_path = std::getenv("XR_SHOTS_TSV");
  static RunSummary summary("gimbal",
                            summary_path != nullptr ? summary_path : "run_summary.json",
                            shots_path != nullptr ? shots_path : "");
  XROBOT_MAIN();
}
