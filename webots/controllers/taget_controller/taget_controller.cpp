// File: target_controller.cpp
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/LED.hpp>

#include <cmath>
#include <random>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <array>

using namespace webots;

// ========================== 配置区域（全部参数在这里改） ==========================
namespace Config {
  // 全局开关
  inline bool BIG_ARMOR = false; // true: led1/2 亮, false: led3/4 亮

  // 设备名（与 world 中 device 名一致）
  inline const std::array<const char*, 6> MOTOR_NAMES = {
    "target_motor_z",
    "target_motor_x",
    "target_motor_y",
    "target_motor_pitch",
    "target_motor_yaw",
    "target_motor_roll",
  };

  // LED 名称（如无需改名可不动）
  inline const char* LED1 = "led1";
  inline const char* LED2 = "led2";
  inline const char* LED3 = "led3";
  inline const char* LED4 = "led4";

  // 随机平滑直线电机驱动器：统一默认参数
  struct DriverDefaults {
    double speedScale       = 0.10;  // 统一放慢倍数：1.0/0.5/0.33/0.1 ...
    double baseVmaxRatio    = 0.70;  // 速度上限 = baseVmaxRatio * motor->getMaxVelocity()
    double baseAmax         = 5.00;  // 最大加速度 m/s^2
    double baseRetarget     = 0.60;  // 换速周期 s（越小越频繁）
    double defaultMin       = -0.25; // 未提供 min/max 时的默认行程
    double defaultMax       =  0.25;
    double edgeBufferPct    = 0.02;  // 端点缓冲比例
    double edgeBufferAbsMin = 0.003; // 端点缓冲绝对下限 m
    unsigned baseSeed       = 12345; // 随机种子基数（每个电机会在此基础上+index）
    double positionForce    = 1000.0;// 位置模式使用的力上限
  };
  inline const DriverDefaults DRIVER_DEFAULTS{};

  // 针对特定电机的默认行程覆盖（仅在设备缺省 min/max 情况下生效）
  // 索引对应 MOTOR_NAMES：3=pitch, 4=yaw, 5=roll
  struct RangeOverride {
    bool enable   = true;
    double minPos = -0.75;
    double maxPos =  0.75;
  };
  inline const RangeOverride ORIENT_RANGE_OVERRIDE{true, -0.75, 0.75};
}
// ==============================================================================

static inline double clampd(double x, double lo, double hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// 随机平滑直线电机驱动器（不依赖 PositionSensor）
class RandomLinearDriver {
public:
  struct Params {
    double speedScale       = 1.0;   // 统一放慢倍数
    double baseVmaxRatio    = 0.7;   // 速度上限 = baseVmaxRatio * motor->getMaxVelocity()
    double baseAmax         = 5.0;   // 最大加速度 m/s^2
    double baseRetarget     = 0.6;   // 换速周期 s（越小越频繁）
    double defaultMin       = -0.25; // 未提供 min/max 时的默认行程
    double defaultMax       =  0.25;
    double edgeBufferPct    = 0.02;  // 端点缓冲比例
    double edgeBufferAbsMin = 0.003; // 端点缓冲绝对下限 m
    unsigned seed           = 12345; // 随机种子
    double positionForce    = 1000.0;// 位置模式使用的力上限
  };

  RandomLinearDriver(Motor* motor, const Params& p)
  : m_motor(motor), m_params(p), m_rng(p.seed), m_uni(-1.0, 1.0) {
    if (!m_motor) throw std::runtime_error("Motor is null");

    // 行程（若无边界则用默认）
    m_min = m_motor->getMinPosition();
    m_max = m_motor->getMaxPosition();
    const bool bounded = std::isfinite(m_min) && std::isfinite(m_max) && m_max > m_min;
    if (!bounded) {
      m_min = m_params.defaultMin;
      m_max = m_params.defaultMax;
      std::cerr << "[WARN] Motor has no min/max, using default [" << m_min << ", " << m_max << "]\n";
    }
    m_span   = m_max - m_min;
    m_buffer = std::max(m_span * m_params.edgeBufferPct, m_params.edgeBufferAbsMin);

    // 速度/加速度/换速周期
    double vmax_hw = m_motor->getMaxVelocity();
    if (!std::isfinite(vmax_hw) || vmax_hw <= 0.0) vmax_hw = 1.0;

    const double S = clampd(m_params.speedScale, 0.001, 10.0);
    m_V_MAX          = vmax_hw * m_params.baseVmaxRatio * S;
    m_A_MAX          = m_params.baseAmax * S;
    m_retargetPeriod = std::max(0.05, m_params.baseRetarget / S); // 下限避免过于频繁

    // 初始状态：从中心起步
    m_x = 0.5 * (m_min + m_max);
    m_motor->setForce(m_params.positionForce);
    m_motor->setVelocity(vmax_hw);   // 位置模式下给足内部允许速度
    m_motor->setPosition(m_x);

    // 初始随机目标速度
    m_v = 0.0;
    m_v_goal = m_V_MAX * m_uni(m_rng);
  }

  // 每步调用：dt（秒）
  void step(double dt) {
    // 1) 到期随机换目标速度
    m_tSinceRetarget += dt;
    if (m_tSinceRetarget >= m_retargetPeriod) {
      m_tSinceRetarget = 0.0;
      m_v_goal = m_V_MAX * m_uni(m_rng);                // [-V_MAX, +V_MAX]
      if (m_x > m_max - m_buffer) m_v_goal = -std::fabs(m_v_goal); // 靠右端向内
      if (m_x < m_min + m_buffer) m_v_goal =  std::fabs(m_v_goal); // 靠左端向内
    }

    // 2) 限加速度逼近 v_goal
    const double dv = clampd(m_v_goal - m_v, -m_A_MAX * dt, m_A_MAX * dt);
    m_v += dv;

    // 3) 积分位置并做端点反射
    double x_new = m_x + m_v * dt;
    if (x_new > m_max) {
      x_new    = m_max - (x_new - m_max);
      m_v      = -std::fabs(m_v);
      m_v_goal = -std::fabs(m_v_goal);
    } else if (x_new < m_min) {
      x_new    = m_min + (m_min - x_new);
      m_v      =  std::fabs(m_v);
      m_v_goal =  std::fabs(m_v_goal);
    }
    m_x = clampd(x_new, m_min, m_max);

    // 4) 下发位置
    m_motor->setPosition(m_x);
  }

private:
  Motor* m_motor = nullptr;
  Params m_params;

  // 行程
  double m_min = 0.0, m_max = 0.0, m_span = 0.0, m_buffer = 0.0;

  // 内部状态
  double m_x = 0.0;          // 位置
  double m_v = 0.0;          // 速度
  double m_v_goal = 0.0;     // 目标速度
  double m_V_MAX = 0.0;      // 速度上限
  double m_A_MAX = 0.0;      // 最大加速度
  double m_retargetPeriod = 0.6;
  double m_tSinceRetarget = 0.0;

  // 随机
  std::mt19937 m_rng;
  std::uniform_real_distribution<double> m_uni;
};

// ========================== main ==========================
int main(int, char**) {
  using namespace Config;

  Robot robot;
  const int timeStep = static_cast<int>(robot.getBasicTimeStep());
  const double dt = timeStep / 1000.0;

  LED* led1 = robot.getLED(LED1);
  LED* led2 = robot.getLED(LED2);
  LED* led3 = robot.getLED(LED3);
  LED* led4 = robot.getLED(LED4);

  // 基于全局默认构造每个电机参数：不同 seed
  std::vector<std::unique_ptr<RandomLinearDriver>> drivers;
  drivers.reserve(MOTOR_NAMES.size());

  for (size_t i = 0; i < MOTOR_NAMES.size(); ++i) {
    if (Motor* m = robot.getMotor(MOTOR_NAMES[i])) {
      RandomLinearDriver::Params p{};
      p.speedScale       = DRIVER_DEFAULTS.speedScale;
      p.baseVmaxRatio    = DRIVER_DEFAULTS.baseVmaxRatio;
      p.baseAmax         = DRIVER_DEFAULTS.baseAmax;
      p.baseRetarget     = DRIVER_DEFAULTS.baseRetarget;
      p.defaultMin       = DRIVER_DEFAULTS.defaultMin;
      p.defaultMax       = DRIVER_DEFAULTS.defaultMax;
      p.edgeBufferPct    = DRIVER_DEFAULTS.edgeBufferPct;
      p.edgeBufferAbsMin = DRIVER_DEFAULTS.edgeBufferAbsMin;
      p.positionForce    = DRIVER_DEFAULTS.positionForce;
      p.seed             = DRIVER_DEFAULTS.baseSeed + static_cast<unsigned>(i + 1);

      // 对 pitch/yaw/roll（索引 3/4/5）放宽默认行程（仅在硬件未给 min/max 时才用到）
      if (ORIENT_RANGE_OVERRIDE.enable && (i >= 3 && i <= 5)) {
        p.defaultMin = ORIENT_RANGE_OVERRIDE.minPos;
        p.defaultMax = ORIENT_RANGE_OVERRIDE.maxPos;
      }

      try {
        drivers.emplace_back(std::make_unique<RandomLinearDriver>(m, p));
      } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << " for " << MOTOR_NAMES[i] << "\n";
      }
    } else {
      std::cerr << "[ERROR] Motor not found: " << MOTOR_NAMES[i] << "\n";
    }
  }

  if (drivers.empty()) {
    std::cerr << "[FATAL] No valid motors.\n";
    return 1;
  }

  // 主循环
  while (robot.step(timeStep) != -1) {
    for (auto& d : drivers) d->step(dt);

    if (BIG_ARMOR) {
      if (led1) led1->set(255);
      if (led2) led2->set(255);
      if (led3) led3->set(0);
      if (led4) led4->set(0);
    } else {
      if (led1) led1->set(0);
      if (led2) led2->set(0);
      if (led3) led3->set(255);
      if (led4) led4->set(255);
    }
  }
  return 0;
}
