// ============================================================================
// SRM 校内赛自瞄赛道主程序
//
// 与 standard.cpp 的区别：
//   * 通信走《SRM 校内赛自瞄通信协议》（io::Gimbal，USB CDC 虚拟串口），
//     而不是 RoboMaster 开发板 C 型的 SocketCAN。
//   * 姿态只有 yaw/pitch/roll 三个角，没有四元数，因此用
//     Solver::set_R_gimbal2world(yaw, pitch, roll) 直接构造 R_gimbal2world。
//   * 不使用 MPC Planner：协议没有 yaw/pitch 的角速度、角加速度通道，
//     纯角度闭环，因此沿用 Aimer + Shooter。
//
// 用法：
//   ./build/contest configs/contest.yaml
//   ./build/contest configs/contest.yaml --no-fire      # 只瞄准，不发开火请求
// ============================================================================

#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <initializer_list>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/config.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double RAD2DEG = 57.29577951308232;

// 机械限位兜底。规则书 3.3：机械限位由下位机保证，这里只是防御性裁剪，
// 避免异常估计值把云台目标甩到物理行程之外。
constexpr double PITCH_MIN_DEG = -45.0;
constexpr double PITCH_MAX_DEG = 45.0;
constexpr double MAX_HORIZON_DISTANCE = 30.0;  // m，超过则认为是异常估计

// 启动后先只瞄准不开火，等待估计器收敛与云台稳定
constexpr double SETTLE_TIME = 1.0;  // s
}  // namespace

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明}"
    "{@config-path   | configs/contest.yaml   | 位置参数，yaml 配置文件路径 }"
    "{no-fire        |                        | 只瞄准，不发送开火请求（安全调试用）}";

  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  const bool no_fire = cli.has("no-fire");

  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto yaml = tools::load_config(config_path);

  // 启动时检查模型资源路径，避免跑到推理那一步才发现路径写错
  for (const auto & key : {"yolov5_model_path", "yolov8_model_path", "yolo11_model_path",
                           "classify_model"}) {
    if (!yaml[key]) continue;
    const auto original = yaml[key].as<std::string>();
    const auto resolved = tools::resolve_asset_path(original, config_path);
    if (resolved == original)
      tools::logger()->warn("[Contest] {} = \"{}\" 未找到，请检查路径。", key, original);
    else if (resolved != original)
      tools::logger()->info("[Contest] {} -> {}", key, resolved);
  }

  // ---- 姿态正方向约定（协议未定义，实测后在此处修正）----------------------
  // yaw_sign: 若下位机 yaw 正方向与代码相反，改为 -1
  // pitch_sign: 若下位机 pitch 正方向与代码相反（代码约定抬头为负），改为 -1
  const double yaw_sign = yaml["yaw_sign"] ? yaml["yaw_sign"].as<double>() : 1.0;
  const double pitch_sign = yaml["pitch_sign"] ? yaml["pitch_sign"].as<double>() : 1.0;
  const double default_bullet_speed =
    yaml["default_bullet_speed"] ? yaml["default_bullet_speed"].as<double>() : 22.0;

  tools::logger()->info(
    "[Contest] yaw_sign = {:.0f}, pitch_sign = {:.0f}, default_bullet_speed = {:.2f} m/s, "
    "auto_fire = {}",
    yaw_sign, pitch_sign, default_bullet_speed, no_fire ? "OFF(--no-fire)" : "见 yaml");

  tools::Exiter exiter;
  tools::Plotter plotter;

  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);

  if (!gimbal.wait_for_first_frame(3s)) {
    tools::logger()->error(
      "[Contest] 3 秒内没有收到任何下行帧，请检查 com_port、接线与下位机是否在发送。");
    return 1;
  }

  cv::Mat img;
  std::chrono::steady_clock::time_point t;

  uint64_t frame_count = 0;
  uint64_t sent_control = 0;
  uint64_t sent_hold = 0;
  uint64_t sent_fire = 0;
  const auto start_time = std::chrono::steady_clock::now();
  auto last_log = start_time;

  while (!exiter.exit()) {
    camera.read(img, t);
    ++frame_count;
    const auto now = std::chrono::steady_clock::now();

    // 用接收时刻对 yaw/pitch 插值，替代原四元数 slerp
    const auto attitude = gimbal.attitude(t);
    const auto state = gimbal.state();

    // 解算一律使用协议原始角度：yaw_sign / pitch_sign 只在"下发"那一步应用。
    // 若在这里翻转，下发时再翻一次会互相抵消，云台会朝反方向跑。
    solver.set_R_gimbal2world(attitude.yaw, attitude.pitch, attitude.roll);

    const double bullet_speed =
      state.bullet_speed > 0 ? state.bullet_speed : default_bullet_speed;

    auto armors = detector.detect(img);
    auto targets = tracker.track(armors, t);
    auto command = aimer.aim(targets, t, bullet_speed);

    // 水平距离只用于日志与安全检查
    double horizon_distance = 0;
    if (!targets.empty()) {
      const auto & x = targets.front().ekf_x();
      horizon_distance = std::sqrt(x[0] * x[0] + x[2] * x[2]);
    }

    // 启动后先只瞄准不开火，等待估计器收敛与云台稳定
    const bool settled = tools::delta_time(now, start_time) >= SETTLE_TIME;

    // 开火保护：命令行 --no-fire 优先；未稳定不开火；弹速无效不开火（弹道解算不可信）
    const bool may_fire = settled && !no_fire && std::isfinite(bullet_speed) && bullet_speed > 0;

    const bool dangerous =
      !std::isfinite(command.yaw) || !std::isfinite(command.pitch) || !command.control ||
      !std::isfinite(horizon_distance) || horizon_distance <= 0.0 ||
      horizon_distance > MAX_HORIZON_DISTANCE;

    if (dangerous) {
      // 协议第 5 条第 5 点：省略记录时下位机不会自动清零旧字段。
      // 因此无有效目标时必须发完整帧，并把当前回传角回填，避免把云台拽向零位。
      // hold() 回填的是协议原始角，不需要再乘 sign。
      gimbal.hold(false);
      ++sent_hold;
    } else {
      const double pitch_deg = tools::limit_min_max(
        pitch_sign * command.pitch * RAD2DEG, PITCH_MIN_DEG, PITCH_MAX_DEG);
      const double yaw_deg = yaw_sign * command.yaw * RAD2DEG;

      const bool fire = command.shoot && may_fire;

      gimbal.send_deg(yaw_deg, pitch_deg, fire);
      ++sent_control;
      if (fire) ++sent_fire;

      nlohmann::json data;
      data["gimbal_yaw"] = state.yaw;
      data["gimbal_pitch"] = state.pitch;
      data["cmd_yaw"] = yaw_deg;
      data["cmd_pitch"] = pitch_deg;
      data["horizon_distance"] = horizon_distance;
      data["bullet_speed"] = bullet_speed;
      data["fire"] = fire ? 1 : 0;
      data["shoot_request"] = command.shoot ? 1 : 0;
      plotter.plot(data);
    }

    // 1 Hz 状态行：用于实测 pitch 正方向、确认延迟与链路健康度
    if (tools::delta_time(now, last_log) >= 1.0) {
      last_log = now;
      tools::logger()->info(
        "[Contest] frames={} ctrl={} hold={} fire={} | rx={} tx={} | gimbal yaw={:+.2f} "
        "pitch={:+.2f} roll={:+.2f} | b={:.1f} m/s | d={:.2f} m | targets={}",
        frame_count, sent_control, sent_hold, sent_fire, gimbal.received_frames(),
        gimbal.sent_frames(), state.yaw, state.pitch, state.roll, bullet_speed, horizon_distance,
        targets.size());
    }
  }

  // 退出时回填当前角度并明确清零开火请求
  gimbal.hold(false);
  tools::logger()->info(
    "[Contest] Exit. frames={} ctrl={} hold={} fire={}", frame_count, sent_control, sent_hold,
    sent_fire);

  return 0;
}
