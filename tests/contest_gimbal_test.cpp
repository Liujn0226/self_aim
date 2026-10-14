// ============================================================================
// SRM 校内赛自瞄协议验证工具
//
// 用途：在不接相机、不跑神经网络的前提下，用最短时间验证
//       "上位机对 18 字节上行帧 / 30 字节下行帧的理解是否正确"。
//
// 用法：
//   ./build/contest_gimbal_test configs/contest.yaml
//   ./build/contest_gimbal_test configs/contest.yaml --send-yaw=0 --send-pitch=0
//   ./build/contest_gimbal_test configs/contest.yaml --send-yaw=-10 --send-pitch=5 --fire
//
// 交互命令（在终端输入后回车）：
//   h                回到当前回传角度（保持，fire_flag = 0）
//   s <yaw> <pitch>  发送指定角度（度），fire_flag = 0
//   f <yaw> <pitch>  发送指定角度（度），fire_flag = 1
//   q                退出
//
// 验证要点：
//   1. 能持续收到 30 字节下行帧（rx 计数增长、parse_errors 不增长）
//      → 说明 body_len=28、ID 1(20B)/ID 2(4B) 的顺序与长度理解正确
//   2. 手动转动云台时 yaw/pitch 跟着变化，且与实物方向一致
//      → 用来实测 pitch 正方向（决定 contest.yaml 里的 pitch_sign）
//   3. 发送指定角度后云台确实转到该角度
//      → 说明 18 字节上行帧被正确接受
// ============================================================================

#include <fmt/core.h>
#include <poll.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include "io/gimbal/gimbal.hpp"
#include "tools/config.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

namespace
{
/// 从 stdin 读一行（带超时），超时返回空 optional
std::optional<std::string> read_line_with_timeout(int timeout_ms)
{
  struct pollfd pfd;
  pfd.fd = 0;  // STDIN_FILENO
  pfd.events = POLLIN;

  const int ready = ::poll(&pfd, 1, timeout_ms);
  if (ready <= 0) return std::nullopt;

  std::string line;
  if (!std::getline(std::cin, line)) return std::nullopt;
  return line;
}
}  // namespace

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ? |                      | 输出命令行参数说明}"
    "{@config-path   | configs/contest.yaml | 位置参数，yaml 配置文件路径 }"
    "{send-yaw       |                      | 发送一帧的 yaw（度），配合 send-pitch 使用后立即退出}"
    "{send-pitch     |                      | 发送一帧的 pitch（度）}"
    "{fire f         |                      | 一次性发送时把 fire_flag 置 1}";

  cv::CommandLineParser cli(argc, argv, keys);
  const auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  const bool has_send = cli.has("send-yaw") && cli.has("send-pitch");
  const double send_yaw = has_send ? cli.get<double>("send-yaw") : 0.0;
  const double send_pitch = has_send ? cli.get<double>("send-pitch") : 0.0;
  const bool send_fire = cli.has("fire");

  // 先解析并打印配置路径（失败时给出搜索过的目录），再交给 Gimbal 打开串口
  (void)tools::load_config(config_path);

  io::Gimbal gimbal(config_path);

  // ---- 一次性发送模式 -----------------------------------------------------
  if (has_send) {
    if (!gimbal.wait_for_first_frame(2s))
      tools::logger()->warn("[Test] 尚未收到下行帧，仍将发送一次上行帧。");

    tools::logger()->info(
      "[Test] Sending one frame: yaw={:.2f} deg pitch={:.2f} deg fire={}", send_yaw, send_pitch,
      send_fire);
    gimbal.send_deg(send_yaw, send_pitch, send_fire);

    // 给下位机一点时间回传，观察云台是否真的动了
    for (int i = 0; i < 10; ++i) {
      std::this_thread::sleep_for(100ms);
      const auto s = gimbal.state();
      tools::logger()->info(
        "[Test] rx={} | yaw={:+.2f} pitch={:+.2f} roll={:+.2f} mode={} color={} b={:.2f} m/s",
        gimbal.received_frames(), s.yaw, s.pitch, s.roll, s.mode, s.color, s.bullet_speed);
    }
    return 0;
  }

  // ---- 交互监控模式 -------------------------------------------------------
  if (!gimbal.wait_for_first_frame(3s)) {
    tools::logger()->error(
      "[Test] 3 秒内没有收到任何下行帧。请依次检查：\n"
      "       1) com_port 是否正确（ls -l /dev/ttyACM*）\n"
      "       2) 当前用户是否在 dialout 组（ls -l /dev/ttyACM0 看 group）\n"
      "       3) 下位机是否在上电并发送\n"
      "       4) 下位机帧长是否为 30 字节（body_len=28）");
    return 1;
  }

  tools::logger()->info(
    "[Test] 已连通。输入 h / s <yaw> <pitch> / f <yaw> <pitch> / q 操作。\n"
    "       同时会以 20Hz 打印回传状态。");

  auto t0 = std::chrono::steady_clock::now();
  uint64_t last_frames = 0;
  auto last_report = std::chrono::steady_clock::now();

  while (true) {
    const auto line = read_line_with_timeout(50);  // 50ms → 约 20Hz
    if (line) {
      std::istringstream iss(*line);
      std::string cmd;
      iss >> cmd;

      if (cmd == "q" || cmd == "quit") {
        break;
      } else if (cmd == "h") {
        gimbal.hold(false);
        tools::logger()->info("[Test] hold: fire_flag = 0");
      } else if (cmd == "s" || cmd == "f") {
        double yaw = 0, pitch = 0;
        if (!(iss >> yaw >> pitch)) {
          tools::logger()->warn("[Test] 用法: {} <yaw_deg> <pitch_deg>", cmd);
        } else {
          const bool fire = (cmd == "f");
          gimbal.send_deg(yaw, pitch, fire);
          tools::logger()->info(
            "[Test] Sent yaw={:.2f} pitch={:.2f} fire={}", yaw, pitch, fire);
        }
      } else if (!cmd.empty()) {
        tools::logger()->warn("[Test] 未知命令: {}（可用: h / s / f / q）", cmd);
      }
    }

    const auto now = std::chrono::steady_clock::now();
    const auto dt = tools::delta_time(now, last_report);
    if (dt < 0.05) continue;

    const auto frames = gimbal.received_frames();
    const auto hz = (frames - last_frames) / dt;
    last_frames = frames;
    last_report = now;

    const auto s = gimbal.state();
    fmt::print(
      "\r[t={:7.2f}s] rx={:6d} ({:5.1f}Hz) tx={:6d} | yaw={:+8.2f} pitch={:+7.2f} "
      "roll={:+7.2f} | mode={} color={} b={:5.2f} m/s        ",
      tools::delta_time(now, t0), frames, hz, gimbal.sent_frames(), s.yaw, s.pitch, s.roll, s.mode,
      s.color, s.bullet_speed);
    std::cout.flush();
  }

  std::cout << "\n";
  gimbal.hold(false);
  tools::logger()->info(
    "[Test] Exit. rx={} tx={}", gimbal.received_frames(), gimbal.sent_frames());
  return 0;
}
