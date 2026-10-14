#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "io/command.hpp"
#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"

namespace io
{

// ============================================================================
// SRM 校内赛自瞄通信协议（《SRM 校内赛自瞄通信协议》最终版）
//
//   报文 = int16 body_len (小端) + body_len 字节的 records
//   每条 record = int16 id + 该 ID 的定长数据体
//   所有整数/浮点均为小端；结构体按 1 字节对齐
//   无帧头、无帧尾、无 CRC、无时间戳、无序列号
//
//   上位机 → 下位机： body_len = 16，总帧长 18，顺序固定 ID 1、ID 2
//   下位机 → 上位机： body_len = 28，总帧长 30，顺序固定 ID 1、ID 2
//
//   注意：USB 包边界 != 协议帧边界，必须流式缓存组帧。
// ============================================================================

constexpr int16_t DL_GIMBAL_RECORD_ID = 1;
constexpr int16_t DL_SHOOT_RECORD_ID = 2;

constexpr size_t UL_GIMBAL_BODY_SIZE = 8;   // yaw + pitch
constexpr size_t UL_SHOOT_BODY_SIZE = 4;    // fire_flag
constexpr size_t DL_GIMBAL_BODY_SIZE = 20;  // yaw + pitch + roll + mode + color
constexpr size_t DL_SHOOT_BODY_SIZE = 4;    // bullet_speed

constexpr size_t UL_FRAME_SIZE = 2 + 2 + UL_GIMBAL_BODY_SIZE + 2 + UL_SHOOT_BODY_SIZE;  // 18
constexpr size_t DL_FRAME_SIZE = 2 + 2 + DL_GIMBAL_BODY_SIZE + 2 + DL_SHOOT_BODY_SIZE;  // 30

/// 上位机 → 下位机，ID 1：云台目标
struct __attribute__((packed)) Gimbal_Receive_s
{
  float yaw;    // 角度制
  float pitch;  // 角度制
};

/// 上位机 → 下位机，ID 2：开火请求
struct __attribute__((packed)) Shoot_Receive_s
{
  int32_t fire_flag;  // 0 = 不请求；非零 = 请求
};

/// 下位机 → 上位机，ID 1：云台状态
struct __attribute__((packed)) Gimbal_Send_s
{
  float yaw;    // 角度制
  float pitch;  // 角度制
  float roll;   // 角度制
  int32_t mode;  // 当前分支固定发 0，保留字段，上位机不应解释为其他枚举
  int32_t color; // 当前分支写入机器人 ID，字段名沿用源码
};

/// 下位机 → 上位机，ID 2：发射状态
struct __attribute__((packed)) Shoot_Send_s
{
  float bullet_speed;  // m/s，裁判系统提供
};

static_assert(sizeof(Gimbal_Receive_s) == UL_GIMBAL_BODY_SIZE, "protocol layout mismatch");
static_assert(sizeof(Shoot_Receive_s) == UL_SHOOT_BODY_SIZE, "protocol layout mismatch");
static_assert(sizeof(Gimbal_Send_s) == DL_GIMBAL_BODY_SIZE, "protocol layout mismatch");
static_assert(sizeof(Shoot_Send_s) == DL_SHOOT_BODY_SIZE, "protocol layout mismatch");

/// 一个完整下行帧解析出来的内容
struct GimbalState
{
  double yaw = 0;           // 度，±180 包裹
  double pitch = 0;         // 度
  double roll = 0;          // 度
  int32_t mode = 0;         // 保留字段
  int32_t color = 0;        // 机器人 ID
  double bullet_speed = 0;  // m/s
};

/// 某一时刻的云台姿态（弧度制，供 Solver 使用）
struct Attitude
{
  double yaw = 0;
  double pitch = 0;
  double roll = 0;
};

class Gimbal
{
public:
  explicit Gimbal(const std::string & config_path);

  ~Gimbal();

  Gimbal(const Gimbal &) = delete;
  Gimbal & operator=(const Gimbal &) = delete;

  /// 是否已经收到过至少一帧下行数据
  bool ready() const;

  /// 自构造起累计收到的完整下行帧数
  uint64_t received_frames() const;

  /// 自构造起累计成功写出的上行帧数
  uint64_t sent_frames() const;

  /// 最近一帧下行数据（度制）
  GimbalState state() const;

  /// 最近一帧的姿态，弧度制。
  /// 注意 yaw 是"已解缠的连续角"，与 attitude(timestamp) 的输出同一约定，
  /// 因此跨越 ±180° 时数值会连续累积，不会跳变。
  Attitude attitude() const;

  /// 用接收时刻对 yaw/pitch 线性插值，得到 timestamp 时刻的姿态（弧度制）
  /// 替代原四元数 slerp：新协议不提供四元数，且相机刚连于云台，只有两个自由度
  Attitude attitude(std::chrono::steady_clock::time_point timestamp);

  /// 阻塞等待第一帧下行数据；返回 false 表示超时
  bool wait_for_first_frame(std::chrono::milliseconds timeout = std::chrono::seconds(3));

  /// 发送一帧（body_len = 16，ID 1 + ID 2）。
  /// yaw/pitch 为弧度，内部转角度制并把 yaw 归一化到 ±180°。
  /// control 仅参与语义表达：新协议没有"视觉是否接管"的控制位，
  /// 接管由下位机的鼠标右键决定，因此该参数在协议层被忽略。
  void send(bool control, bool fire, double yaw, double pitch);

  /// 直接发送角度制数值
  void send_deg(double yaw_deg, double pitch_deg, bool fire);

  /// 以最近一次回传的角度发送（无目标时使用，避免把云台拽向零位）
  void hold(bool fire = false);

private:
  struct Sample
  {
    double heading;  // 连续（已解缠）的 yaw，弧度
    double pitch;    // 弧度
    std::chrono::steady_clock::time_point timestamp;
  };

  void open_serial();
  void read_thread();
  /// 解析一条 body（不含 2 字节 body_len）。返回 true 表示帧被成功消费。
  bool handle_frame(const uint8_t * body, size_t body_len);
  void reconnect();
  void write_frame(double yaw_deg, double pitch_deg, bool fire);

  /// 不阻塞地返回最近一帧角度（弧度制），供 attitude(timestamp) 超时兜底
  Attitude latest_attitude() const;

  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_{false};
  mutable std::mutex serial_mutex_;  // 串口读写与重连互斥

  mutable std::mutex state_mutex_;
  GimbalState state_;
  bool has_state_ = false;
  double prev_heading_deg_ = 0;  // 已解缠的 yaw，单位：度
  bool has_prev_heading_ = false;
  // 最近一帧已解缠的姿态（弧度制），保证 attitude() 与 attitude(t) 同一约定
  Attitude latest_attitude_;

  // PopWhenFull = true：队列满时丢最旧的样本。
  // 必须如此，否则 push 会静默丢弃新样本，而 pop/front 会无限等待。
  tools::ThreadSafeQueue<Sample, true> queue_{5000};

  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> tx_frames_{0};
  std::atomic<uint64_t> rx_errors_{0};
  std::atomic<uint64_t> rx_resyncs_{0};

  std::string com_port_;
  uint32_t baudrate_ = 115200;
  uint32_t rx_timeout_ms_ = 20;
  double default_bullet_speed_ = 22.0;
};

}  // namespace io

#endif  // IO__GIMBAL_HPP
