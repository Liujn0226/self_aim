#include "gimbal.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
namespace
{
constexpr double RAD2DEG = 57.29577951308232;
constexpr double DEG2RAD = 1.0 / RAD2DEG;

/// 归一化到 (-180, 180]
double wrap_180(double deg)
{
  double r = std::fmod(deg + 180.0, 360.0);
  if (r < 0) r += 360.0;
  return r - 180.0;
}

/// 把 current 解缠到 previous 附近：两者相差超过 180° 时补 ±360°
double unwrap_near(double current, double previous)
{
  return previous + wrap_180(current - previous);
}
}  // namespace

Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  com_port_ = tools::read<std::string>(yaml, "com_port");
  if (yaml["baudrate"]) baudrate_ = yaml["baudrate"].as<uint32_t>();
  if (yaml["rx_timeout_ms"]) rx_timeout_ms_ = yaml["rx_timeout_ms"].as<uint32_t>();
  if (yaml["default_bullet_speed"])
    default_bullet_speed_ = yaml["default_bullet_speed"].as<double>();

  state_.bullet_speed = default_bullet_speed_;

  open_serial();

  thread_ = std::thread(&Gimbal::read_thread, this);

  tools::logger()->info(
    "[Gimbal] Opened {}. Uplink {}B (body_len {}), downlink {}B (body_len {}).", com_port_,
    UL_FRAME_SIZE, UL_GIMBAL_BODY_SIZE + UL_SHOOT_BODY_SIZE, DL_FRAME_SIZE,
    DL_GIMBAL_BODY_SIZE + DL_SHOOT_BODY_SIZE);
  tools::logger()->info(
    "[Gimbal] Protocol constants: UL body {}+{}={} (frame {}), DL body {}+{}={} "
    "(frame {}), expected downlink body_len={}.",
    UL_GIMBAL_BODY_SIZE, UL_SHOOT_BODY_SIZE, UL_GIMBAL_BODY_SIZE + UL_SHOOT_BODY_SIZE,
    UL_FRAME_SIZE, DL_GIMBAL_BODY_SIZE, DL_SHOOT_BODY_SIZE,
    DL_GIMBAL_BODY_SIZE + DL_SHOOT_BODY_SIZE, DL_FRAME_SIZE,
    DL_GIMBAL_BODY_SIZE + DL_SHOOT_BODY_SIZE);
  tools::logger()->info(
    "[Gimbal] Waiting for the first downlink frame... (default bullet_speed = {:.2f} m/s)",
    default_bullet_speed_);
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();

  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    try {
      if (serial_.isOpen()) serial_.close();
    } catch (...) {
    }
  }

  tools::logger()->info(
    "[Gimbal] Closed. rx={} tx={} parse_errors={} resyncs={}", rx_frames_.load(), tx_frames_.load(),
    rx_errors_.load(), rx_resyncs_.load());
}

void Gimbal::open_serial()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    serial_.setPort(com_port_);
    serial_.setBaudrate(baudrate_);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    // 必须有超时，否则 read() 永久阻塞、read_thread 无法退出、析构死锁
    serial_.setTimeout(rx_timeout_ms_, rx_timeout_ms_, 0, rx_timeout_ms_, 0);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial {}: {}", com_port_, e.what());
    exit(1);
  }
}

bool Gimbal::ready() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return has_state_;
}

uint64_t Gimbal::received_frames() const { return rx_frames_.load(); }

uint64_t Gimbal::sent_frames() const { return tx_frames_.load(); }

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

Attitude Gimbal::attitude() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_attitude_;
}

bool Gimbal::wait_for_first_frame(std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (ready()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return ready();
}

Attitude Gimbal::attitude(std::chrono::steady_clock::time_point timestamp)
{
  Sample a;
  // 正常情况下队列里始终有数据，会立刻返回。
  // 超时兜底：避免下位机停发时 attitude() 永久阻塞导致主循环卡死。
  if (!queue_.try_pop(a, std::chrono::milliseconds(200))) return latest_attitude();

  const auto age_limit = std::chrono::milliseconds(200);
  const auto started_at = std::chrono::steady_clock::now();
  constexpr double MAX_WAIT = 0.2;  // s，单次调用最多额外等待多久

  while (true) {
    Sample b;
    // 找不到新的右端点时用最新样本兜底（比 a 更新，不会倒退）
    if (!queue_.try_pop(b, age_limit)) return latest_attitude();

    const auto dt_ab = std::chrono::duration<double>(b.timestamp - a.timestamp).count();
    const auto dt_ac = std::chrono::duration<double>(timestamp - a.timestamp).count();

    if (dt_ab > 1e-9) {
      const auto k = dt_ac / dt_ab;
      if (k < 0.0) return Attitude{a.heading, a.pitch, 0.0};
      if (k <= 1.0) {
        Attitude attitude;
        attitude.yaw = a.heading + k * (b.heading - a.heading);
        attitude.pitch = a.pitch + k * (b.pitch - a.pitch);
        attitude.roll = 0.0;
        return attitude;
      }
    } else if (dt_ac <= 0.0) {
      return Attitude{a.heading, a.pitch, 0.0};
    }

    // timestamp 晚于 b，用 b 作为新的左端点继续向前推进。
    // 但总等待时间要有上限，否则接收频率骤降时会把控制循环一起拖住。
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count() >
        MAX_WAIT)
      return latest_attitude();

    a = b;
  }
}

Attitude Gimbal::latest_attitude() const
{
  // 兜底路径：不阻塞，直接返回最近一帧（已解缠、弧度制），与 attitude(t) 同一约定
  std::lock_guard<std::mutex> lock(state_mutex_);
  return latest_attitude_;
}

void Gimbal::send(bool control, bool fire, double yaw, double pitch)
{
  (void)control;  // 新协议没有"视觉是否接管"的控制位，接管由下位机鼠标右键决定
  write_frame(wrap_180(yaw * RAD2DEG), pitch * RAD2DEG, fire);
}

void Gimbal::send_deg(double yaw_deg, double pitch_deg, bool fire)
{
  write_frame(wrap_180(yaw_deg), pitch_deg, fire);
}

void Gimbal::hold(bool fire)
{
  const auto s = state();
  write_frame(wrap_180(s.yaw), s.pitch, fire);
}

void Gimbal::write_frame(double yaw_deg, double pitch_deg, bool fire)
{
  // body_len = 16，其后依次是 ID 1（8B）与 ID 2（4B）
  uint8_t buffer[UL_FRAME_SIZE];

  const int16_t body_len = static_cast<int16_t>(UL_FRAME_SIZE - sizeof(int16_t));
  const int16_t gimbal_id = DL_GIMBAL_RECORD_ID;
  const int16_t shoot_id = DL_SHOOT_RECORD_ID;

  const float yaw_f = static_cast<float>(yaw_deg);
  const float pitch_f = static_cast<float>(pitch_deg);
  const int32_t fire_flag = fire ? 1 : 0;

  size_t o = 0;
  std::memcpy(buffer + o, &body_len, sizeof(body_len));
  o += sizeof(body_len);
  std::memcpy(buffer + o, &gimbal_id, sizeof(gimbal_id));
  o += sizeof(gimbal_id);
  std::memcpy(buffer + o, &yaw_f, sizeof(yaw_f));
  o += sizeof(yaw_f);
  std::memcpy(buffer + o, &pitch_f, sizeof(pitch_f));
  o += sizeof(pitch_f);
  std::memcpy(buffer + o, &shoot_id, sizeof(shoot_id));
  o += sizeof(shoot_id);
  std::memcpy(buffer + o, &fire_flag, sizeof(fire_flag));
  o += sizeof(fire_flag);

  if (o != UL_FRAME_SIZE) {
  tools::logger()->error(
    "[Gimbal] Internal error: built {} bytes, expected {}", o, UL_FRAME_SIZE);
  return;
}

std::lock_guard<std::mutex> lock(serial_mutex_);
try {
  const auto written = serial_.write(buffer, UL_FRAME_SIZE);
  if (written == UL_FRAME_SIZE)
    tx_frames_.fetch_add(1);
  else
    tools::logger()->warn(
      "[Gimbal] Short write: {}/{}", written, UL_FRAME_SIZE);
} catch (const std::exception & e) {
  tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
}
}

bool Gimbal::handle_frame(const uint8_t * body, size_t body_len)
{
  size_t offset = 0;
  bool has_gimbal = false;
  bool has_shoot = false;
  Gimbal_Send_s gimbal_send{};
  Shoot_Send_s shoot_send{};

  while (offset + sizeof(int16_t) <= body_len) {
    int16_t id = 0;
    std::memcpy(&id, body + offset, sizeof(id));
    offset += sizeof(id);

    if (id == DL_GIMBAL_RECORD_ID) {
      if (offset + DL_GIMBAL_BODY_SIZE > body_len) return false;
      std::memcpy(&gimbal_send, body + offset, DL_GIMBAL_BODY_SIZE);
      offset += DL_GIMBAL_BODY_SIZE;
      has_gimbal = true;
    } else if (id == DL_SHOOT_RECORD_ID) {
      if (offset + DL_SHOOT_BODY_SIZE > body_len) return false;
      std::memcpy(&shoot_send, body + offset, DL_SHOOT_BODY_SIZE);
      offset += DL_SHOOT_BODY_SIZE;
      has_shoot = true;
    } else {
      // 未知 ID：协议没有魔数和 CRC，无法在任意字节处可靠重同步，丢弃整帧
      rx_errors_.fetch_add(1);
      tools::logger()->debug("[Gimbal] Unknown record id {}, frame dropped.", id);
      return false;
    }
  }

  // 本赛制要求每帧都包含 ID 1 与 ID 2
  if (!has_gimbal || !has_shoot) {
    rx_errors_.fetch_add(1);
    tools::logger()->debug(
      "[Gimbal] Incomplete frame (id1={}, id2={}), dropped.", has_gimbal, has_shoot);
    return false;
  }

  const auto now = std::chrono::steady_clock::now();

  double yaw_raw = std::isfinite(gimbal_send.yaw) ? gimbal_send.yaw : 0.0;
  double pitch = std::isfinite(gimbal_send.pitch) ? gimbal_send.pitch : 0.0;
  double roll = std::isfinite(gimbal_send.roll) ? gimbal_send.roll : 0.0;

  // 协议 4.2：裁判系统未注入时下位机内部默认 22.0；这里对 0/非法值做兜底
  double bullet_speed = default_bullet_speed_;
  if (std::isfinite(shoot_send.bullet_speed) && shoot_send.bullet_speed > 0)
    bullet_speed = shoot_send.bullet_speed;

  GimbalState snapshot;
  double heading_rad = 0.0;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // yaw 归一化到 ±180°，跨越边界时补 ±360° 再插值，避免算出 360° 的假速度
    const double heading_deg =
      has_prev_heading_ ? unwrap_near(yaw_raw, prev_heading_deg_) : yaw_raw;
    prev_heading_deg_ = heading_deg;
    has_prev_heading_ = true;
    heading_rad = heading_deg * DEG2RAD;

    state_.yaw = wrap_180(yaw_raw);
    state_.pitch = pitch;
    state_.roll = roll;
    state_.mode = gimbal_send.mode;
    state_.color = gimbal_send.color;
    state_.bullet_speed = bullet_speed;
    has_state_ = true;
    snapshot = state_;

    latest_attitude_.yaw = heading_rad;
    latest_attitude_.pitch = pitch * DEG2RAD;
    latest_attitude_.roll = roll * DEG2RAD;
  }

  queue_.push(Sample{heading_rad, pitch * DEG2RAD, now});

  const auto frames = rx_frames_.fetch_add(1) + 1;
  if (frames == 1) {
    tools::logger()->info(
      "[Gimbal] First downlink frame: yaw={:.2f} pitch={:.2f} roll={:.2f} mode={} color={} "
      "bullet_speed={:.2f}",
      snapshot.yaw, snapshot.pitch, snapshot.roll, snapshot.mode, snapshot.color,
      snapshot.bullet_speed);
  } else if (frames % 1000 == 0) {
    tools::logger()->info(
      "[Gimbal] rx={} tx={} errors={} resyncs={} | yaw={:.2f} pitch={:.2f} roll={:.2f} "
      "bullet_speed={:.2f}",
      frames, tx_frames_.load(), rx_errors_.load(), rx_resyncs_.load(), snapshot.yaw,
      snapshot.pitch, snapshot.roll, snapshot.bullet_speed);
  }

  // 接近万向锁时 R_gimbal2world = Rz(yaw)Ry(pitch) 不再可靠，提前告警
  if (std::abs(snapshot.pitch) > 85.0) {
    static auto last_warn = std::chrono::steady_clock::time_point::min();
    if (std::chrono::duration<double>(now - last_warn).count() > 5.0) {
      last_warn = now;
      tools::logger()->warn(
        "[Gimbal] pitch = {:.2f} deg is close to gimbal lock; the yaw/pitch-only attitude model "
        "is unreliable here.",
        snapshot.pitch);
    }
  }

  return true;
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  // 流式接收缓冲区：USB 包边界 != 协议帧边界，一次读取可能包含半帧、整帧或多帧
  std::vector<uint8_t> buffer;
  buffer.reserve(4 * UL_FRAME_SIZE);

  uint8_t chunk[512];
  uint64_t idle_count = 0;

  while (!quit_) {
    size_t n = 0;
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      try {
        if (serial_.isOpen()) n = serial_.read(chunk, sizeof(chunk));
      } catch (const std::exception & e) {
        tools::logger()->debug("[Gimbal] read exception: {}", e.what());
        n = 0;
        rx_errors_.fetch_add(1);
      }
    }

    if (n == 0) {
      // 长时间完全收不到数据（或串口已被重连关闭），尝试重连
      // 500 次 * rx_timeout_ms(20ms) ≈ 10s
      if (++idle_count > 500) {
        idle_count = 0;
        tools::logger()->warn("[Gimbal] No data for a while, attempting to reconnect...");
        buffer.clear();
        queue_.clear();
        reconnect();
      }
      continue;
    }

    idle_count = 0;
    buffer.insert(buffer.end(), chunk, chunk + n);

    // 尽可能多地抠出完整帧
    while (true) {
      if (buffer.size() < sizeof(int16_t)) break;

      int16_t body_len = 0;
      std::memcpy(&body_len, buffer.data(), sizeof(body_len));

      // 协议没有帧头/CRC，长度字段是唯一的逻辑帧边界：非法长度时丢弃全部缓存
      if (body_len != static_cast<int16_t>(DL_FRAME_SIZE - 2)) {
        const auto resyncs = rx_resyncs_.fetch_add(1) + 1;
        // 只在前几次打印原始字节，避免刷屏。
        // 这段字节是定位"期望值 vs 实际字节流"不一致的关键证据。
        if (resyncs <= 5) {
          std::string hex;
          const size_t dump_size = std::min<size_t>(32, buffer.size());
          for (size_t i = 0; i < dump_size; i++)
            hex += fmt::format("{:02X} ", buffer[i]);
          tools::logger()->warn(
            "[Gimbal] Illegal body_len {} (expected {}), buffer.size={}, first {} bytes: {}",
            body_len, DL_FRAME_SIZE -2, buffer.size(), dump_size, hex);
        } else {
          tools::logger()->debug("[Gimbal] Illegal body_len {}, dropping buffer.", body_len);
        }
        buffer.clear();
        break;
      }

      const size_t frame_size = sizeof(int16_t) + static_cast<size_t>(body_len);
      if (buffer.size() < frame_size) break;

      // 只有真正解析成功的帧才被消费；语义非法的帧保留在缓冲区里，
      // 由下面的长度防御逻辑处理，避免静默丢数据。
      if (handle_frame(buffer.data() + sizeof(int16_t), static_cast<size_t>(body_len)))
        buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(frame_size));
      else
        break;
    }

    // 防御：缓冲区不应长期无界增长
    if (buffer.size() > 8 * UL_FRAME_SIZE) buffer.clear();
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  for (int i = 0; i < 10 && !quit_; ++i) {
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      try {
        if (serial_.isOpen()) serial_.close();
      } catch (...) {
      }
    }

    // open_serial 失败时会走 exit(1) 并结束进程；因为重连失败等同于
    // "设备彻底不可用"，继续空转没有意义，所以这里不再额外重试。
    try {
      open_serial();
      tools::logger()->info("[Gimbal] Reconnected {} successfully.", com_port_);
      return;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect attempt {}/10 failed: {}", i + 1, e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  tools::logger()->error("[Gimbal] Reconnect failed after 10 attempts.");
}

}  // namespace io
