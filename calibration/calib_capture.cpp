// ============================================================================
// 校内赛标定数据采集（USB CDC 版本）
//
// 与 calibration/capture.cpp 的区别：
//   原版从 io::CBoard 通过 SocketCAN 读 IMU 四元数（写 .txt 为 wxyz）。
//   本赛制的下位机走 USB CDC，且协议直接给 yaw/pitch/roll 三个角，
//   没有四元数，因此这里把三个角写入 .txt，配合 calib_handeye 使用。
//
// 内参标定也使用同一批图片：calib_camera 只读 .jpg，忽略 .txt，
// 所以一次采集即可同时用于内参和外参。
//
// ★ 实时转角提示：
//   面板会显示"已存位姿里最小的两两位姿夹角"。手眼标定（默认 TSAI 方法）
//   要求任意一对位姿的云台相对旋转大于约 17°，否则有效方程不足、
//   函数会静默返回单位阵。这个数字 >= 17° 才说明位姿散得够开。
//
// 用法：
//   ./build/calib_capture configs/calibration_contest.yaml
//   ./build/calib_capture configs/calibration_contest.yaml -o calib_imgs
//   ./build/calib_capture configs/calibration_contest.yaml -o calib_imgs --on-existing=clear
//   ./build/calib_capture configs/calibration_contest.yaml -o calib_imgs --on-existing=append
//
// 按键：
//   s  保存当前图片 + 当前云台角度
//   q  退出
// ============================================================================

#include <fmt/core.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/config.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace
{

constexpr double RAD2DEG = 57.29577951308232;
constexpr double DEG2RAD = 1.0 / RAD2DEG;

// TSAI 方法丢弃旋转小于约 17° 的运动对（|P| = 2·sin(θ/2) < 0.3）
constexpr double MIN_USEFUL_ANGLE_DEG = 17.2;
// 建议达到的冗余量，留出安全边界
constexpr double GOOD_ANGLE_DEG = 25.0;

/// 一帧云台姿态，单位：度
struct Pose
{
  double yaw = 0;
  double pitch = 0;
  double roll = 0;
};

/// 写出一帧姿态。三行，单位：度，顺序 yaw / pitch / roll。
/// 与之配套的读取端见 calibrate_handeye.cpp 的 read_ypr_deg()。
bool write_ypr(const std::string & path, const Pose & p)
{
  std::ofstream file(path);
  if (!file) return false;
  file << fmt::format("{:.6f}\n{:.6f}\n{:.6f}\n", p.yaw, p.pitch, p.roll);
  return static_cast<bool>(file);
}

/// 读取一帧姿态。返回 false 表示无法使用，原因有三类：
///   1. 文件不存在 / 打不开；
///   2. 不足三个数（文件被截断）；
///   3. 数值非法（NaN / inf）。
/// 注意：旧版 capture.cpp 写的是单行 wxyz 四元数，它只有 4 个数、第 4 个是
/// 同一行的 z，但前 3 个能读出来，因此会被误判为合法姿态。
/// 这里用"是否还有第 4 个数字"来识别这种旧格式并拒绝，避免把四元数当欧拉角用。
bool read_ypr(const std::string & path, Pose & p)
{
  std::ifstream file(path);
  if (!file) return false;

  double yaw = 0, pitch = 0, roll = 0;
  if (!(file >> yaw >> pitch >> roll)) return false;

  double extra = 0;
  if (file >> extra) return false;  // 多于 3 个数 → 旧格式，拒绝

  if (!std::isfinite(yaw) || !std::isfinite(pitch) || !std::isfinite(roll)) return false;

  p = Pose{yaw, pitch, roll};
  return true;
}

/// 两个姿态之间的云台相对旋转角，单位：度
double rotation_angle_deg(const Pose & a, const Pose & b)
{
  const Eigen::Matrix3d Ra =
    tools::rotation_matrix(Eigen::Vector3d{a.yaw, a.pitch, a.roll} * DEG2RAD);
  const Eigen::Matrix3d Rb =
    tools::rotation_matrix(Eigen::Vector3d{b.yaw, b.pitch, b.roll} * DEG2RAD);
  const Eigen::Matrix3d R_rel = Ra.transpose() * Rb;
  const double cos_theta = std::clamp((R_rel.trace() - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(cos_theta) * RAD2DEG;
}

/// 两两位姿夹角的最小值。>= MIN_USEFUL_ANGLE_DEG 才说明散得够开。
double min_pairwise_angle_deg(const std::vector<Pose> & poses)
{
  if (poses.size() < 2) return 0.0;
  double minimum = 1e9;
  for (size_t i = 0; i < poses.size(); i++)
    for (size_t j = i + 1; j < poses.size(); j++)
      minimum = std::min(minimum, rotation_angle_deg(poses[i], poses[j]));
  return minimum;
}

/// 当前姿态与已存位姿的最大夹角。
/// 用来判断"这一张新位姿有没有带来新的旋转信息"——若很小，存了也没用。
double max_angle_to_saved_deg(const std::vector<Pose> & poses, const Pose & candidate)
{
  if (poses.empty()) return 1e9;  // 第一张总是有价值的
  double maximum = 0.0;
  for (const auto & p : poses) maximum = std::max(maximum, rotation_angle_deg(p, candidate));
  return maximum;
}

enum class OnExisting
{
  ask,
  append,
  clear
};

/// 扫描输出文件夹里已有的数据
struct Session
{
  int max_index = 0;
  int image_count = 0;
  int pose_count = 0;
  std::vector<Pose> poses;
  std::vector<int> unreadable_poses;  // 有图但读不出/格式不对的下标
};

Session scan_existing(const std::string & folder)
{
  Session session;
  namespace fs = std::filesystem;
  std::error_code ec;

  // 扫描目录统计图片数量与最大编号。
  // 不按 "1,2,3... 遇到缺号就停" 的方式扫描，避免中间缺号导致漏统计。
  for (const auto & entry : fs::directory_iterator(folder, ec)) {
    if (!entry.is_regular_file()) continue;
    if (entry.path().extension() != ".jpg") continue;
    session.image_count++;
    try {
      const int index = std::stoi(entry.path().stem().string());
      session.max_index = std::max(session.max_index, index);
    } catch (const std::exception &) {
      // 文件名不是数字，忽略
    }
  }

  // 逐个尝试读取姿态。读不出来的记为 unreadable（例如旧版 capture.cpp
  // 写的是单行 wxyz 四元数，只有 3 个数能读出来会失败）。
  for (int i = 1; i <= session.max_index; i++) {
    const auto pose_path = fmt::format("{}/{}.txt", folder, i);
    if (!fs::exists(pose_path, ec)) continue;
    Pose p;
    if (read_ypr(pose_path, p)) {
      session.poses.push_back(p);
      session.pose_count++;
    } else {
      session.unreadable_poses.push_back(i);
    }
  }

  return session;
}

/// 询问用户如何处理已有数据。
/// 返回 std::nullopt 表示用户选择退出（q）。
std::optional<OnExisting> ask_user(const std::string & folder, const Session & session)
{
  fmt::print(
    "\n"
    "======================================================================\n"
    "  输出文件夹 {} 里已有旧数据\n"
    "    图片 {} 张（最大编号 {}）\n"
    "    可读姿态 {} 组\n"
    "    无法解析的姿态 {} 组（可能是旧版 capture.cpp 写的四元数格式）\n"
    "----------------------------------------------------------------------\n"
    "  注意：如果这次标定之前**动过标定板或相机**，\n"
    "        旧数据与新数据不能混用，否则手眼标定会算出错误结果\n"
    "        而且不会报错（板子一动，AX=XB 的前提就破了）。\n"
    "----------------------------------------------------------------------\n"
    "  [c] 清空旧数据，从 1 开始重新采集   （板子/相机动过时选这个）\n"
    "  [a] 保留旧数据，接着往后编号        （板子/相机完全没动时才能选）\n"
    "  [q] 退出，什么都不做\n"
    "======================================================================\n",
    folder, session.image_count, session.max_index, session.pose_count,
    session.unreadable_poses.size());

  if (!session.unreadable_poses.empty()) {
    fmt::print("  无法解析的姿态编号：");
    for (size_t i = 0; i < session.unreadable_poses.size(); i++)
      fmt::print("{}{}", i ? ", " : "", session.unreadable_poses[i]);
    fmt::print("\n  （这些图片仍会被 calib_camera 用于内参标定，但外参不会用到它们）\n");
  }

  while (true) {
    fmt::print("\n请输入 c / a / q 后回车: ");
    std::fflush(stdout);
    std::string line;
    if (!std::getline(std::cin, line)) return std::nullopt;  // stdin 关闭，按退出处理
    if (line.empty()) continue;
    const char c = static_cast<char>(std::tolower(line[0]));
    if (c == 'c') return OnExisting::clear;
    if (c == 'a') return OnExisting::append;
    if (c == 'q') return std::nullopt;
    fmt::print("  无效输入，请输入 c、a 或 q。\n");
  }
}

/// 清空旧的采集数据。
/// 只删除"文件名是纯数字的 .jpg / .txt"，其它文件一律保留。
int clear_existing(const std::string & folder)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  int removed = 0;

  for (const auto & entry : fs::directory_iterator(folder, ec)) {
    if (!entry.is_regular_file()) continue;
    const auto ext = entry.path().extension().string();
    if (ext != ".jpg" && ext != ".txt") continue;

    const auto stem = entry.path().stem().string();
    // 必须是纯数字文件名，避免误删用户的其它文件
    if (stem.empty() || !std::all_of(stem.begin(), stem.end(), [](unsigned char c) {
          return std::isdigit(c) != 0;
        }))
      continue;

    std::error_code remove_ec;
    if (fs::remove(entry.path(), remove_ec))
      removed++;
    else
      tools::logger()->warn("[Calib] 无法删除 {}", entry.path().string());
  }

  return removed;
}

}  // namespace

int main(int argc, char * argv[])
{
  const std::string keys =
    "{help h usage ?  |                                 | 输出命令行参数说明}"
    "{@config-path c  | configs/calibration_contest.yaml | yaml 配置文件路径 }"
    "{output-folder o | calib_imgs                      | 输出文件夹路径 }"
    "{on-existing     | ask                             | 已有旧数据时: ask | append | clear}";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const auto config_path = cli.get<std::string>(0);
  const auto output_folder = cli.get<std::string>("output-folder");
  const auto on_existing_str = cli.get<std::string>("on-existing");

  if (config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  OnExisting on_existing = OnExisting::ask;
  if (on_existing_str == "append")
    on_existing = OnExisting::append;
  else if (on_existing_str == "clear")
    on_existing = OnExisting::clear;
  else if (on_existing_str != "ask") {
    tools::logger()->error(
      "[Calib] --on-existing 只能是 ask / append / clear，收到 \"{}\"。", on_existing_str);
    return 2;
  }

  auto yaml = tools::load_config(config_path);

  const int pattern_cols = tools::read<int>(yaml, "pattern_cols");
  const int pattern_rows = tools::read<int>(yaml, "pattern_rows");
  const auto pattern_size = cv::Size(pattern_cols, pattern_rows);

  // 标定板图案类型：circle（对称圆点阵，原实现）/ chessboard（棋盘格）
  const std::string pattern_type =
    yaml["pattern_type"] ? yaml["pattern_type"].as<std::string>() : "circle";
  const bool use_circle = (pattern_type == "circle");

  std::filesystem::create_directory(output_folder);

  tools::logger()->info(
    "[Calib] 目标文件夹 {}，标定板 {} 列 x {} 行，图案类型 {}", output_folder, pattern_cols,
    pattern_rows, pattern_type);

  // ---- 处理已有旧数据 -----------------------------------------------------
  std::vector<Pose> saved_poses;
  int next_index = 1;

  auto session = scan_existing(output_folder);
  if (session.image_count > 0 || !session.poses.empty()) {
    OnExisting decision = on_existing;
    if (decision == OnExisting::ask) {
      const auto answer = ask_user(output_folder, session);
      if (!answer.has_value()) {
        tools::logger()->info("[Calib] 用户选择退出，未做任何修改。");
        return 0;
      }
      decision = answer.value();
    }

    if (decision == OnExisting::clear) {
      const int removed = clear_existing(output_folder);
      tools::logger()->info("[Calib] 已清空旧数据，删除 {} 个文件。", removed);
      next_index = 1;
    } else {
      next_index = session.max_index + 1;
      saved_poses = session.poses;
      tools::logger()->warn(
        "[Calib] 保留旧数据，从编号 {} 继续。已加载 {} 组可读姿态。",
        next_index, saved_poses.size());
      tools::logger()->warn(
        "[Calib] 只有在【标定板与相机都没有移动过】的前提下，新旧数据才能混用；\n"
        "        否则请改用 --on-existing=clear 重新采集，不然外参会静默算错。");

      if (!session.unreadable_poses.empty()) {
        tools::logger()->warn(
          "[Calib] 有 {} 组旧姿态无法解析（旧版四元数格式？），"
          "转角提示不会把它们计入；外参标定同样会跳过它们，但对应图片仍可用于内参。",
          session.unreadable_poses.size());
      }
    }
  }

  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  if (!gimbal.wait_for_first_frame(std::chrono::seconds(3))) {
    tools::logger()->error(
      "[Calib] 3 秒内没有收到云台下行帧，无法记录姿态。\n"
      "        外参标定必须同时记录云台角度，请先确认串口与下位机正常。");
    return 1;
  }

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  int count = 0;  // 本次会话保存的数量
  double min_pair_angle = min_pairwise_angle_deg(saved_poses);

  while (true) {
    camera.read(img, timestamp);

    auto state = gimbal.state();
    const Pose current{state.yaw, state.pitch, state.roll};

    // 这一张相对已存位姿的最大夹角：太小说明没带来新信息
    const double angle_to_saved = max_angle_to_saved_deg(saved_poses, current);

    auto drawing = img.clone();

    // 姿态读数
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", state.yaw), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", state.pitch), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", state.roll), {40, 120}, {0, 0, 255});

    // 转角提示
    tools::draw_text(
      drawing, fmt::format("saved {}  (this session {})", saved_poses.size(), count), {40, 170},
      {0, 255, 0});

    if (saved_poses.empty()) {
      tools::draw_text(
        drawing, "min pair angle: --  (first shot)", {40, 215}, {0, 255, 255});
    } else if (min_pair_angle >= GOOD_ANGLE_DEG) {
      tools::draw_text(
        drawing, fmt::format("min pair angle: {:.1f} deg  GOOD", min_pair_angle), {40, 215},
        {0, 255, 0});
    } else if (min_pair_angle >= MIN_USEFUL_ANGLE_DEG) {
      tools::draw_text(
        drawing, fmt::format("min pair angle: {:.1f} deg  OK (>=17.2)", min_pair_angle), {40, 215},
        {0, 255, 255});
    } else {
      tools::draw_text(
        drawing,
        fmt::format("min pair angle: {:.1f} deg  TOO SMALL (<17.2)", min_pair_angle), {40, 215},
        {0, 0, 255});
    }

    // 当前位姿的新颖度
    if (!saved_poses.empty()) {
      const auto color =
        angle_to_saved >= MIN_USEFUL_ANGLE_DEG ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
      tools::draw_text(
        drawing, fmt::format("vs saved (max): {:.1f} deg", angle_to_saved), {40, 255}, color);
    }

    // 标定板检测结果
    std::vector<cv::Point2f> centers_2d;
    bool found = false;
    if (use_circle)
      found = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);
    else
      found = cv::findChessboardCorners(img, pattern_size, centers_2d);

    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, found);
    cv::putText(
      drawing, found ? "PATTERN OK" : "NO PATTERN", {40, 300}, cv::FONT_HERSHEY_SIMPLEX, 0.9,
      found ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

    cv::resize(drawing, drawing, {}, 0.5, 0.5);
    cv::imshow("Press s to save, q to quit", drawing);

    const auto key = cv::waitKey(1);
    if (key == 'q') break;
    if (key != 's') continue;

    if (!found) {
      tools::logger()->warn("[Calib] 当前帧未检测到标定板，已跳过（不保存废图）。");
      continue;
    }

    // ★ 图片与角度必须是同一时刻的：这里用同一个 state 快照
    const auto img_path = fmt::format("{}/{}.jpg", output_folder, next_index);
    const auto ypr_path = fmt::format("{}/{}.txt", output_folder, next_index);
    if (!cv::imwrite(img_path, img)) {
      tools::logger()->error("[Calib] 写入图片失败：{}", img_path);
      continue;
    }
    if (!write_ypr(ypr_path, current)) {
      tools::logger()->error("[Calib] 写入姿态失败：{}", ypr_path);
      continue;
    }

    saved_poses.push_back(current);
    const double new_min_pair = min_pairwise_angle_deg(saved_poses);

    tools::logger()->info(
      "[{}] 已保存 yaw={:.2f} pitch={:.2f} roll={:.2f} | 与已存位姿最大夹角 {:.1f}° | "
      "最小两两夹角 {:.1f}°",
      next_index, current.yaw, current.pitch, current.roll, angle_to_saved, new_min_pair);

    if (new_min_pair < MIN_USEFUL_ANGLE_DEG && saved_poses.size() >= 2) {
      tools::logger()->warn(
        "[Calib] 最小两两夹角 {:.1f}° < {:.1f}°，TSAI 会丢弃这些运动对。\n"
        "        请把云台转得更开再采（例如 yaw 扫 ±30°、pitch 扫 ±15°）。",
        new_min_pair, MIN_USEFUL_ANGLE_DEG);
    }

    min_pair_angle = new_min_pair;
    next_index++;
    count++;
  }

  tools::logger()->info("[Calib] 本次采集 {} 组，累计可读位姿 {} 组。", count, saved_poses.size());
  if (!saved_poses.empty())
    tools::logger()->info("[Calib] 最终最小两两夹角 {:.1f}°。", min_pair_angle);

  if (saved_poses.size() < 15) {
    tools::logger()->warn(
      "[Calib] 累计只有 {} 组位姿。手眼标定建议 15~25 组。", saved_poses.size());
  }
  if (min_pair_angle < MIN_USEFUL_ANGLE_DEG && saved_poses.size() >= 2) {
    tools::logger()->error(
      "[Calib] 最小两两夹角 {:.1f}° 小于 {:.1f}°，外参标定很可能失败\n"
      "        （会打印 Not enough informative motions 并静默返回单位阵）。\n"
      "        请重新采集，让云台位姿更分散。",
      min_pair_angle, MIN_USEFUL_ANGLE_DEG);
  }

  return 0;
}
