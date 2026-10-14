#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <algorithm>
#include <cmath>
#include <fstream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

#include "io/gimbal/gimbal.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                          | 输出命令行参数说明}"
  "{config-path c  | configs/calibration_contest.yaml | yaml配置文件路径 }"
  "{@input-folder  | calib_imgs               | 输入文件夹路径   }";

std::vector<cv::Point3f> centers_3d(const cv::Size & pattern_size, const float center_distance)
{
  std::vector<cv::Point3f> centers_3d;

  for (int i = 0; i < pattern_size.height; i++)
    for (int j = 0; j < pattern_size.width; j++)
      centers_3d.push_back({j * center_distance, i * center_distance, 0});

  return centers_3d;
}

/// 读取一帧云台姿态，单位：度，顺序 yaw / pitch / roll。
/// 与 calib_capture.cpp 的 write_ypr() 对应。
/// 注意：新协议没有四元数，因此不再需要 R_gimbal2imubody 换算，
///       直接用 tools::rotation_matrix({yaw,pitch,roll}) 得到 R_gimbal2world，
///       与原实现的 Rᵀ·q·R（R 为单位阵时退化为 q.toRotationMatrix()）等价。
io::Attitude read_ypr_deg(const std::string & ypr_path)
{
  std::ifstream file(ypr_path);
  double yaw = 0, pitch = 0, roll = 0;
  file >> yaw >> pitch >> roll;
  io::Attitude a;
  a.yaw = yaw;
  a.pitch = pitch;
  a.roll = roll;
  return a;
}

bool load(
  const std::string & input_folder, const std::string & config_path,
  std::vector<cv::Mat> & R_gimbal2world_list, std::vector<cv::Mat> & t_gimbal2world_list,
  std::vector<cv::Mat> & rvecs, std::vector<cv::Mat> & tvecs)
{
  // 读取yaml参数
  auto yaml = YAML::LoadFile(config_path);
  auto pattern_cols = yaml["pattern_cols"].as<int>();
  auto pattern_rows = yaml["pattern_rows"].as<int>();
  auto center_distance_mm = yaml["center_distance_mm"].as<double>();
  auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
  auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

  cv::Size pattern_size(pattern_cols, pattern_rows);
  const std::string pattern_type =
    yaml["pattern_type"] ? yaml["pattern_type"].as<std::string>() : "circle";
  const bool use_circle = (pattern_type == "circle");

  // camera_matrix 支持展平的 9 个数（与 configs/*.yaml 及 calib_camera 的输出一致）。
  // 也容忍 16 个数的写法（Eigen 风格 4x4），此时取左上 3x3。
  cv::Mat camera_matrix;
  if (camera_matrix_data.size() == 9) {
    camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
  } else if (camera_matrix_data.size() == 16) {
    cv::Mat m4(4, 4, CV_64F, camera_matrix_data.data());
    camera_matrix = m4(cv::Rect(0, 0, 3, 3)).clone();
  } else {
    fmt::print(
      "[ERROR] camera_matrix 需要展平的 9 个数，实际 {} 个。\n"
      "        请先把 calib_camera 输出的结果填回 yaml 再运行本程序。\n",
      camera_matrix_data.size());
    return false;
  }
  cv::Mat distort_coeffs(distort_coeffs_data);

  for (int i = 1; true; i++) {
    // 读取图片和对应云台姿态
    auto img_path = fmt::format("{}/{}.jpg", input_folder, i);
    auto ypr_path = fmt::format("{}/{}.txt", input_folder, i);
    auto img = cv::imread(img_path);
    if (img.empty()) break;
    const auto attitude = read_ypr_deg(ypr_path);

    // 云台 -> 世界：新协议直接给 yaw/pitch/roll，不再经过四元数
    Eigen::Matrix3d R_gimbal2world =
      tools::rotation_matrix(Eigen::Vector3d{attitude.yaw, attitude.pitch, attitude.roll} * (M_PI / 180.0));
    Eigen::Vector3d ypr = tools::eulers(R_gimbal2world, 2, 1, 0) * 57.3;  // degree

    // 在图片上显示云台的欧拉角，用来确认角度记录正确、无零漂
    auto drawing = img.clone();
    tools::draw_text(drawing, fmt::format("yaw   {:.2f}", ypr[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("pitch {:.2f}", ypr[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(drawing, fmt::format("roll  {:.2f}", ypr[2]), {40, 120}, {0, 0, 255});

    // 识别标定板
    std::vector<cv::Point2f> centers_2d;
    bool success = false;
    if (use_circle)
      success = cv::findCirclesGrid(img, pattern_size, centers_2d, cv::CALIB_CB_SYMMETRIC_GRID);
    else
      success = cv::findChessboardCorners(img, pattern_size, centers_2d);

    // 显示识别结果
    cv::drawChessboardCorners(drawing, pattern_size, centers_2d, success);
    cv::resize(drawing, drawing, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("Press any to continue", drawing);
    cv::waitKey(0);

    // 输出识别结果
    fmt::print("[{}] {}\n", success ? "success" : "failure", img_path);
    if (!success) continue;

    // 计算所需的数据
    // t_gimbal2world 恒为 0，因为云台只转动、不平移
    cv::Mat t_gimbal2world = (cv::Mat_<double>(3, 1) << 0, 0, 0);
    cv::Mat R_gimbal2world_cv;
    cv::eigen2cv(R_gimbal2world, R_gimbal2world_cv);
    cv::Mat rvec, tvec;
    auto centers_3d_ = centers_3d(pattern_size, center_distance_mm);
    cv::solvePnP(
      centers_3d_, centers_2d, camera_matrix, distort_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE);

    // 记录所需的数据
    R_gimbal2world_list.emplace_back(R_gimbal2world_cv);
    t_gimbal2world_list.emplace_back(t_gimbal2world);
    rvecs.emplace_back(rvec);
    tvecs.emplace_back(tvec);
  }

  fmt::print("\n[load] 共读到 {} 组有效位姿。\n", R_gimbal2world_list.size());
  return true;
}

void print_result(const cv::Mat & R_camera2gimbal, const cv::Mat & t_camera2gimbal)
{
  auto flow = [](const std::vector<double> & v) {
    std::string s = "[";
    for (size_t i = 0; i < v.size(); i++)
      s += fmt::format("{}{:.12g}", i ? ", " : "", v[i]);
    return s + "]";
  };

  std::vector<double> R_data(R_camera2gimbal.begin<double>(), R_camera2gimbal.end<double>());
  std::vector<double> t_data(t_camera2gimbal.begin<double>(), t_camera2gimbal.end<double>());

  fmt::print("\n===== 复制下面两行到 configs/contest.yaml =====\n");
  fmt::print("R_camera2gimbal: {}\n", flow(R_data));
  fmt::print("t_camera2gimbal: {}\n", flow(t_data));
  fmt::print("===============================================\n");

  // 打印相机轴相对云台轴的角度，用于判断安装是否合理、结果是否可信
  Eigen::Matrix3d R;
  cv::cv2eigen(R_camera2gimbal, R);
  const Eigen::Vector3d cam_x = R.col(0), cam_y = R.col(1), cam_z = R.col(2);
  const Eigen::Vector3d g_x = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d g_y = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d g_z = Eigen::Vector3d::UnitZ();
  const auto deg = [](const Eigen::Vector3d & a, const Eigen::Vector3d & b) {
    return std::acos(std::clamp(a.dot(b), -1.0, 1.0)) * 57.29577951308232;
  };

  fmt::print("\n相机轴在云台系中的方向（用于判断结果是否合理）：\n");
  fmt::print("  相机 x  {:+7.2f}° 相对云台 +x / {:+7.2f}° 相对 -y\n", deg(cam_x, g_x), deg(cam_x, -g_y));
  fmt::print("  相机 y  {:+7.2f}° 相对云台 -y / {:+7.2f}° 相对 -z\n", deg(cam_y, -g_y), deg(cam_y, -g_z));
  fmt::print("  光轴 z  {:+7.2f}° 相对云台 +x（枪管方向）\n", deg(cam_z, g_x));
  fmt::print("  相机位置  {:+.4f} {:+.4f} {:+.4f} m（相对云台原点）\n", t_data[0], t_data[1],
             t_data[2]);
  fmt::print("\n判读：光轴相对枪管的偏角通常应在 1~2° 量级。\n");
  fmt::print("      若大到 7° 以上，说明机械安装偏差大或标定数据有问题，务必用\n");
  fmt::print("      solver.world2pixel() 把已知世界点重投影回图像验证一次。\n\n");
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_folder = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");

  // 从输入文件夹中加载标定所需的数据
  std::vector<cv::Mat> R_gimbal2world_list, t_gimbal2world_list;
  std::vector<cv::Mat> rvecs, tvecs;
  if (!load(input_folder, config_path, R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs))
    return 1;

  if (R_gimbal2world_list.size() < 3) {
    fmt::print("[ERROR] 有效位姿只有 {} 组，手眼标定至少需要 3 组。\n", R_gimbal2world_list.size());
    return 1;
  }

  // 手眼标定。
  // 第 1 次不指定方法，完全复现原实现的默认行为（OpenCV 默认是 TSAI）。
  // 然后逐个方法跑一遍做对比：不同方法鲁棒性差别很大。
  //   特别注意：TSAI 会丢弃旋转过小的运动对（约小于 17°），
  //   本云台只有 2 个自由度，位姿不够散开时 TSAI 可能只剩不到 2 对有效运动，
  //   此时函数会打印 "Not enough informative motions" 并返回无效结果，
  //   换成 PARK / HORAUD / DANIILIDIS 往往仍然可用。
  cv::Mat R_default, t_default;
  cv::calibrateHandEye(
    R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs, R_default, t_default);
  fmt::print("\n===== 方法 DEFAULT（不指定 method，与改前的原实现一致） =====");
  print_result(R_default, t_default / 1e3);  // mm -> m

  const std::vector<std::pair<std::string, cv::HandEyeCalibrationMethod>> methods = {
    {"TSAI", cv::CALIB_HAND_EYE_TSAI},
    {"PARK", cv::CALIB_HAND_EYE_PARK},
    {"HORAUD", cv::CALIB_HAND_EYE_HORAUD},
    {"ANDREFF", cv::CALIB_HAND_EYE_ANDREFF},
    {"DANIILIDIS", cv::CALIB_HAND_EYE_DANIILIDIS},
  };

  for (const auto & [name, method] : methods) {
    cv::Mat R, t;
    try {
      cv::calibrateHandEye(
        R_gimbal2world_list, t_gimbal2world_list, rvecs, tvecs, R, t, method);
    } catch (const std::exception & e) {
      fmt::print("\n----- 方法 {} 抛异常: {} -----\n", name, e.what());
      continue;
    }
    if (R.empty() || t.empty()) {
      fmt::print("\n----- 方法 {} 未返回有效结果 -----\n", name);
      continue;
    }
    fmt::print("\n===== 方法 {} =====", name);
    print_result(R, t / 1e3);  // mm -> m
  }

  fmt::print(
    "\n\n########## 上面每个方法都打印了一份可粘贴的结果 ##########\n"
    "如何选择：优先用 DEFAULT / TSAI 的结果；如果它失败或与其它方法差很多，\n"
    "说明位姿散开程度不够。请重新采集：云台 yaw 覆盖 ±30°、pitch 覆盖 ±15°，\n"
    "标定板放在不同距离（1/2/3m）和不同画面位置，并让角点铺满画面。\n"
    "判定最终结果是否可信，必须用 solver.world2pixel() 把已知世界点重投影回图像验证。\n\n");

  return 0;
}
