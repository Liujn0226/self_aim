#pragma once

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "io/camera.hpp"

namespace chesscal {
namespace fs = std::filesystem;
constexpr double deg2rad = CV_PI / 180.0;

struct Board {
  int cols = 0, rows = 0;
  double spacing_mm = 0;
  explicit Board(const YAML::Node &cfg) {
    if (!cfg["pattern_type"] || cfg["pattern_type"].as<std::string>() != "chessboard")
      throw std::runtime_error("Set pattern_type: chessboard in the config.");
    cols = cfg["pattern_cols"].as<int>();
    rows = cfg["pattern_rows"].as<int>();
    spacing_mm = cfg["center_distance_mm"].as<double>();
    if (cols < 3 || rows < 3 || !std::isfinite(spacing_mm) || spacing_mm <= 0)
      throw std::runtime_error("Invalid internal corner counts / square size (mm).");
  }
  std::vector<cv::Point3f> points() const {
    std::vector<cv::Point3f> p;
    for (int r = 0; r < rows; ++r)
      for (int c = 0; c < cols; ++c)
        p.emplace_back(float(c * spacing_mm), float(r * spacing_mm), 0.f);
    return p;
  }
};

inline bool detect(const cv::Mat &img, const Board &b, std::vector<cv::Point2f> &corners) {
  cv::Mat gray;
  if (img.channels() == 1) gray = img;
  else cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  const bool found = cv::findChessboardCorners(gray, {b.cols, b.rows}, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
  if (found)
    cv::cornerSubPix(gray, corners, {11, 11}, {-1, -1},
        {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 40, 0.001});
  return found;
}

inline std::vector<fs::path> images(const fs::path &dir) {
  std::vector<fs::path> files;
  if (!fs::exists(dir)) return files;
  for (const auto &e : fs::directory_iterator(dir)) {
    auto stem = e.path().stem().string();
    if (e.is_regular_file() && e.path().extension() == ".jpg" && !stem.empty() &&
        std::all_of(stem.begin(), stem.end(), [](unsigned char c) { return c >= '0' && c <= '9'; }))
      files.push_back(e.path());
  }
  std::sort(files.begin(), files.end(), [](const auto &a, const auto &b) {
    return std::stoll(a.stem().string()) < std::stoll(b.stem().string());
  });
  return files;
}
inline int next_index(const fs::path &dir) {
  auto files = images(dir);
  return files.empty() ? 1 : std::stoi(files.back().stem().string()) + 1;
}
inline void write_yaml(const fs::path &file, const YAML::Node &node) {
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(16);
  emitter << node;
  if (!emitter.good()) throw std::runtime_error("Cannot serialize YAML: " + file.string());
  std::ofstream out(file);
  if (!out || !(out << emitter.c_str() << '\n'))
    throw std::runtime_error("Cannot write: " + file.string());
}
inline YAML::Node flat(const cv::Mat &src) {
  cv::Mat m; src.convertTo(m, CV_64F);
  YAML::Node n(YAML::NodeType::Sequence);
  n.SetStyle(YAML::EmitterStyle::Flow);
  for (int r = 0; r < m.rows; ++r)
    for (int c = 0; c < m.cols; ++c) n.push_back(m.at<double>(r, c));
  return n;
}
inline cv::Mat read_matrix(const YAML::Node &n, int rows, int cols) {
  auto v = n.as<std::vector<double>>();
  if (int(v.size()) != rows * cols) throw std::runtime_error("Wrong matrix size in config.");
  cv::Mat m(rows, cols, CV_64F);
  std::copy(v.begin(), v.end(), m.ptr<double>());
  if (!cv::checkRange(m)) throw std::runtime_error("Non-finite matrix in config.");
  return m;
}
inline double rotation_angle(const cv::Mat &R) {
  return std::acos(std::clamp((cv::trace(R)[0] - 1.0) / 2.0, -1.0, 1.0)) / deg2rad;
}
// Same convention as tools::rotation_matrix and contest Solver: Rz(yaw)*Ry(pitch)*Rx(roll).
inline cv::Mat rotation(double yaw, double pitch, double roll) {
  yaw *= deg2rad; pitch *= deg2rad; roll *= deg2rad;
  double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);
  return (cv::Mat_<double>(3,3) << cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
      sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, -sp, cp*sr, cp*cr);
}
inline double reprojection_rms(const std::vector<cv::Point3f> &obj,
    const std::vector<cv::Point2f> &pts, const cv::Mat &rv, const cv::Mat &tv,
    const cv::Mat &K, const cv::Mat &D) {
  std::vector<cv::Point2f> projected;
  cv::projectPoints(obj, rv, tv, K, D, projected);
  return cv::norm(pts, projected, cv::NORM_L2) / std::sqrt(double(pts.size()));
}
inline void show(const std::string &window, const cv::Mat &img, const Board &board,
    const std::vector<cv::Point2f> &corners, bool found, const std::string &status,
    const std::string &extra = "") {
  cv::Mat view = img.clone();
  if (view.channels() == 1) cv::cvtColor(view, view, cv::COLOR_GRAY2BGR);
  cv::drawChessboardCorners(view, {board.cols, board.rows}, corners, found);
  const double scale = std::min(1.0, 1100.0 / view.cols);
  if (scale < 1) cv::resize(view, view, {}, scale, scale);
  cv::putText(view, found ? "PATTERN OK" : "NO PATTERN", {15, 30},
      cv::FONT_HERSHEY_SIMPLEX, .7, found ? cv::Scalar(0,255,0) : cv::Scalar(0,0,255), 2);
  cv::putText(view, status, {15,60}, cv::FONT_HERSHEY_SIMPLEX, .6, {0,255,255}, 1);
  cv::putText(view, extra, {15,90}, cv::FONT_HERSHEY_SIMPLEX, .5, {0,255,255}, 1);
  cv::imshow(window, view);
}
inline void metadata(const fs::path &dir, const std::string &kind,
    const Board &b, cv::Size size, bool allow_create) {
  auto file = dir / "session.yaml";
  if (fs::exists(file)) {
    auto n = YAML::LoadFile(file.string());
    if (kind == "extrinsic" && (!n["pose_convention"] ||
        n["pose_convention"].as<std::string>() != "raw_degrees_ZYX_fixed_origin"))
      throw std::runtime_error("External pose convention metadata missing / incompatible.");
    if (n["kind"].as<std::string>() != kind || n["pattern_cols"].as<int>() != b.cols ||
        n["pattern_rows"].as<int>() != b.rows ||
        std::abs(n["center_distance_mm"].as<double>() - b.spacing_mm) > 1e-6 ||
        n["image_width"].as<int>() != size.width || n["image_height"].as<int>() != size.height)
      throw std::runtime_error("Session board / image size / mode mismatch. Use a new folder.");
  } else if (kind == "extrinsic" && !allow_create) {
    throw std::runtime_error("External images require this program session.yaml and raw pose convention.");
  } else if (allow_create) {
    if (!images(dir).empty())
      throw std::runtime_error("Existing images have no session.yaml. Use a new folder for capture.");
    YAML::Node n;
    n["kind"] = kind;
    if (kind == "extrinsic") n["pose_convention"] = "raw_degrees_ZYX_fixed_origin";
    n["pattern_cols"] = b.cols; n["pattern_rows"] = b.rows;
    n["center_distance_mm"] = b.spacing_mm;
    n["image_width"] = size.width; n["image_height"] = size.height;
    write_yaml(file, n);
  }
}
inline void check_size(cv::Size &size, const cv::Mat &img) {
  if (size.area() == 0) size = img.size();
  if (img.size() != size) throw std::runtime_error("Mixed image sizes. Use one resolution.");
}
inline std::string flow_string(const YAML::Node &n) {
  YAML::Emitter e; e.SetDoublePrecision(16); e << n; return e.c_str();
}
inline void save_result(const fs::path &dir, const std::string &filename,
    const YAML::Node &cfg, const YAML::Node &result, const std::vector<std::string> &keys) {
  write_yaml(dir / filename, result);
  auto updated = YAML::Clone(cfg);
  for (const auto &key : keys) {
    updated[key] = result[key];
    if (key != "image_width" && key != "image_height")
      std::cout << key << ": " << flow_string(result[key]) << '\n';
  }
  write_yaml(dir / "updated_calibration.yaml", updated);
  std::cout << "Saved " << (dir / filename) << "\nFull calibration config: "
            << (dir / "updated_calibration.yaml") << '\n';
}
inline const std::string cli_keys =
    "{help h | false | help}"
    "{@config | configs/calibration_contest.yaml | config YAML}"
    "{output o | | sample directory (required)}"
    "{solve | false | solve existing images without opening devices}";
} // namespace chesscal
