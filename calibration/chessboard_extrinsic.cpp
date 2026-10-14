#include "chessboard_common.hpp"
#include "io/gimbal/gimbal.hpp"

#include <deque>
#include <memory>

using namespace chesscal;

struct Pose { double yaw, pitch, roll; };
Pose pose(const io::GimbalState &s) { return {s.yaw, s.pitch, s.roll}; }
cv::Mat rot(const Pose &p) { return rotation(p.yaw, p.pitch, p.roll); }
Pose read_pose(const fs::path &file) {
  std::ifstream in(file);
  Pose p{}; double extra;
  if (!(in >> p.yaw >> p.pitch >> p.roll) || (in >> extra) ||
      !std::isfinite(p.yaw) || !std::isfinite(p.pitch) || !std::isfinite(p.roll))
    throw std::runtime_error("Missing/invalid yaw pitch roll (degrees): " + file.string());
  in.clear();
  in >> std::ws;
  if (in.peek() != std::char_traits<char>::eof())
    throw std::runtime_error("Trailing non-numeric pose data: " + file.string());
  return p;
}
void intrinsics(const YAML::Node &cfg, cv::Mat &K, cv::Mat &D) {
  K = read_matrix(cfg["camera_matrix"], 3, 3);
  auto v = cfg["distort_coeffs"].as<std::vector<double>>();
  if (v.size() != 5) throw std::runtime_error("Expected 5 distortion coefficients from intrinsic program.");
  D = cv::Mat(v).clone();
  if (!cv::checkRange(D) || K.at<double>(0,0) <= 0 || K.at<double>(1,1) <= 0)
    throw std::runtime_error("Invalid intrinsics. Run intrinsic calibration first.");
}
void intrinsic_size(const YAML::Node &cfg, cv::Size size) {
  if (!cfg["image_width"] || !cfg["image_height"])
    throw std::runtime_error("Use intrinsic updated_calibration.yaml (includes calibrated image size).");
  if (cfg["image_width"].as<int>() != size.width || cfg["image_height"].as<int>() != size.height)
    throw std::runtime_error("Camera resolution differs from intrinsic calibration.");
}

bool solve_extrinsic(const fs::path &dir, const YAML::Node &cfg, const Board &board) {
  cv::Mat K, D; intrinsics(cfg, K, D);
  std::vector<cv::Mat> Rg, tg, Rc, tc;
  cv::Size size;
  const auto obj = board.points();
  for (const auto &file : images(dir)) {
    cv::Mat img = cv::imread(file.string());
    if (img.empty()) throw std::runtime_error("Cannot read " + file.string());
    check_size(size, img); intrinsic_size(cfg, size);
    metadata(dir, "extrinsic", board, size, false);
    fs::path posefile = file; posefile.replace_extension(".txt");
    Pose p = read_pose(posefile); // Never substitute zero for missing pose.
    std::vector<cv::Point2f> corners;
    if (!detect(img, board, corners)) {
      std::cerr << "SKIP no full chessboard: " << file << '\n'; continue;
    }
    cv::Mat rv, tv, R;
    if (!cv::solvePnP(obj, corners, K, D, rv, tv, false, cv::SOLVEPNP_ITERATIVE) ||
        !cv::checkRange(rv) || !cv::checkRange(tv))
      throw std::runtime_error("PnP failed: " + file.string());
    cv::Rodrigues(rv, R);
    for (const auto &pt : obj) {
      cv::Mat x = (cv::Mat_<double>(3,1) << pt.x, pt.y, pt.z);
      cv::Mat xc = R * x + tv;
      if (xc.at<double>(2) <= 0) throw std::runtime_error("PnP board behind camera.");
    }
    const double error = reprojection_rms(obj, corners, rv, tv, K, D);
    std::cout << file.filename() << " PnP RMS " << error << " px\n";
    if (error > 2.0) { std::cerr << "SKIP: PnP RMS above 2 px.\n"; continue; }
    Rg.push_back(rot(p)); tg.push_back(cv::Mat::zeros(3,1,CV_64F));
    Rc.push_back(R); tc.push_back(tv);
  }
  if (Rg.size() < 8) {
    std::cerr << "Need >=8 valid poses; recommend 20-30. Found " << Rg.size() << '\n';
    return false;
  }
  // Require excitation of more than one axis. Thresholds are program quality gates,
  // not universal OpenCV requirements. Do not require every pose pair to exceed 17 degrees.
  cv::Mat axes, system;
  double maximum = 0;
  for (size_t i = 0; i < Rg.size(); ++i) for (size_t j = i+1; j < Rg.size(); ++j) {
    cv::Mat relative = Rg[j].t() * Rg[i], rv;
    maximum = std::max(maximum, rotation_angle(relative));
    cv::Rodrigues(relative, rv);
    axes.push_back(rv.t());
    cv::Mat block = relative - cv::Mat::eye(3,3,CV_64F);
    system.push_back(block);
  }
  cv::SVD a(axes, cv::SVD::NO_UV), s(system, cv::SVD::NO_UV);
  const double second_ratio = a.w.at<double>(1) / std::max(a.w.at<double>(0), 1e-12);
  const double translation_ratio = s.w.at<double>(2) / std::max(s.w.at<double>(0), 1e-12);
  if (maximum < 10 || second_ratio < .05 || translation_ratio < .02) {
    std::cerr << "Insufficient rotation diversity. Need yaw AND pitch, with combined poses.\n"
              << "Max rotation " << maximum << " deg, second-axis ratio " << second_ratio
              << ", translation ratio " << translation_ratio << '\n';
    return false;
  }
  cv::Mat R, t;
  cv::calibrateHandEye(Rg, tg, Rc, tc, R, t, cv::CALIB_HAND_EYE_PARK);
  if (R.empty() || t.empty() || !cv::checkRange(R) || !cv::checkRange(t) ||
      std::abs(cv::determinant(R)-1.0) > 1e-3 ||
      cv::norm(R.t()*R-cv::Mat::eye(3,3,CV_64F)) > 1e-3)
    throw std::runtime_error("Hand-eye solver returned an invalid rigid transform.");
  // With a fixed board and fixed gimbal origin, world<-board must stay constant.
  std::vector<cv::Mat> board_R, board_t;
  cv::Mat mean_R = cv::Mat::zeros(3,3,CV_64F), mean_t = cv::Mat::zeros(3,1,CV_64F);
  for (size_t i = 0; i < Rg.size(); ++i) {
    cv::Mat wr = Rg[i] * R * Rc[i];
    cv::Mat wt = Rg[i] * (R * tc[i] + t);
    board_R.push_back(wr); board_t.push_back(wt); mean_R += wr; mean_t += wt;
  }
  mean_t /= double(Rg.size());
  cv::SVD avg(mean_R, cv::SVD::FULL_UV);
  cv::Mat correction = cv::Mat::eye(3,3,CV_64F);
  correction.at<double>(2,2) = cv::determinant(avg.u * avg.vt) < 0 ? -1.0 : 1.0;
  mean_R = avg.u * correction * avg.vt;
  double rotation_sq = 0, translation_sq = 0;
  for (size_t i = 0; i < Rg.size(); ++i) {
    double angle = rotation_angle(mean_R.t() * board_R[i]);
    double shift = cv::norm(board_t[i] - mean_t);
    rotation_sq += angle*angle; translation_sq += shift*shift;
  }
  double rotation_rms = std::sqrt(rotation_sq/Rg.size());
  double translation_rms = std::sqrt(translation_sq/Rg.size());
  YAML::Node diagnostics;
  diagnostics["valid_poses"] = int(Rg.size());
  diagnostics["fixed_board_rotation_rms_deg"] = rotation_rms;
  diagnostics["fixed_board_translation_rms_mm"] = translation_rms;
  diagnostics["max_relative_rotation_deg"] = maximum;
  diagnostics["second_axis_ratio"] = second_ratio;
  diagnostics["translation_condition_ratio"] = translation_ratio;
  const bool accepted = rotation_rms <= 3.0 && translation_rms <= 30.0;
  diagnostics["accepted"] = accepted;
  write_yaml(dir / "extrinsic_diagnostics.yaml", diagnostics);
  std::cout << "Fixed-board scatter: " << rotation_rms << " deg RMS / "
            << translation_rms << " mm RMS\n";
  if (!accepted) {
    std::cerr << "Rejected: scatter exceeds 3 deg / 30 mm. Check board motion, pose signs/order,\n"
              << "camera mounting, PnP corner order and fixed-origin model. No new result written.\n";
    return false;
  }
  YAML::Node result = YAML::Clone(diagnostics);
  result["R_camera2gimbal"] = flat(R);
  cv::Mat t_m = t / 1000.0;
  result["t_camera2gimbal"] = flat(t_m);
  result["translation_unit"] = "m";
  result["method"] = "PARK";
  save_result(dir, "extrinsic_result.yaml", cfg, result, {"R_camera2gimbal", "t_camera2gimbal"});
  std::cout << "Convention: p_gimbal = R_camera2gimbal*p_camera + t_camera2gimbal (meters).\n"
            << "Passed internal consistency gates; validate on independent poses before use.\n";
  return true;
}

int main(int argc, char **argv) {
  try {
    cv::CommandLineParser cli(argc, argv, cli_keys);
    if (cli.get<bool>("help")) { cli.printMessage(); return 0; }
    const std::string config = cli.get<std::string>(0), folder = cli.get<std::string>("output");
    const bool offline = cli.get<bool>("solve");
    if (!cli.check()) { cli.printErrors(); return 1; }
    if (folder.empty()) throw std::runtime_error("Specify -o=/path/to/extrinsic_images.");
    const fs::path dir = fs::absolute(folder);
    const YAML::Node cfg = YAML::LoadFile(config);
    const Board board(cfg);
    cv::Mat K, D; intrinsics(cfg, K, D);
    if (offline) return solve_extrinsic(dir, cfg, board) ? 0 : 1;
    // A new folder per external capture session prevents mixing moved-board datasets.
    if (!images(dir).empty()) throw std::runtime_error("External capture requires an empty image folder. Use --solve=true to recompute.");
    fs::create_directories(dir);
    io::Gimbal gimbal(config);
    if (!gimbal.wait_for_first_frame(std::chrono::seconds(3)))
      throw std::runtime_error("No gimbal downlink. Check serial port and firmware.");
    io::Camera camera(config);
    std::cout << "EXTRINSIC: keep board AND robot base fixed; rotate gimbal yaw and pitch.\n"
              << "Camera must remain rigidly mounted. No motor/fire commands are sent.\n"
              << "s=save (hold still >=0.5s), c=compute, q=quit\n";
    int index = 1;
    cv::Size size;
    using Clock = std::chrono::steady_clock;
    std::deque<std::pair<Clock::time_point, Pose>> history;
    uint64_t last_rx = gimbal.received_frames();
    auto last_update = Clock::now();
    std::vector<cv::Mat> saved;
    while (true) {
      cv::Mat img; Clock::time_point timestamp;
      camera.read(img, timestamp);
      if (img.empty()) continue;
      if (!gimbal.wait_for_first_frame(std::chrono::milliseconds(1))) {
        std::cerr << "Waiting for renewed gimbal downlink after reconnect.\n";
        continue;
      }
      check_size(size, img); intrinsic_size(cfg, size);
      metadata(dir, "extrinsic", board, size, true);
      auto now = Clock::now();
      auto rx = gimbal.received_frames();
      if (rx != last_rx) { last_update = now; last_rx = rx; }
      Pose p = pose(gimbal.state());
      history.emplace_back(now, p);
      while (history.size() > 2 && now-history[1].first > std::chrono::milliseconds(600))
        history.pop_front();
      bool stable = now-history.front().first >= std::chrono::milliseconds(500);
      cv::Mat current_R = rot(p);
      for (const auto &sample : history)
        stable = stable && rotation_angle(current_R.t()*rot(sample.second)) < .15;
      std::vector<cv::Point2f> corners;
      bool found = detect(img, board, corners);
      std::ostringstream text;
      text << std::fixed << std::setprecision(2) << "yaw=" << p.yaw << " pitch=" << p.pitch
           << " roll=" << p.roll << "  saved=" << index-1;
      show("Extrinsic: s save / c compute / q quit", img, board, corners, found,
          text.str(), stable ? "HOLD STILL: ready  s=save c=compute q=quit" : "Wait: hold gimbal still >=0.5s");
      int key = cv::waitKey(1) & 0xff;
      if (key == 'q') break;
      if (key == 'c') { solve_extrinsic(dir, cfg, board); continue; }
      if (key != 's') continue;
      if (!found || !stable) { std::cerr << "Not saved: full board and still pose required.\n"; continue; }
      if (Clock::now()-last_update > std::chrono::milliseconds(300) ||
          Clock::now()-timestamp > std::chrono::milliseconds(300)) {
        std::cerr << "Not saved: camera frame or gimbal downlink too old (>300ms).\n"; continue;
      }
      Pose after = pose(gimbal.state());
      if (rotation_angle(current_R.t()*rot(after)) > .15) {
        std::cerr << "Not saved: gimbal moved during image processing.\n"; continue;
      }
      bool duplicate = false;
      for (const auto &R : saved) if (rotation_angle(R.t()*current_R) < 2) duplicate = true;
      if (duplicate) { std::cerr << "Not saved: pose within 2 deg of an existing pose.\n"; continue; }
      fs::path jpg = dir/(std::to_string(index)+".jpg"), txt = dir/(std::to_string(index)+".txt");
      if (fs::exists(jpg) || fs::exists(txt)) throw std::runtime_error("Refusing to overwrite sample.");
      if (!cv::imwrite(jpg.string(), img)) throw std::runtime_error("Cannot save image.");
      std::ofstream out(txt);
      out << std::setprecision(12) << p.yaw << '\n' << p.pitch << '\n' << p.roll << '\n';
      out.close();
      if (!out) {
        fs::remove(jpg); fs::remove(txt);
        throw std::runtime_error("Could not save paired pose; image removed.");
      }
      saved.push_back(current_R.clone());
      std::cout << "Saved " << jpg << " and " << txt << '\n'; ++index;
    }
    cv::destroyAllWindows(); return 0;
  } catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << '\n'; return 1;
  }
}
