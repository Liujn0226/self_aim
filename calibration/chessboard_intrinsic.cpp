#include "chessboard_common.hpp"

using namespace chesscal;

bool solve_intrinsic(const fs::path &dir, const YAML::Node &cfg, const Board &board) {
  std::vector<std::vector<cv::Point2f>> image_points;
  std::vector<std::vector<cv::Point3f>> object_points;
  std::vector<std::string> names;
  cv::Size size;
  for (const auto &file : images(dir)) {
    cv::Mat img = cv::imread(file.string());
    if (img.empty()) throw std::runtime_error("Cannot read " + file.string());
    check_size(size, img);
    metadata(dir, "intrinsic", board, size, false);
    std::vector<cv::Point2f> corners;
    if (!detect(img, board, corners)) {
      std::cerr << "SKIP no full chessboard: " << file << '\n';
      continue;
    }
    image_points.push_back(corners); object_points.push_back(board.points());
    names.push_back(file.filename().string());
  }
  if (image_points.size() < 6) {
    std::cerr << "Need at least 6 detected images; recommend 20-30. Found " << image_points.size() << '\n';
    return false;
  }
  if (image_points.size() < 20) std::cerr << "WARNING: fewer than 20 images. Collect varied positions/tilts.\n";
  cv::Mat K, D;
  std::vector<cv::Mat> rv, tv;
  // Same five-coefficient model consumed by the project; k3 is fixed to zero as in calib_camera.
  double rms = cv::calibrateCamera(object_points, image_points, size, K, D, rv, tv,
      cv::CALIB_FIX_K3, {cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 100, 1e-10});
  if (!std::isfinite(rms) || !cv::checkRange(K) || !cv::checkRange(D) ||
      K.at<double>(0,0) <= 0 || K.at<double>(1,1) <= 0 || D.total() != 5)
    throw std::runtime_error("Invalid intrinsic solution.");
  YAML::Node result;
  result["camera_matrix"] = flat(K); result["distort_coeffs"] = flat(D);
  result["image_width"] = size.width; result["image_height"] = size.height;
  result["rms_reprojection_px"] = rms;
  result["valid_images"] = int(image_points.size());
  for (size_t i = 0; i < names.size(); ++i) {
    double error = reprojection_rms(object_points[i], image_points[i], rv[i], tv[i], K, D);
    std::cout << names[i] << " RMS " << error << " px\n";
    YAML::Node entry; entry["image"] = names[i]; entry["rms_px"] = error;
    result["per_image"].push_back(entry);
  }
  std::cout << "Global RMS: " << rms << " px. Review per-image errors and image coverage.\n";
  save_result(dir, "intrinsic_result.yaml", cfg, result,
      {"camera_matrix", "distort_coeffs", "image_width", "image_height"});
  return true;
}

int main(int argc, char **argv) {
  try {
    cv::CommandLineParser cli(argc, argv, cli_keys);
    if (cli.get<bool>("help")) { cli.printMessage(); return 0; }
    const std::string config = cli.get<std::string>(0);
    const std::string folder = cli.get<std::string>("output");
    const bool offline = cli.get<bool>("solve");
    if (!cli.check()) { cli.printErrors(); return 1; }
    if (folder.empty()) throw std::runtime_error("Specify -o=/path/to/intrinsic_images.");
    const fs::path dir = fs::absolute(folder);
    const YAML::Node cfg = YAML::LoadFile(config);
    const Board board(cfg);
    if (offline) return solve_intrinsic(dir, cfg, board) ? 0 : 1;
    fs::create_directories(dir);
    int index = next_index(dir);
    std::cout << "INTRINSIC: camera only; move the board. s=save, c=compute, q=quit.\n"
              << "Images: " << dir << "\n";
    io::Camera camera(config); // Deliberately no Gimbal / serial port.
    cv::Size size;
    while (true) {
      cv::Mat img; std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);
      if (img.empty()) continue;
      check_size(size, img);
      metadata(dir, "intrinsic", board, size, true);
      std::vector<cv::Point2f> corners;
      bool found = detect(img, board, corners);
      show("Intrinsic: s save / c compute / q quit", img, board, corners, found,
          "Next image: " + std::to_string(index) + "   s=save  c=compute  q=quit",
          "Move board: center, edges, distances, horizontal and vertical tilts.");
      int key = cv::waitKey(1) & 0xff;
      if (key == 'q') break;
      if (key == 'c') { solve_intrinsic(dir, cfg, board); continue; }
      if (key != 's') continue;
      if (!found) { std::cerr << "Not saved: NO PATTERN.\n"; continue; }
      fs::path file = dir / (std::to_string(index) + ".jpg");
      if (fs::exists(file)) throw std::runtime_error("Refusing to overwrite " + file.string());
      if (!cv::imwrite(file.string(), img)) throw std::runtime_error("Cannot save image.");
      std::cout << "Saved " << file << '\n'; ++index;
    }
    cv::destroyAllWindows();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << "ERROR: " << e.what() << '\n'; return 1;
  }
}
