#include "ceres_cam_imu/io/kalibr_result_parser.h"

#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace ceres_cam_imu {
namespace {

std::vector<double> extractDoubles(const std::string& line) {
  static const std::regex number_re(
      R"([-+]?(?:(?:\d+\.\d*)|(?:\.\d+)|(?:\d+))(?:[eE][-+]?\d+)?)");
  std::vector<double> values;
  for (auto it = std::sregex_iterator(line.begin(), line.end(), number_re);
       it != std::sregex_iterator(); ++it) {
    values.push_back(std::stod(it->str()));
  }
  return values;
}

Mat4 readMatrix4(const std::vector<std::string>& lines, std::size_t first_row) {
  Mat4 matrix = Mat4::Identity();
  for (int r = 0; r < 4; ++r) {
    const std::vector<double> values = extractDoubles(lines.at(first_row + r));
    if (values.size() < 4) {
      throw std::runtime_error("failed to parse 4x4 matrix row from Kalibr result");
    }
    for (int c = 0; c < 4; ++c) {
      matrix(r, c) = values[static_cast<std::size_t>(c)];
    }
  }
  return matrix;
}

double readMeanAfterToken(const std::string& line) {
  const std::size_t pos = line.find("mean");
  if (pos == std::string::npos) {
    return 0.0;
  }
  const std::vector<double> values = extractDoubles(line.substr(pos));
  return values.empty() ? 0.0 : values.front();
}

}  // namespace

KalibrResult readKalibrResult(const std::string& result_txt_path) {
  std::ifstream input(result_txt_path);
  if (!input) {
    throw std::runtime_error("failed to open Kalibr result txt: " + result_txt_path);
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    lines.push_back(line);
  }

  KalibrResult result;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    const std::string& l = lines[i];
    if (l.find("Reprojection error (cam0):") != std::string::npos) {
      result.residuals.reprojection_normalized_mean = readMeanAfterToken(l);
    } else if (l.find("Gyroscope error (imu0):") != std::string::npos) {
      result.residuals.gyro_normalized_mean = readMeanAfterToken(l);
    } else if (l.find("Accelerometer error (imu0):") != std::string::npos) {
      result.residuals.accel_normalized_mean = readMeanAfterToken(l);
    } else if (l.find("Reprojection error (cam0) [px]") != std::string::npos) {
      result.residuals.reprojection_mean_px = readMeanAfterToken(l);
    } else if (l.find("Gyroscope error (imu0) [rad/s]") != std::string::npos) {
      result.residuals.gyro_mean_rad_s = readMeanAfterToken(l);
    } else if (l.find("Accelerometer error (imu0) [m/s^2]") != std::string::npos) {
      result.residuals.accel_mean_m_s2 = readMeanAfterToken(l);
    } else if (l.find("T_ci:") != std::string::npos && i + 4 < lines.size()) {
      result.T_ci = readMatrix4(lines, i + 1);
    } else if (l.find("T_ic:") != std::string::npos && i + 4 < lines.size()) {
      result.T_ic = readMatrix4(lines, i + 1);
    } else if (l.find("timeshift cam0 to imu0") != std::string::npos &&
               i + 1 < lines.size()) {
      const std::vector<double> values = extractDoubles(lines[i + 1]);
      if (!values.empty()) {
        result.timeshift_cam_to_imu_s = values.front();
      }
    } else if (l.find("Gravity vector") != std::string::npos &&
               i + 1 < lines.size()) {
      const std::vector<double> values = extractDoubles(lines[i + 1]);
      if (values.size() >= 3) {
        result.gravity = Vec3(values[0], values[1], values[2]);
      }
    }
  }

  return result;
}

}  // namespace ceres_cam_imu
