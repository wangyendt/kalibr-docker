#include "ceres_cam_imu/io/config_reader.h"

#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace ceres_cam_imu {
namespace {

std::vector<std::string> readLines(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("failed to open yaml file: " + path);
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }
    lines.push_back(line);
  }
  return lines;
}

std::vector<double> extractNumbers(const std::string& text) {
  static const std::regex number_re(
      R"([-+]?(?:(?:\d+\.\d*)|(?:\.\d+)|(?:\d+))(?:[eE][-+]?\d+)?)");
  std::vector<double> values;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), number_re);
       it != std::sregex_iterator(); ++it) {
    values.push_back(std::stod(it->str()));
  }
  return values;
}

std::string valueTextForKey(const std::vector<std::string>& lines,
                            const std::string& key) {
  const std::regex key_re("(^|[[:space:]])" + key + "[[:space:]]*:");
  for (const std::string& line : lines) {
    std::smatch match;
    if (!std::regex_search(line, match, key_re)) {
      continue;
    }
    const std::size_t start = static_cast<std::size_t>(match.position());
    const std::size_t colon = line.find(':', start);
    if (colon != std::string::npos) {
      return line.substr(colon + 1);
    }
  }
  throw std::runtime_error("missing yaml key: " + key);
}

std::vector<double> numericSequence(const std::vector<std::string>& lines,
                                    const std::string& key,
                                    const std::size_t expected) {
  const std::vector<double> values = extractNumbers(valueTextForKey(lines, key));
  if (values.size() < expected) {
    throw std::runtime_error("yaml key has too few numeric values: " + key);
  }
  return values;
}

double numericScalar(const std::vector<std::string>& lines,
                     const std::string& key) {
  const std::vector<double> values = extractNumbers(valueTextForKey(lines, key));
  if (values.empty()) {
    throw std::runtime_error("yaml key is not numeric: " + key);
  }
  return values.front();
}

bool tryNumericScalar(const std::vector<std::string>& lines,
                      const std::string& key, double* value) {
  try {
    *value = numericScalar(lines, key);
    return true;
  } catch (const std::runtime_error&) {
    return false;
  }
}

bool tryMatrix4AfterKey(const std::vector<std::string>& lines,
                        const std::string& key, Mat4* matrix) {
  const std::regex key_re("(^|[[:space:]])" + key + "[[:space:]]*:");
  for (std::size_t line_index = 0; line_index < lines.size(); ++line_index) {
    std::smatch match;
    if (!std::regex_search(lines[line_index], match, key_re)) {
      continue;
    }
    if (line_index + 4 >= lines.size()) {
      throw std::runtime_error("yaml matrix has too few rows: " + key);
    }
    Mat4 parsed = Mat4::Identity();
    for (int row = 0; row < 4; ++row) {
      const std::vector<double> values =
          extractNumbers(lines.at(line_index + 1 + static_cast<std::size_t>(row)));
      if (values.size() < 4) {
        throw std::runtime_error("failed to parse yaml matrix row: " + key);
      }
      for (int col = 0; col < 4; ++col) {
        parsed(row, col) = values[static_cast<std::size_t>(col)];
      }
    }
    *matrix = parsed;
    return true;
  }
  return false;
}

std::string stringScalar(const std::vector<std::string>& lines,
                         const std::string& key,
                         const std::string& default_value) {
  try {
    const std::string value = valueTextForKey(lines, key);
    const std::size_t first = value.find_first_not_of(" \t'\"");
    const std::size_t last = value.find_last_not_of(" \t'\"");
    if (first == std::string::npos || last == std::string::npos || last < first) {
      return default_value;
    }
    return value.substr(first, last - first + 1);
  } catch (const std::runtime_error&) {
    return default_value;
  }
}

}  // namespace

CameraIntrinsics readCameraIntrinsics(const std::string& yaml_path) {
  const std::vector<std::string> lines = readLines(yaml_path);

  CameraIntrinsics intrinsics;
  intrinsics.camera_model = stringScalar(lines, "camera_model", "pinhole");
  intrinsics.distortion_model = stringScalar(lines, "distortion_model", "radtan");

  const std::vector<double> intr = numericSequence(lines, "intrinsics", 4);
  intrinsics.fx = intr[0];
  intrinsics.fy = intr[1];
  intrinsics.cx = intr[2];
  intrinsics.cy = intr[3];

  const std::vector<double> dist = numericSequence(lines, "distortion_coeffs", 4);
  intrinsics.k1 = dist[0];
  intrinsics.k2 = dist[1];
  intrinsics.p1 = dist[2];
  intrinsics.p2 = dist[3];

  const std::vector<double> resolution = numericSequence(lines, "resolution", 2);
  intrinsics.width = static_cast<int>(resolution[0]);
  intrinsics.height = static_cast<int>(resolution[1]);
  return intrinsics;
}

ImuNoise readImuNoise(const std::string& yaml_path) {
  const std::vector<std::string> lines = readLines(yaml_path);
  ImuNoise noise;
  noise.update_rate_hz = numericScalar(lines, "update_rate");
  noise.accelerometer_noise_density = numericScalar(lines, "accelerometer_noise_density");
  noise.accelerometer_random_walk = numericScalar(lines, "accelerometer_random_walk");
  noise.gyroscope_noise_density = numericScalar(lines, "gyroscope_noise_density");
  noise.gyroscope_random_walk = numericScalar(lines, "gyroscope_random_walk");
  return noise;
}

AprilGridConfig readAprilGridConfig(const std::string& yaml_path) {
  const std::vector<std::string> lines = readLines(yaml_path);
  AprilGridConfig config;
  config.tag_cols = static_cast<int>(numericScalar(lines, "tagCols"));
  config.tag_rows = static_cast<int>(numericScalar(lines, "tagRows"));
  config.tag_size_m = numericScalar(lines, "tagSize");
  config.tag_spacing_ratio = numericScalar(lines, "tagSpacing");
  return config;
}

CamchainImuPrior readCamchainImuPrior(const std::string& yaml_path) {
  const std::vector<std::string> lines = readLines(yaml_path);
  CamchainImuPrior prior;
  prior.has_T_cam_imu =
      tryMatrix4AfterKey(lines, "T_cam_imu", &prior.T_cam_imu);
  prior.has_timeshift_cam_imu =
      tryNumericScalar(lines, "timeshift_cam_imu",
                       &prior.timeshift_cam_imu_s);
  return prior;
}

}  // namespace ceres_cam_imu
