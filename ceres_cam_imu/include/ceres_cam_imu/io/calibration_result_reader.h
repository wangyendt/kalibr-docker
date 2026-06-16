#pragma once

#include <string>

#include "ceres_cam_imu/core/types.h"

namespace ceres_cam_imu {

struct CalibrationResultDelta {
  double rotation_deg = 0.0;
  double translation_m = 0.0;
  double time_shift_s = 0.0;
  double gravity_norm = 0.0;
};

struct CalibrationResultFile {
  int format_version = 0;
  Mat4 T_c_b = Mat4::Identity();
  Mat4 T_b_c = Mat4::Identity();
  double time_shift_s = 0.0;
  Vec3 gravity = Vec3::Zero();
  KalibrResidualStats residuals;
  bool has_kalibr_delta = false;
  CalibrationResultDelta kalibr_delta;
};

CalibrationResultFile readCalibrationResultYaml(
    const std::string& result_yaml_path);

}  // namespace ceres_cam_imu
