#include <iostream>
#include <string>

#include <algorithm>
#include <cmath>

#include "ceres_cam_imu/io/calibration_result_reader.h"
#include "ceres_cam_imu/io/kalibr_result_parser.h"

namespace {

std::string argValue(int argc, char** argv, const std::string& name) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == name) {
      return argv[i + 1];
    }
  }
  return "";
}

double rotationDeltaDeg(const ceres_cam_imu::Mat4& lhs,
                        const ceres_cam_imu::Mat4& rhs) {
  const ceres_cam_imu::Mat3 dR =
      lhs.block<3, 3>(0, 0) * rhs.block<3, 3>(0, 0).transpose();
  const double cos_angle =
      std::clamp((dR.trace() - 1.0) * 0.5, -1.0, 1.0);
  constexpr double kPi = 3.14159265358979323846;
  return std::acos(cos_angle) * 180.0 / kPi;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string result_path = argValue(argc, argv, "--kalibr-result");
  const std::string ceres_result_path = argValue(argc, argv, "--ceres-result");
  if (result_path.empty()) {
    std::cout << "compare_kalibr_result --kalibr-result result.txt "
                 "[--ceres-result result.yaml]\n";
    return 2;
  }
  const ceres_cam_imu::KalibrResult result =
      ceres_cam_imu::readKalibrResult(result_path);
  std::cout << "T_ci:\n" << result.T_ci << "\n";
  std::cout << "T_ic:\n" << result.T_ic << "\n";
  std::cout << "timeshift_cam_to_imu_s: " << result.timeshift_cam_to_imu_s << "\n";
  std::cout << "gravity: " << result.gravity.transpose() << "\n";
  std::cout << "mean residuals: reproj_px="
            << result.residuals.reprojection_mean_px
            << " gyro_rad_s=" << result.residuals.gyro_mean_rad_s
            << " accel_m_s2=" << result.residuals.accel_mean_m_s2 << "\n";
  std::cout << "mean normalized residuals: reproj="
            << result.residuals.reprojection_normalized_mean
            << " gyro=" << result.residuals.gyro_normalized_mean
            << " accel=" << result.residuals.accel_normalized_mean << "\n";
  if (!ceres_result_path.empty()) {
    const ceres_cam_imu::CalibrationResultFile ceres_result =
        ceres_cam_imu::readCalibrationResultYaml(ceres_result_path);
    const double rotation_deg =
        rotationDeltaDeg(ceres_result.T_c_b, result.T_ci);
    const double translation_m =
        (ceres_result.T_c_b.block<3, 1>(0, 3) -
         result.T_ci.block<3, 1>(0, 3))
            .norm();
    const double time_shift_delta_s =
        ceres_result.time_shift_s - result.timeshift_cam_to_imu_s;
    const double gravity_norm =
        (ceres_result.gravity - result.gravity).norm();
    std::cout << "ceres T_c_b:\n" << ceres_result.T_c_b << "\n";
    std::cout << "ceres time_shift_s: " << ceres_result.time_shift_s << "\n";
    std::cout << "ceres gravity: " << ceres_result.gravity.transpose()
              << "\n";
    std::cout << "ceres mean residuals: reproj_px="
              << ceres_result.residuals.reprojection_mean_px
              << " gyro_rad_s=" << ceres_result.residuals.gyro_mean_rad_s
              << " accel_m_s2="
              << ceres_result.residuals.accel_mean_m_s2 << "\n";
    std::cout << "ceres_vs_kalibr: rotation_deg=" << rotation_deg
              << " translation_m=" << translation_m
              << " time_shift_s=" << time_shift_delta_s
              << " gravity_norm=" << gravity_norm << "\n";
    std::cout << "residual_delta_ceres_minus_kalibr: reproj_px="
              << (ceres_result.residuals.reprojection_mean_px -
                  result.residuals.reprojection_mean_px)
              << " gyro_rad_s="
              << (ceres_result.residuals.gyro_mean_rad_s -
                  result.residuals.gyro_mean_rad_s)
              << " accel_m_s2="
              << (ceres_result.residuals.accel_mean_m_s2 -
                  result.residuals.accel_mean_m_s2)
              << "\n";
    if (ceres_result.has_kalibr_delta) {
      std::cout << "embedded_kalibr_delta: rotation_deg="
                << ceres_result.kalibr_delta.rotation_deg
                << " translation_m="
                << ceres_result.kalibr_delta.translation_m
                << " time_shift_s="
                << ceres_result.kalibr_delta.time_shift_s
                << " gravity_norm="
                << ceres_result.kalibr_delta.gravity_norm << "\n";
    }
  }
  return 0;
}
