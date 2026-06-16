#include "ceres_cam_imu/optimizer/residual_statistics.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>

#include <Eigen/Core>

#include "ceres_cam_imu/camera/camera_model.h"
#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/so3_jacobians.h"
#include "ceres_cam_imu/residuals/imu_model.h"
#include "ceres_cam_imu/trajectory/spline_eval.h"
#include "ceres_cam_imu/variables/imu_intrinsics.h"

namespace ceres_cam_imu {
namespace {

Vec3 blockVec3(const double *data) { return Eigen::Map<const Vec3>(data); }

Vec6 evalPoseBlock(const SplineSegmentMeta6 &segment, const double timestamp_s,
                   const std::vector<PoseControlBlock> &controls,
                   const int derivative_order) {
  std::array<const double *, 6> active{};
  for (int i = 0; i < 6; ++i) {
    active[static_cast<std::size_t>(i)] =
        controls.at(static_cast<std::size_t>(segment.coeff_start + i)).data();
  }
  return evalPoseCurve6(segment, timestamp_s, active, derivative_order);
}

Vec3 evalBiasBlock(const SplineSegmentMeta6 &segment, const double timestamp_s,
                   const std::vector<BiasControlBlock> &controls) {
  std::array<const double *, 6> active{};
  for (int i = 0; i < 6; ++i) {
    active[static_cast<std::size_t>(i)] =
        controls.at(static_cast<std::size_t>(segment.coeff_start + i)).data();
  }
  return evalBiasCurve6(segment, timestamp_s, active, 0);
}

ResidualMagnitudeStats computeStats(std::vector<double> *values) {
  ResidualMagnitudeStats stats;
  if (!values || values->empty()) {
    return stats;
  }

  std::sort(values->begin(), values->end());
  stats.count = static_cast<int>(values->size());
  stats.max = values->back();
  const double sum = std::accumulate(values->begin(), values->end(), 0.0);
  stats.mean = sum / static_cast<double>(values->size());
  const std::size_t mid = values->size() / 2;
  if (values->size() % 2 == 0) {
    stats.median = 0.5 * (values->at(mid - 1) + values->at(mid));
  } else {
    stats.median = values->at(mid);
  }

  double square_sum = 0.0;
  double variance_sum = 0.0;
  for (const double value : *values) {
    square_sum += value * value;
    const double centered = value - stats.mean;
    variance_sum += centered * centered;
  }
  const double inv_count = 1.0 / static_cast<double>(values->size());
  stats.rms = std::sqrt(square_sum * inv_count);
  stats.stddev = std::sqrt(variance_sum * inv_count);
  return stats;
}

std::size_t countCameraMeasurements(const std::vector<ImageObservation> &images,
                                    const CalibrationOptions &options) {
  std::size_t count = 0;
  int frame_count = 0;
  for (const ImageObservation &image : images) {
    if (options.max_frames > 0 && frame_count >= options.max_frames) {
      break;
    }
    ++frame_count;
    count += image.corners.size();
  }
  return count;
}

std::size_t countSelectedImuSamples(const std::vector<ImuSample> &imu_samples,
                                    const CalibrationOptions &options) {
  const int stride = std::max(1, options.imu_stride);
  std::size_t count = 0;
  for (std::size_t i = 0; i < imu_samples.size();
       i += static_cast<std::size_t>(stride)) {
    if (options.max_imu_residuals > 0 &&
        count >= static_cast<std::size_t>(options.max_imu_residuals)) {
      break;
    }
    ++count;
  }
  return count;
}

void insertTopAccelOutlier(std::vector<ImuResidualOutlier> *outliers,
                           const int max_count,
                           const ImuResidualOutlier &candidate) {
  if (!outliers || max_count <= 0) {
    return;
  }
  if (outliers->size() < static_cast<std::size_t>(max_count)) {
    outliers->push_back(candidate);
    return;
  }
  auto weakest = std::min_element(
      outliers->begin(), outliers->end(),
      [](const ImuResidualOutlier &lhs, const ImuResidualOutlier &rhs) {
        return lhs.accel_error_m_s2 < rhs.accel_error_m_s2;
      });
  if (weakest != outliers->end() &&
      candidate.accel_error_m_s2 > weakest->accel_error_m_s2) {
    *weakest = candidate;
  }
}

bool usesScaleMisalignment(const ImuCalibrationModel model) {
  return model == ImuCalibrationModel::kScaleMisalignment ||
         model == ImuCalibrationModel::kScaleMisalignmentSizeEffect;
}

bool usesSizeEffect(const ImuCalibrationModel model) {
  return model == ImuCalibrationModel::kScaleMisalignmentSizeEffect;
}

} // namespace

CalibrationResidualStatistics evaluateCalibrationResidualStatistics(
    const CameraIntrinsics &intrinsics, const ImuNoise &imu_noise,
    const std::vector<ImageObservation> &images,
    const std::vector<ImuSample> &imu_samples,
    const CalibrationOptions &options, const CalibrationState &state) {
  CalibrationResidualStatistics result;
  const CameraModel camera(intrinsics);
  std::vector<double> reprojection_px;
  reprojection_px.reserve(countCameraMeasurements(images, options));
  std::vector<double> reprojection_normalized;
  reprojection_normalized.reserve(reprojection_px.capacity());

  const Vec3 t_c_b = blockVec3(state.T_c_b.data());
  const Vec3 r_c_b = blockVec3(state.T_c_b.data() + 3);
  const Mat3 R_c_b = rotationVectorToMatrix(r_c_b);
  const double reprojection_scale =
      1.0 / (std::max(1e-12, options.reprojection_sigma_px) * std::sqrt(2.0));

  int frame_count = 0;
  for (const ImageObservation &image : images) {
    if (options.max_frames > 0 && frame_count >= options.max_frames) {
      break;
    }
    ++frame_count;
    const double query_time =
        image.timestamp_s + state.camera_time_shift_s.value;
    if (!state.pose_spline.isValidTime(query_time)) {
      ++result.skipped_camera_frames;
      continue;
    }

    const SplineSegmentMeta6 pose_meta =
        state.pose_spline.segmentMeta6(query_time);
    const Vec6 pose =
        evalPoseBlock(pose_meta, query_time, state.pose_controls, 0);
    const Vec3 t_w_b = pose.head<3>();
    const Vec3 r_w_b = pose.tail<3>();
    const Mat3 R_b_w = rotationVectorToMatrix(r_w_b).transpose();

    for (const CornerMeasurement &corner : image.corners) {
      const Vec3 p_b = R_b_w * (corner.target_point - t_w_b);
      const Vec3 p_c = R_c_b * p_b + t_c_b;
      Vec2 pixel;
      if (!camera.projectWithJacobian(p_c, &pixel, nullptr)) {
        ++result.skipped_camera_projections;
        continue;
      }
      const double error_px = (corner.pixel - pixel).norm();
      reprojection_px.push_back(error_px);
      reprojection_normalized.push_back(reprojection_scale * error_px);
    }
  }

  std::vector<double> gyro_rad_s;
  gyro_rad_s.reserve(countSelectedImuSamples(imu_samples, options));
  std::vector<double> gyro_normalized;
  gyro_normalized.reserve(gyro_rad_s.capacity());
  std::vector<double> accel_m_s2;
  accel_m_s2.reserve(gyro_rad_s.capacity());
  std::vector<double> accel_normalized;
  accel_normalized.reserve(gyro_rad_s.capacity());

  const Vec3 r_b = blockVec3(state.imu_extrinsic.data());
  const Vec3 r_i_b = blockVec3(state.imu_extrinsic.data() + 3);
  const Mat3 R_i_b = rotationVectorToMatrix(r_i_b);
  const Vec3 gravity = blockVec3(state.gravity.data());
  const double gyro_scale =
      1.0 / std::max(1e-12, imu_noise.gyroDiscreteSigma());
  const double accel_scale =
      1.0 / std::max(1e-12, imu_noise.accelDiscreteSigma());

  int added_imu = 0;
  const int stride = std::max(1, options.imu_stride);
  for (std::size_t i = 0; i < imu_samples.size();
       i += static_cast<std::size_t>(stride)) {
    if (options.max_imu_residuals > 0 &&
        added_imu >= options.max_imu_residuals) {
      break;
    }
    const ImuSample &sample = imu_samples[i];
    if (!state.pose_spline.isValidTime(sample.timestamp_s) ||
        !state.gyro_bias_spline.isValidTime(sample.timestamp_s) ||
        !state.accel_bias_spline.isValidTime(sample.timestamp_s)) {
      ++result.skipped_imu_samples;
      continue;
    }

    const SplineSegmentMeta6 pose_meta =
        state.pose_spline.segmentMeta6(sample.timestamp_s);
    const Vec6 curve =
        evalPoseBlock(pose_meta, sample.timestamp_s, state.pose_controls, 0);
    const Vec6 curve_dot =
        evalPoseBlock(pose_meta, sample.timestamp_s, state.pose_controls, 1);
    const Vec6 curve_ddot =
        evalPoseBlock(pose_meta, sample.timestamp_s, state.pose_controls, 2);

    const Vec3 r_w_b = curve.tail<3>();
    const Mat3 R_bw = rotationVectorToMatrix(r_w_b).transpose();
    const Mat3 J_left = leftJacobianSO3(r_w_b);
    const Vec3 omega_b = -J_left * curve_dot.tail<3>();
    const Vec3 alpha_b = -J_left * curve_ddot.tail<3>();
    const Vec3 h_b = R_bw * (curve_ddot.head<3>() - gravity);
    const Vec3 angular_accel_lever = alpha_b.cross(r_b);
    const Vec3 centripetal_lever = omega_b.cross(omega_b.cross(r_b));
    const Vec3 lever = angular_accel_lever + centripetal_lever;

    const SplineSegmentMeta6 gyro_bias_meta =
        state.gyro_bias_spline.segmentMeta6(sample.timestamp_s);
    const Vec3 gyro_bias = evalBiasBlock(gyro_bias_meta, sample.timestamp_s,
                                         state.gyro_bias_controls);
    Vec3 gyro_predicted;
    if (usesScaleMisalignment(options.imu_model)) {
      const Mat3 R_gyro_i = rotationVectorToMatrix(
          vector3Block(state.imu_intrinsics.gyro_sensing_rotation.data()));
      gyro_predicted = predictScaleMisalignedGyroscope(
          R_i_b, R_gyro_i,
          lowerTriangularMatrix(state.imu_intrinsics.gyro_M.data()),
          matrix3Block(state.imu_intrinsics.gyro_accel_sensitivity.data()),
          omega_b, h_b + lever, gyro_bias);
    } else {
      gyro_predicted = predictCalibratedGyroscope(R_i_b, omega_b, gyro_bias);
    }
    const double gyro_error = (gyro_predicted - sample.gyro_rad_s).norm();
    gyro_rad_s.push_back(gyro_error);
    gyro_normalized.push_back(gyro_scale * gyro_error);

    const SplineSegmentMeta6 accel_bias_meta =
        state.accel_bias_spline.segmentMeta6(sample.timestamp_s);
    const Vec3 accel_bias = evalBiasBlock(accel_bias_meta, sample.timestamp_s,
                                          state.accel_bias_controls);
    Vec3 accel_predicted;
    if (usesSizeEffect(options.imu_model)) {
      accel_predicted = predictSizeEffectAccelerometer(
          R_i_b, lowerTriangularMatrix(state.imu_intrinsics.accel_M.data()),
          h_b, r_b, vector3Block(state.imu_intrinsics.accel_axis_rx_i.data()),
          vector3Block(state.imu_intrinsics.accel_axis_ry_i.data()),
          vector3Block(state.imu_intrinsics.accel_axis_rz_i.data()), omega_b,
          alpha_b, accel_bias);
    } else if (usesScaleMisalignment(options.imu_model)) {
      accel_predicted = predictScaleMisalignedAccelerometer(
          R_i_b, lowerTriangularMatrix(state.imu_intrinsics.accel_M.data()),
          h_b, lever, accel_bias);
    } else {
      accel_predicted =
          predictCalibratedAccelerometer(R_i_b, h_b, lever, accel_bias);
    }
    const double accel_error = (accel_predicted - sample.accel_m_s2).norm();
    accel_m_s2.push_back(accel_error);
    accel_normalized.push_back(accel_scale * accel_error);

    ImuResidualOutlier outlier;
    outlier.sample_index = static_cast<int>(i);
    outlier.timestamp_s = sample.timestamp_s;
    outlier.accel_error_m_s2 = accel_error;
    outlier.accel_normalized = accel_scale * accel_error;
    outlier.gyro_error_rad_s = gyro_error;
    outlier.gyro_normalized = gyro_scale * gyro_error;
    outlier.measured_accel_norm = sample.accel_m_s2.norm();
    outlier.predicted_accel_norm = accel_predicted.norm();
    outlier.pose_accel_world_norm = curve_ddot.head<3>().norm();
    outlier.gravity_corrected_body_accel_norm = h_b.norm();
    outlier.angular_accel_lever_norm = angular_accel_lever.norm();
    outlier.centripetal_lever_norm = centripetal_lever.norm();
    outlier.omega_body_norm = omega_b.norm();
    outlier.alpha_body_norm = alpha_b.norm();
    insertTopAccelOutlier(&result.top_accel_outliers, options.top_residuals,
                          outlier);
    ++added_imu;
  }

  result.reprojection_px = computeStats(&reprojection_px);
  result.reprojection_normalized = computeStats(&reprojection_normalized);
  result.gyro_rad_s = computeStats(&gyro_rad_s);
  result.gyro_normalized = computeStats(&gyro_normalized);
  result.accel_m_s2 = computeStats(&accel_m_s2);
  result.accel_normalized = computeStats(&accel_normalized);
  std::sort(result.top_accel_outliers.begin(), result.top_accel_outliers.end(),
            [](const ImuResidualOutlier &lhs, const ImuResidualOutlier &rhs) {
              return lhs.accel_error_m_s2 > rhs.accel_error_m_s2;
            });
  return result;
}

CalibrationResidualStatistics evaluateCalibrationResidualStatistics(
    const std::vector<CameraObservationDataset> &cameras,
    const ImuNoise &imu_noise,
    const std::vector<ImuSample> &imu_samples,
    const CalibrationOptions &options, const CalibrationState &state) {
  if (cameras.empty()) {
    return evaluateCalibrationResidualStatistics(
        CameraIntrinsics(), imu_noise, std::vector<ImageObservation>(),
        imu_samples, options, state);
  }

  CalibrationResidualStatistics result =
      evaluateCalibrationResidualStatistics(cameras.front().intrinsics,
                                            imu_noise, cameras.front().images,
                                            imu_samples, options, state);

  std::vector<double> reprojection_px;
  std::vector<double> reprojection_normalized;
  std::size_t reserve_count = 0;
  for (const CameraObservationDataset &camera : cameras) {
    reserve_count += countCameraMeasurements(camera.images, options);
  }
  reprojection_px.reserve(reserve_count);
  reprojection_normalized.reserve(reserve_count);
  result.skipped_camera_frames = 0;
  result.skipped_camera_projections = 0;

  const double reprojection_scale =
      1.0 / (std::max(1e-12, options.reprojection_sigma_px) * std::sqrt(2.0));
  for (std::size_t camera_index = 0; camera_index < cameras.size();
       ++camera_index) {
    const CameraObservationDataset &camera_dataset = cameras[camera_index];
    const CameraModel camera(camera_dataset.intrinsics);
    const CameraExtrinsicBlock *extrinsic = &state.T_c_b;
    if (camera_index < state.camera_extrinsics.size()) {
      extrinsic = &state.camera_extrinsics[camera_index];
    }
    const double time_shift =
        camera_index < state.camera_time_shifts.size()
            ? state.camera_time_shifts[camera_index].value
            : state.camera_time_shift_s.value;
    const Vec3 t_c_b = blockVec3(extrinsic->data());
    const Vec3 r_c_b = blockVec3(extrinsic->data() + 3);
    const Mat3 R_c_b = rotationVectorToMatrix(r_c_b);

    int frame_count = 0;
    for (const ImageObservation &image : camera_dataset.images) {
      if (options.max_frames > 0 && frame_count >= options.max_frames) {
        break;
      }
      ++frame_count;
      const double query_time = image.timestamp_s + time_shift;
      if (!state.pose_spline.isValidTime(query_time)) {
        ++result.skipped_camera_frames;
        continue;
      }

      const SplineSegmentMeta6 pose_meta =
          state.pose_spline.segmentMeta6(query_time);
      const Vec6 pose =
          evalPoseBlock(pose_meta, query_time, state.pose_controls, 0);
      const Vec3 t_w_b = pose.head<3>();
      const Vec3 r_w_b = pose.tail<3>();
      const Mat3 R_b_w = rotationVectorToMatrix(r_w_b).transpose();

      for (const CornerMeasurement &corner : image.corners) {
        const Vec3 p_b = R_b_w * (corner.target_point - t_w_b);
        const Vec3 p_c = R_c_b * p_b + t_c_b;
        Vec2 pixel;
        if (!camera.projectWithJacobian(p_c, &pixel, nullptr)) {
          ++result.skipped_camera_projections;
          continue;
        }
        const double error_px = (corner.pixel - pixel).norm();
        reprojection_px.push_back(error_px);
        reprojection_normalized.push_back(reprojection_scale * error_px);
      }
    }
  }

  result.reprojection_px = computeStats(&reprojection_px);
  result.reprojection_normalized = computeStats(&reprojection_normalized);
  return result;
}

} // namespace ceres_cam_imu
