#include "ceres_cam_imu/residuals/gyroscope_residual.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <utility>

#include <ceres/sized_cost_function.h>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/so3_jacobians.h"

namespace ceres_cam_imu {
namespace {

void writeMatrixRowMajor(const Eigen::MatrixXd& matrix, const int block_size,
                         double* jacobian) {
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      jacobian[row * block_size + col] = matrix(row, col);
    }
  }
}

class GyroscopeCost final
    : public ceres::SizedCostFunction<3, 6, 6, 6, 6, 6, 6, 6,
                                      3, 3, 3, 3, 3, 3> {
 public:
  GyroscopeCost(ImuSample sample, ImuNoise noise, SplineSegmentMeta6 pose_segment,
                SplineSegmentMeta6 bias_segment)
      : sample(std::move(sample)),
        noise(std::move(noise)),
        pose_segment(std::move(pose_segment)),
        bias_segment(std::move(bias_segment)) {
    inv_sigma = 1.0 / std::max(1e-12, this->noise.gyroDiscreteSigma());
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const std::array<double, 6> pose_weights =
        pose_segment.weights(sample.timestamp_s, 0);
    const std::array<double, 6> pose_dot_weights =
        pose_segment.weights(sample.timestamp_s, 1);
    const std::array<double, 6> bias_weights =
        bias_segment.weights(sample.timestamp_s, 0);

    Vec6 curve = Vec6::Zero();
    Vec6 curve_dot = Vec6::Zero();
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec6> control(parameters[1 + i]);
      curve += pose_weights[static_cast<std::size_t>(i)] * control;
      curve_dot += pose_dot_weights[static_cast<std::size_t>(i)] * control;
    }

    Vec3 bias = Vec3::Zero();
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec3> control(parameters[7 + i]);
      bias += bias_weights[static_cast<std::size_t>(i)] * control;
    }

    const Vec3 r = curve.tail<3>();
    const Vec3 r_dot = curve_dot.tail<3>();
    const Mat3 J_left = leftJacobianSO3(r);
    const Vec3 omega_b = -J_left * r_dot;

    const Eigen::Map<const Vec3> r_i_b(parameters[0] + 3);
    const Mat3 R_i_b = rotationVectorToMatrix(r_i_b);
    const Vec3 predicted = R_i_b * omega_b + bias;
    const Vec3 residual = inv_sigma * (predicted - sample.gyro_rad_s);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (jacobians) {
      const Mat3 d_omega_d_r =
          -leftJacobianTimesVectorDerivative(r, r_dot);
      const Mat3 d_omega_d_r_dot = -J_left;

      if (jacobians[0]) {
        std::fill(jacobians[0], jacobians[0] + 18, 0.0);
        const Mat3 d_pred_d_r_i_b =
            inv_sigma * R_i_b * skew(omega_b) * leftJacobianSO3(r_i_b);
        for (int row = 0; row < 3; ++row) {
          for (int col = 0; col < 3; ++col) {
            jacobians[0][row * 6 + 3 + col] = d_pred_d_r_i_b(row, col);
          }
        }
      }

      for (int i = 0; i < 6; ++i) {
        double* J = jacobians[1 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 18, 0.0);
        const Mat3 d_omega_d_control_rotation =
            d_omega_d_r * pose_weights[static_cast<std::size_t>(i)]
            + d_omega_d_r_dot * pose_dot_weights[static_cast<std::size_t>(i)];
        const Mat3 d_residual_d_control_rotation =
            inv_sigma * R_i_b * d_omega_d_control_rotation;
        for (int row = 0; row < 3; ++row) {
          for (int col = 0; col < 3; ++col) {
            J[row * 6 + 3 + col] = d_residual_d_control_rotation(row, col);
          }
        }
      }

      for (int i = 0; i < 6; ++i) {
        double* J = jacobians[7 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 9, 0.0);
        const Mat3 d_residual_d_bias =
            inv_sigma * bias_weights[static_cast<std::size_t>(i)]
            * Mat3::Identity();
        writeMatrixRowMajor(d_residual_d_bias, 3, J);
      }
    }
    return true;
  }

 private:
  ImuSample sample;
  ImuNoise noise;
  SplineSegmentMeta6 pose_segment;
  SplineSegmentMeta6 bias_segment;
  double inv_sigma = 1.0;
};

}  // namespace

ceres::CostFunction* createGyroscopeResidual(
    const ImuSample& sample, const ImuNoise& noise,
    const SplineSegmentMeta6& pose_segment,
    const SplineSegmentMeta6& gyro_bias_segment) {
  return new GyroscopeCost(sample, noise, pose_segment, gyro_bias_segment);
}

}  // namespace ceres_cam_imu
