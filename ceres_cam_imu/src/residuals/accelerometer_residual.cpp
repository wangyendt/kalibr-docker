#include "ceres_cam_imu/residuals/accelerometer_residual.h"

#include <array>
#include <algorithm>
#include <cmath>
#include <utility>

#include <ceres/sized_cost_function.h>

#include "ceres_cam_imu/core/so3.h"
#include "ceres_cam_imu/core/so3_jacobians.h"

namespace ceres_cam_imu {
namespace {

void writeMatrixRowMajor(const Mat3& matrix, const int block_size,
                         double* jacobian, const int col_offset = 0) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      jacobian[row * block_size + col_offset + col] = matrix(row, col);
    }
  }
}

class AccelerometerCost final
    : public ceres::SizedCostFunction<3, 6, 3, 6, 6, 6, 6,
                                      6, 6, 3, 3, 3, 3, 3, 3> {
 public:
  AccelerometerCost(ImuSample sample, ImuNoise noise,
                    SplineSegmentMeta6 pose_segment,
                    SplineSegmentMeta6 bias_segment)
      : sample(std::move(sample)),
        noise(std::move(noise)),
        pose_segment(std::move(pose_segment)),
        bias_segment(std::move(bias_segment)) {
    inv_sigma = 1.0 / std::max(1e-12, this->noise.accelDiscreteSigma());
  }

  bool Evaluate(double const* const* parameters, double* residuals,
                double** jacobians) const override {
    const std::array<double, 6> pose_weights =
        pose_segment.weights(sample.timestamp_s, 0);
    const std::array<double, 6> pose_dot_weights =
        pose_segment.weights(sample.timestamp_s, 1);
    const std::array<double, 6> pose_ddot_weights =
        pose_segment.weights(sample.timestamp_s, 2);
    const std::array<double, 6> bias_weights =
        bias_segment.weights(sample.timestamp_s, 0);

    Vec6 curve = Vec6::Zero();
    Vec6 curve_dot = Vec6::Zero();
    Vec6 curve_ddot = Vec6::Zero();
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec6> control(parameters[2 + i]);
      curve += pose_weights[static_cast<std::size_t>(i)] * control;
      curve_dot += pose_dot_weights[static_cast<std::size_t>(i)] * control;
      curve_ddot += pose_ddot_weights[static_cast<std::size_t>(i)] * control;
    }

    Vec3 bias = Vec3::Zero();
    for (int i = 0; i < 6; ++i) {
      const Eigen::Map<const Vec3> control(parameters[8 + i]);
      bias += bias_weights[static_cast<std::size_t>(i)] * control;
    }

    const Vec3 r_w_b = curve.tail<3>();
    const Mat3 R_bw = rotationVectorToMatrix(r_w_b).transpose();
    const Vec3 a_w = curve_ddot.head<3>();
    const Eigen::Map<const Vec3> g_w(parameters[1]);
    const Vec3 q_w = a_w - g_w;
    const Vec3 h_b = R_bw * q_w;

    const Vec3 r_dot = curve_dot.tail<3>();
    const Vec3 r_ddot = curve_ddot.tail<3>();
    const Mat3 J_left = leftJacobianSO3(r_w_b);
    const Vec3 omega_b = -J_left * r_dot;
    const Vec3 alpha_b = -J_left * r_ddot;

    const Eigen::Map<const Vec3> r_b(parameters[0]);
    const Vec3 lever =
        alpha_b.cross(r_b) + omega_b.cross(omega_b.cross(r_b));
    const Vec3 body_specific_force = h_b + lever;

    const Eigen::Map<const Vec3> r_i_b(parameters[0] + 3);
    const Mat3 R_i_b = rotationVectorToMatrix(r_i_b);
    const Vec3 predicted = R_i_b * body_specific_force + bias;
    const Vec3 residual = inv_sigma * (predicted - sample.accel_m_s2);
    residuals[0] = residual.x();
    residuals[1] = residual.y();
    residuals[2] = residual.z();

    if (jacobians) {
      const Mat3 d_omega_d_r =
          -leftJacobianTimesVectorDerivative(r_w_b, r_dot);
      const Mat3 d_omega_d_r_dot = -J_left;
      const Mat3 d_alpha_d_r =
          -leftJacobianTimesVectorDerivative(r_w_b, r_ddot);
      const Mat3 d_alpha_d_r_ddot = -J_left;
      const Mat3 d_h_d_r =
          rotationTransposeTimesVectorDerivative(r_w_b, q_w);
      const Mat3 d_lever_d_alpha = -skew(r_b);
      const Mat3 d_lever_d_omega =
          -skew(omega_b.cross(r_b)) - skew(omega_b) * skew(r_b);
      const Mat3 d_body_d_r =
          d_h_d_r + d_lever_d_alpha * d_alpha_d_r
          + d_lever_d_omega * d_omega_d_r;
      const Mat3 d_body_d_r_dot =
          d_lever_d_omega * d_omega_d_r_dot;
      const Mat3 d_body_d_r_ddot =
          d_lever_d_alpha * d_alpha_d_r_ddot;

      if (jacobians[0]) {
        std::fill(jacobians[0], jacobians[0] + 18, 0.0);
        const Mat3 d_body_d_r_b =
            skew(alpha_b) + skew(omega_b) * skew(omega_b);
        const Mat3 d_residual_d_r_b = inv_sigma * R_i_b * d_body_d_r_b;
        const Mat3 d_residual_d_r_i_b =
            inv_sigma * R_i_b * skew(body_specific_force)
            * leftJacobianSO3(r_i_b);
        writeMatrixRowMajor(d_residual_d_r_b, 6, jacobians[0], 0);
        writeMatrixRowMajor(d_residual_d_r_i_b, 6, jacobians[0], 3);
      }

      if (jacobians[1]) {
        const Mat3 d_residual_d_gravity = -inv_sigma * R_i_b * R_bw;
        writeMatrixRowMajor(d_residual_d_gravity, 3, jacobians[1]);
      }

      for (int i = 0; i < 6; ++i) {
        double* J = jacobians[2 + i];
        if (!J) {
          continue;
        }
        std::fill(J, J + 18, 0.0);
        const Mat3 d_body_d_translation_control =
            pose_ddot_weights[static_cast<std::size_t>(i)] * R_bw;
        const Mat3 d_body_d_rotation_control =
            pose_weights[static_cast<std::size_t>(i)] * d_body_d_r
            + pose_dot_weights[static_cast<std::size_t>(i)] * d_body_d_r_dot
            + pose_ddot_weights[static_cast<std::size_t>(i)]
                  * d_body_d_r_ddot;
        writeMatrixRowMajor(inv_sigma * R_i_b * d_body_d_translation_control,
                            6, J, 0);
        writeMatrixRowMajor(inv_sigma * R_i_b * d_body_d_rotation_control,
                            6, J, 3);
      }

      for (int i = 0; i < 6; ++i) {
        double* J = jacobians[8 + i];
        if (!J) {
          continue;
        }
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

ceres::CostFunction* createAccelerometerResidual(
    const ImuSample& sample, const ImuNoise& noise,
    const SplineSegmentMeta6& pose_segment,
    const SplineSegmentMeta6& accel_bias_segment) {
  return new AccelerometerCost(sample, noise, pose_segment, accel_bias_segment);
}

}  // namespace ceres_cam_imu
