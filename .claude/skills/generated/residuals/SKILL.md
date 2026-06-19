---
name: residuals
description: "Skill for the Residuals area of kalibr-docker. 35 symbols across 10 files."
---

# Residuals

35 symbols | 10 files | Cohesion: 77%

## When to Use

- Working with code in `kalibr-camimu-ceres/`
- Understanding how skew, rotationVectorToMatrix, leftJacobianSO3 work
- Modifying residuals-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | writeMatrixRowMajor, writeLowerTriangularProductJacobian, leverAccelerationPointJacobian, leverAccelerationOmegaJacobian, leverAccelerationAlphaJacobian (+7) |
| `kalibr-camimu-ceres/src/optimizer/calibration_problem.cpp` | dataPtr, overlapsLocalPoseMotionWindow, usesScaleMisalignment, usesSizeEffect, addImuIntrinsicParameterBlocks (+1) |
| `kalibr-camimu-ceres/src/residuals/gyroscope_residual.cpp` | writeMatrixRowMajor, Evaluate, Evaluate, createGyroscopeResidual, createScaleMisalignedGyroscopeResidual |
| `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3_jacobians.h` | leftJacobianSO3, leftJacobianTimesVectorDerivative, rotationTransposeTimesVectorDerivative |
| `kalibr-camimu-ceres/src/residuals/camera_reprojection_residual.cpp` | rightJacobianSO3Exp, evalPose, Evaluate |
| `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3.h` | skew, rotationVectorToMatrix |
| `kalibr-camimu-ceres/src/residuals/imu_model.cpp` | commonLeverAcceleration |
| `kalibr-camimu-ceres/include/ceres_cam_imu/residuals/pose_motion_prior.h` | createPoseMotionPrior |
| `kalibr-camimu-ceres/include/ceres_cam_imu/trajectory/spline_eval.h` | evalPoseCurve6 |
| `kalibr-camimu-ceres/src/initialization/time_shift_initializer.cpp` | angularVelocityAt |

## Entry Points

Start here when exploring this area:

- **`skew`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3.h:10`
- **`rotationVectorToMatrix`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3.h:22`
- **`leftJacobianSO3`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3_jacobians.h:9`
- **`leftJacobianTimesVectorDerivative`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3_jacobians.h:29`
- **`rotationTransposeTimesVectorDerivative`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3_jacobians.h:79`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `skew` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3.h` | 10 |
| `rotationVectorToMatrix` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3.h` | 22 |
| `leftJacobianSO3` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3_jacobians.h` | 9 |
| `leftJacobianTimesVectorDerivative` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3_jacobians.h` | 29 |
| `rotationTransposeTimesVectorDerivative` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3_jacobians.h` | 79 |
| `commonLeverAcceleration` | Function | `kalibr-camimu-ceres/src/residuals/imu_model.cpp` | 6 |
| `createPoseMotionPrior` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/residuals/pose_motion_prior.h` | 8 |
| `buildCalibrationProblem` | Function | `kalibr-camimu-ceres/src/optimizer/calibration_problem.cpp` | 401 |
| `createAccelerometerResidual` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 553 |
| `createScaleMisalignedAccelerometerResidual` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 560 |
| `createSizeEffectAccelerometerResidual` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 568 |
| `createGyroscopeResidual` | Function | `kalibr-camimu-ceres/src/residuals/gyroscope_residual.cpp` | 329 |
| `createScaleMisalignedGyroscopeResidual` | Function | `kalibr-camimu-ceres/src/residuals/gyroscope_residual.cpp` | 336 |
| `evalPoseCurve6` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/trajectory/spline_eval.h` | 12 |
| `writeMatrixRowMajor` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 17 |
| `writeLowerTriangularProductJacobian` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 26 |
| `leverAccelerationPointJacobian` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 38 |
| `leverAccelerationOmegaJacobian` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 43 |
| `leverAccelerationAlphaJacobian` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 47 |
| `evaluateAccelKinematics` | Function | `kalibr-camimu-ceres/src/residuals/accelerometer_residual.cpp` | 202 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `EstimateOrientationGravityAndGyroBiasPrior → Skew` | cross_community | 5 |
| `EstimateCameraImuTimeShiftPrior → Skew` | cross_community | 5 |
| `FitPoseSplineControlsFromCameraPoses → Skew` | cross_community | 5 |
| `EstimateOrientationGravityAndGyroBiasPrior → EvalPoseCurve6` | cross_community | 4 |
| `Evaluate → Skew` | intra_community | 4 |
| `Evaluate → Skew` | intra_community | 4 |
| `BuildCalibrationProblem → UsesScaleMisalignment` | cross_community | 4 |
| `BuildCalibrationProblem → DataPtr` | cross_community | 4 |
| `BuildCalibrationProblem → UsesSizeEffect` | cross_community | 4 |
| `BuildCalibrationProblem → TimeShiftPriorCost` | cross_community | 4 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Variables | 7 calls |
| Optimizer | 4 calls |
| Trajectory | 1 calls |

## How to Explore

1. `context({name: "skew"})` — see callers and callees
2. `query({query: "residuals"})` — find related execution flows
3. Read key files listed above for implementation details
