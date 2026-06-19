---
name: optimizer
description: "Skill for the Optimizer area of kalibr-docker. 41 symbols across 15 files."
---

# Optimizer

41 symbols | 15 files | Cohesion: 71%

## When to Use

- Working with code in `kalibr-camimu-ceres/`
- Understanding how estimateOrientationGravityAndGyroBiasPrior, estimateCameraImuTimeShiftPrior, writeCalibrationResultYaml work
- Modifying optimizer-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `kalibr-camimu-ceres/src/optimizer/staged_optimizer.cpp` | hasFiniteStageCosts, decideCalibrationStageStateUpdate, solveCalibrationStage, stageOptions, stageIteration (+4) |
| `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | calibrationStageStateDecisionName, decideCalibrationStageStateUpdate, makeConservativeCalibrationStages, makeConservativeCalibrationStages, makeCalibrationStagesFromFreeMasks (+1) |
| `kalibr-camimu-ceres/src/optimizer/calibration_problem.cpp` | makeLoss, fillProblemSizeSummary, buildCalibrationProblem, cameraExtrinsicToMatrix, operator() (+1) |
| `kalibr-camimu-ceres/include/ceres_cam_imu/residuals/imu_model.h` | commonLeverAcceleration, predictScaleMisalignedGyroscope, predictCalibratedAccelerometer, predictScaleMisalignedAccelerometer, predictSizeEffectAccelerometer |
| `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp` | computeStats, countCameraMeasurements, evaluateCalibrationResidualStatistics |
| `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/state_snapshot.h` | isCompatibleStateSnapshot, restoreCalibrationState |
| `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/calibration_problem.h` | buildCalibrationProblem, solveCalibrationProblem |
| `kalibr-camimu-ceres/include/ceres_cam_imu/initialization/orientation_gravity_initializer.h` | estimateOrientationGravityAndGyroBiasPrior |
| `kalibr-camimu-ceres/include/ceres_cam_imu/initialization/time_shift_initializer.h` | estimateCameraImuTimeShiftPrior |
| `kalibr-camimu-ceres/include/ceres_cam_imu/io/calibration_result_writer.h` | writeCalibrationResultYaml |

## Entry Points

Start here when exploring this area:

- **`estimateOrientationGravityAndGyroBiasPrior`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/initialization/orientation_gravity_initializer.h:36`
- **`estimateCameraImuTimeShiftPrior`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/initialization/time_shift_initializer.h:28`
- **`writeCalibrationResultYaml`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/io/calibration_result_writer.h:16`
- **`readCamchainImuPrior`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/io/config_reader.h:20`
- **`calibrationStageStateDecisionName`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h:34`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `estimateOrientationGravityAndGyroBiasPrior` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/initialization/orientation_gravity_initializer.h` | 36 |
| `estimateCameraImuTimeShiftPrior` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/initialization/time_shift_initializer.h` | 28 |
| `writeCalibrationResultYaml` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/calibration_result_writer.h` | 16 |
| `readCamchainImuPrior` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/config_reader.h` | 20 |
| `calibrationStageStateDecisionName` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | 34 |
| `decideCalibrationStageStateUpdate` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | 37 |
| `makeConservativeCalibrationStages` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | 40 |
| `makeConservativeCalibrationStages` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | 43 |
| `makeCalibrationStagesFromFreeMasks` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | 47 |
| `applyStagePoseMotionOrders` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | 57 |
| `isCompatibleStateSnapshot` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/state_snapshot.h` | 22 |
| `restoreCalibrationState` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/state_snapshot.h` | 25 |
| `commonLeverAcceleration` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/residuals/imu_model.h` | 12 |
| `predictScaleMisalignedGyroscope` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/residuals/imu_model.h` | 18 |
| `predictCalibratedAccelerometer` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/residuals/imu_model.h` | 24 |
| `predictScaleMisalignedAccelerometer` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/residuals/imu_model.h` | 27 |
| `predictSizeEffectAccelerometer` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/residuals/imu_model.h` | 31 |
| `camera` | Function | `kalibr-camimu-ceres/tests/test_math.cpp` | 48 |
| `buildCalibrationProblem` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/calibration_problem.h` | 162 |
| `solveCalibrationProblem` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/calibration_problem.h` | 181 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `BuildCalibrationProblem → UsesScaleMisalignment` | cross_community | 4 |
| `BuildCalibrationProblem → DataPtr` | cross_community | 4 |
| `BuildCalibrationProblem → UsesSizeEffect` | cross_community | 4 |
| `BuildCalibrationProblem → TimeShiftPriorCost` | cross_community | 4 |
| `BuildCalibrationProblem → MarkActiveSegment` | cross_community | 3 |
| `BuildCalibrationProblem → CameraReprojectionCost` | intra_community | 3 |
| `EvaluateCalibrationResidualStatistics → Skew` | cross_community | 3 |
| `SolveCalibrationStage → IsSolverFailure` | intra_community | 3 |
| `SolveCalibrationStage → HasFiniteStageCosts` | intra_community | 3 |
| `SolveCalibrationStage → HasStageCostIncrease` | intra_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Apps | 8 calls |
| Io | 7 calls |
| Residuals | 4 calls |
| Initialization | 2 calls |
| Variables | 2 calls |

## How to Explore

1. `context({name: "estimateOrientationGravityAndGyroBiasPrior"})` — see callers and callees
2. `query({query: "optimizer"})` — find related execution flows
3. Read key files listed above for implementation details
