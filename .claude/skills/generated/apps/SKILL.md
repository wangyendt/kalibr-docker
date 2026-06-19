---
name: apps
description: "Skill for the Apps area of kalibr-docker. 20 symbols across 6 files."
---

# Apps

20 symbols | 6 files | Cohesion: 64%

## When to Use

- Working with code in `kalibr-camimu-ceres/`
- Understanding how main, matrixToPose6, pose6ToMatrix work
- Modifying apps-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | argValue, intArg, doubleArg, printFinalState, printLocalTimeDiagnostics (+9) |
| `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/calibration_problem.h` | matrixToPose6, pose6ToMatrix |
| `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/residual_statistics.h` | writeImuDiagnosticsCsv |
| `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | applyStagePoseMotionVariances |
| `kalibr-camimu-ceres/include/ceres_cam_imu/variables/imu_intrinsics.h` | parseImuCalibrationModel |
| `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3.h` | rotationMatrixToVector |

## Entry Points

Start here when exploring this area:

- **`main`** (Function) — `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp:757`
- **`matrixToPose6`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/calibration_problem.h:194`
- **`pose6ToMatrix`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/calibration_problem.h:195`
- **`writeImuDiagnosticsCsv`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/residual_statistics.h:59`
- **`applyStagePoseMotionVariances`** (Function) — `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h:52`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `main` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 757 |
| `matrixToPose6` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/calibration_problem.h` | 194 |
| `pose6ToMatrix` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/calibration_problem.h` | 195 |
| `writeImuDiagnosticsCsv` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/residual_statistics.h` | 59 |
| `applyStagePoseMotionVariances` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/optimizer/staged_optimizer.h` | 52 |
| `parseImuCalibrationModel` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/variables/imu_intrinsics.h` | 50 |
| `rotationMatrixToVector` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/core/so3.h` | 72 |
| `argValue` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 33 |
| `intArg` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 86 |
| `doubleArg` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 91 |
| `printFinalState` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 521 |
| `printLocalTimeDiagnostics` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 682 |
| `setLowerTriangularBlock` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 369 |
| `setMatrix3Block` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 383 |
| `setVector3Block` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 395 |
| `initializeImuIntrinsicsFromKalibr` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 405 |
| `initializeImuIntrinsicsFromResult` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 463 |
| `printResidualStatistics` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 560 |
| `printFinalResidualStatistics` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 597 |
| `printFinalResidualStatistics` | Function | `kalibr-camimu-ceres/apps/calibrate_cam_imu.cpp` | 611 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Optimizer | 12 calls |
| Io | 9 calls |

## How to Explore

1. `context({name: "main"})` — see callers and callees
2. `query({query: "apps"})` — find related execution flows
3. Read key files listed above for implementation details
