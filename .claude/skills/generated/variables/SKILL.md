---
name: variables
description: "Skill for the Variables area of kalibr-docker. 22 symbols across 7 files."
---

# Variables

22 symbols | 7 files | Cohesion: 80%

## When to Use

- Working with code in `kalibr-camimu-ceres/`
- Understanding how output, output, camera work
- Modifying variables-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp` | blockVec3, evalPoseBlock, evalBiasBlock, usesScaleMisalignment, usesSizeEffect (+2) |
| `kalibr-camimu-ceres/src/residuals/imu_model.cpp` | predictCalibratedGyroscope, predictScaleMisalignedGyroscope, predictCalibratedAccelerometer, predictScaleMisalignedAccelerometer, predictSizeEffectAccelerometer |
| `kalibr-camimu-ceres/include/ceres_cam_imu/variables/imu_intrinsics.h` | data, data, data |
| `kalibr-camimu-ceres/src/variables/imu_intrinsics.cpp` | lowerTriangularMatrix, matrix3Block, vector3Block |
| `kalibr-camimu-ceres/include/ceres_cam_imu/variables/extrinsics.h` | data, data |
| `kalibr-camimu-ceres/include/ceres_cam_imu/variables/gravity.h` | data |
| `kalibr-camimu-ceres/src/io/calibration_result_writer.cpp` | output |

## Entry Points

Start here when exploring this area:

- **`output`** (Function) — `kalibr-camimu-ceres/src/io/calibration_result_writer.cpp:122`
- **`output`** (Function) — `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp:141`
- **`camera`** (Function) — `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp:282`
- **`predictCalibratedGyroscope`** (Function) — `kalibr-camimu-ceres/src/residuals/imu_model.cpp:11`
- **`predictScaleMisalignedGyroscope`** (Function) — `kalibr-camimu-ceres/src/residuals/imu_model.cpp:16`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `output` | Function | `kalibr-camimu-ceres/src/io/calibration_result_writer.cpp` | 122 |
| `output` | Function | `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp` | 141 |
| `camera` | Function | `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp` | 282 |
| `predictCalibratedGyroscope` | Function | `kalibr-camimu-ceres/src/residuals/imu_model.cpp` | 11 |
| `predictScaleMisalignedGyroscope` | Function | `kalibr-camimu-ceres/src/residuals/imu_model.cpp` | 16 |
| `predictCalibratedAccelerometer` | Function | `kalibr-camimu-ceres/src/residuals/imu_model.cpp` | 25 |
| `predictScaleMisalignedAccelerometer` | Function | `kalibr-camimu-ceres/src/residuals/imu_model.cpp` | 30 |
| `predictSizeEffectAccelerometer` | Function | `kalibr-camimu-ceres/src/residuals/imu_model.cpp` | 36 |
| `lowerTriangularMatrix` | Function | `kalibr-camimu-ceres/src/variables/imu_intrinsics.cpp` | 8 |
| `matrix3Block` | Function | `kalibr-camimu-ceres/src/variables/imu_intrinsics.cpp` | 19 |
| `vector3Block` | Function | `kalibr-camimu-ceres/src/variables/imu_intrinsics.cpp` | 29 |
| `data` | Method | `kalibr-camimu-ceres/include/ceres_cam_imu/variables/extrinsics.h` | 11 |
| `data` | Method | `kalibr-camimu-ceres/include/ceres_cam_imu/variables/extrinsics.h` | 20 |
| `data` | Method | `kalibr-camimu-ceres/include/ceres_cam_imu/variables/gravity.h` | 10 |
| `data` | Method | `kalibr-camimu-ceres/include/ceres_cam_imu/variables/imu_intrinsics.h` | 14 |
| `data` | Method | `kalibr-camimu-ceres/include/ceres_cam_imu/variables/imu_intrinsics.h` | 23 |
| `data` | Method | `kalibr-camimu-ceres/include/ceres_cam_imu/variables/imu_intrinsics.h` | 31 |
| `blockVec3` | Function | `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp` | 22 |
| `evalPoseBlock` | Function | `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp` | 24 |
| `evalBiasBlock` | Function | `kalibr-camimu-ceres/src/optimizer/residual_statistics.cpp` | 35 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `Evaluate → CommonLeverAcceleration` | cross_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Residuals | 7 calls |
| Io | 3 calls |
| Optimizer | 2 calls |

## How to Explore

1. `context({name: "output"})` — see callers and callees
2. `query({query: "variables"})` — find related execution flows
3. Read key files listed above for implementation details
