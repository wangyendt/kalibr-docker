---
name: io
description: "Skill for the Io area of kalibr-docker. 53 symbols across 18 files."
---

# Io

53 symbols | 18 files | Cohesion: 74%

## When to Use

- Working with code in `kalibr-camimu-ceres/`
- Understanding how operator<<, toStream, paddedToStream work
- Modifying io-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `kalibr-camimu-ceres/src/io/config_reader.cpp` | readLines, extractNumbers, numericScalar, tryNumericScalar, tryMatrix4AfterKey (+12) |
| `kalibr-camimu-ceres/src/io/calibration_result_reader.cpp` | trimAscii, valueAfterColon, parseDoubles, parseScalarAfterColon, parseMatrix4AfterColon (+3) |
| `kalibr-camimu-ceres/src/io/kalibr_result_parser.cpp` | extractDoubles, readMatrix4, readMatrix3, tryReadVector3, readMeanAfterToken (+1) |
| `kalibr-camimu-ceres/src/io/calibration_result_writer.cpp` | writeMatrix4, writeMatrix3, writeStdArray, writeControlBlocks |
| `kalibr-camimu-ceres/include/ceres_cam_imu/io/config_reader.h` | readCameraIntrinsics, readImuNoise, readAprilGridConfig |
| `Schweizer-Messer/sm_common/include/sm/string_routines.hpp` | toStream, paddedToStream |
| `kalibr-camimu-ceres/include/ceres_cam_imu/processing/dataset_processing.h` | trimImuSamplesKalibr, countCornerMeasurements |
| `Schweizer-Messer/sm_common/include/sm/hash_id.hpp` | operator<< |
| `Schweizer-Messer/sm_timing/src/Timer.cpp` | print |
| `aslam_optimizer/aslam_backend/include/aslam/backend/implementation/CompressedColumnMatrix.hpp` | value |

## Entry Points

Start here when exploring this area:

- **`operator<<`** (Function) — `Schweizer-Messer/sm_common/include/sm/hash_id.hpp:141`
- **`toStream`** (Function) — `Schweizer-Messer/sm_common/include/sm/string_routines.hpp:228`
- **`paddedToStream`** (Function) — `Schweizer-Messer/sm_common/include/sm/string_routines.hpp:261`
- **`MEMBER`** (Function) — `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/GenericScalar.hpp:17`
- **`main`** (Function) — `kalibr-camimu-ceres/apps/check_dataset.cpp:43`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `operator<<` | Function | `Schweizer-Messer/sm_common/include/sm/hash_id.hpp` | 141 |
| `toStream` | Function | `Schweizer-Messer/sm_common/include/sm/string_routines.hpp` | 228 |
| `paddedToStream` | Function | `Schweizer-Messer/sm_common/include/sm/string_routines.hpp` | 261 |
| `MEMBER` | Function | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/GenericScalar.hpp` | 17 |
| `main` | Function | `kalibr-camimu-ceres/apps/check_dataset.cpp` | 43 |
| `readCameraIntrinsics` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/config_reader.h` | 15 |
| `readImuNoise` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/config_reader.h` | 18 |
| `readAprilGridConfig` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/config_reader.h` | 19 |
| `readCornerCsv` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/corner_csv_reader.h` | 9 |
| `readImuCsv` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/imu_csv_reader.h` | 9 |
| `readPoseCsv` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/pose_csv_reader.h` | 9 |
| `trimImuSamplesKalibr` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/processing/dataset_processing.h` | 24 |
| `countCornerMeasurements` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/processing/dataset_processing.h` | 31 |
| `input` | Function | `kalibr-camimu-ceres/src/io/calibration_result_reader.cpp` | 138 |
| `readImuNoise` | Function | `kalibr-camimu-ceres/src/io/config_reader.cpp` | 292 |
| `readAprilGridConfig` | Function | `kalibr-camimu-ceres/src/io/config_reader.cpp` | 306 |
| `readCamchainImuPrior` | Function | `kalibr-camimu-ceres/src/io/config_reader.cpp` | 316 |
| `input` | Function | `kalibr-camimu-ceres/src/io/kalibr_result_parser.cpp` | 96 |
| `main` | Function | `kalibr-camimu-ceres/apps/compare_kalibr_result.cpp` | 51 |
| `readCalibrationResultYaml` | Function | `kalibr-camimu-ceres/include/ceres_cam_imu/io/calibration_result_reader.h` | 43 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `Input → TrimAscii` | intra_community | 4 |
| `Input → ParseDoubles` | intra_community | 3 |
| `Input → ExtractDoubles` | intra_community | 3 |
| `ReadCameraIntrinsics → IsCameraHeaderLine` | cross_community | 3 |
| `ReadCameraIntrinsics → Lowercase` | cross_community | 3 |
| `ReadCameraIntrinsics → ValueTextForKey` | cross_community | 3 |
| `ReadCamchainImuPrior → IsCameraHeaderLine` | cross_community | 3 |
| `ReadCamchainImuPrior → ExtractNumbers` | intra_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Camera | 2 calls |

## How to Explore

1. `context({name: "operator<<"})` — see callers and callees
2. `query({query: "io"})` — find related execution flows
3. Read key files listed above for implementation details
