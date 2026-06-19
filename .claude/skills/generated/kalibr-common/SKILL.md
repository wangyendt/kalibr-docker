---
name: kalibr-common
description: "Skill for the Kalibr_common area of kalibr-docker. 49 symbols across 6 files."
---

# Kalibr_common

49 symbols | 6 files | Cohesion: 76%

## When to Use

- Working with code in `aslam_offline_calibration/`
- Understanding how logWarn, catch_keyerror, CameraParameters work
- Modifying kalibr_common-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | raiseError, checkRosTopic, checkRosTopic, getCameraParameters, getExtrinsicsLastCamToHere (+33) |
| `aslam_offline_calibration/kalibr/python/kalibr_common/ImageDatasetReader.py` | next, __init__, truncateIndicesFromTime, truncateIndicesFromFreq |
| `aslam_offline_calibration/kalibr/python/kalibr_common/ImuDatasetReader.py` | next, __init__, truncateIndicesFromTime |
| `aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccSensors.py` | printDetails, __init__ |
| `Schweizer-Messer/sm_python/python/sm/__init__.py` | logWarn |
| `aslam_offline_calibration/kalibr/python/kalibr_rs_camera_calibration/RsCalibrator.py` | __generateExtrinsicsInitialGuess |

## Entry Points

Start here when exploring this area:

- **`logWarn`** (Function) — `Schweizer-Messer/sm_python/python/sm/__init__.py:26`
- **`catch_keyerror`** (Function) — `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py:182`
- **`CameraParameters`** (Class) — `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py:237`
- **`CameraChainParameters`** (Class) — `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py:655`
- **`ParametersBase`** (Class) — `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py:192`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `CameraParameters` | Class | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 237 |
| `CameraChainParameters` | Class | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 655 |
| `ParametersBase` | Class | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 192 |
| `ImuParameters` | Class | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 427 |
| `CalibrationTargetParameters` | Class | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 530 |
| `logWarn` | Function | `Schweizer-Messer/sm_python/python/sm/__init__.py` | 26 |
| `catch_keyerror` | Function | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 182 |
| `raiseError` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 232 |
| `checkRosTopic` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 245 |
| `checkRosTopic` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 435 |
| `getCameraParameters` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 670 |
| `getExtrinsicsLastCamToHere` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 687 |
| `getExtrinsicsImuToCam` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 708 |
| `checkTimeshiftCamImu` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 719 |
| `checkCamOverlaps` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 735 |
| `numCameras` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 752 |
| `printDetails` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 755 |
| `checkUpdateRate` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 449 |
| `getUpdateRate` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 454 |
| `checkAccelerometerStatistics` | Method | `aslam_offline_calibration/kalibr/python/kalibr_common/ConfigReader.py` | 463 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `PrintDetails → RaiseError` | cross_community | 4 |
| `Calibrate → LogWarn` | cross_community | 3 |

## How to Explore

1. `context({name: "logWarn"})` — see callers and callees
2. `query({query: "kalibr_common"})` — find related execution flows
3. Read key files listed above for implementation details
