---
name: kalibr-camera-calibration
description: "Skill for the Kalibr_camera_calibration area of kalibr-docker. 33 symbols across 8 files."
---

# Kalibr_camera_calibration

33 symbols | 8 files | Cohesion: 70%

## When to Use

- Working with code in `aslam_offline_calibration/`
- Understanding how logError, getReprojectionErrorStatistics, getReprojectionErrors work
- Modifying kalibr_camera_calibration-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | getReprojectionErrorStatistics, getReprojectionErrors, plotAllReprojectionErrors, printParameters, printDebugEnd (+7) |
| `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/ObsDb.py` | getAllViewTimestamps, getObservationAtTime, getAllObsTwoCams, getAllObsCam, numCameras (+1) |
| `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraCalibrator.py` | initGeometryFromObservations, fromTargetViewObservations, initializeBaselineDVs, addTargetView |
| `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py` | solveFullBatch, addPoseDesignVariable, stereoCalibrate, calibrateIntrinsics |
| `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/MulticamGraph.py` | getInitialGuesses, plotGraph, initializeGraphFromObsDb |
| `Schweizer-Messer/sm_python/python/sm/__init__.py` | logError, logDebug |
| `aslam_offline_calibration/kalibr/python/kalibr_imu_camera_calibration/IccCalibrator.py` | optimize |
| `aslam_optimizer/aslam_backend_python/python/aslam_backend/__init__.py` | TransformationDv |

## Entry Points

Start here when exploring this area:

- **`logError`** (Function) — `Schweizer-Messer/sm_python/python/sm/__init__.py:42`
- **`getReprojectionErrorStatistics`** (Function) — `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py:102`
- **`getReprojectionErrors`** (Function) — `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py:128`
- **`plotAllReprojectionErrors`** (Function) — `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py:226`
- **`printParameters`** (Function) — `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py:638`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `TransformationDv` | Class | `aslam_optimizer/aslam_backend_python/python/aslam_backend/__init__.py` | 9 |
| `logError` | Function | `Schweizer-Messer/sm_python/python/sm/__init__.py` | 42 |
| `getReprojectionErrorStatistics` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 102 |
| `getReprojectionErrors` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 128 |
| `plotAllReprojectionErrors` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 226 |
| `printParameters` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 638 |
| `printDebugEnd` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 686 |
| `getAllPointStatistics` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 164 |
| `plotPolarError` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 175 |
| `plotAzumithalError` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 201 |
| `generateReport` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 413 |
| `logDebug` | Function | `Schweizer-Messer/sm_python/python/sm/__init__.py` | 18 |
| `solveFullBatch` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py` | 277 |
| `addPoseDesignVariable` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py` | 5 |
| `stereoCalibrate` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py` | 14 |
| `calibrateIntrinsics` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraIntializers.py` | 187 |
| `normalize` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 32 |
| `getImageCenterRay` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 35 |
| `getPointStatistics` | Function | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraUtils.py` | 46 |
| `initGeometryFromObservations` | Method | `aslam_offline_calibration/kalibr/python/kalibr_camera_calibration/CameraCalibrator.py` | 54 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `GenerateReport → Normalize` | cross_community | 6 |
| `GenerateReport → RecoverCovariance` | cross_community | 3 |
| `GenerateReport → GetReprojectionErrors` | cross_community | 3 |
| `GenerateReport → GetReprojectionErrorStatistics` | cross_community | 3 |
| `AddTargetView → TransformationDv` | intra_community | 3 |
| `AddTargetView → LogDebug` | cross_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Kalibr_common | 1 calls |

## How to Explore

1. `context({name: "logError"})` — see callers and callees
2. `query({query: "kalibr_camera_calibration"})` — find related execution flows
3. Read key files listed above for implementation details
