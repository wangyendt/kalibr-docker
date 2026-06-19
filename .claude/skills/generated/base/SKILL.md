---
name: base
description: "Skill for the Base area of kalibr-docker. 16 symbols across 6 files."
---

# Base

16 symbols | 6 files | Cohesion: 86%

## When to Use

- Working with code in `aslam_incremental_calibration/`
- Understanding how lock, computeObservation, interrupt work
- Modifying base-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp` | interrupt, exit, lock, safeSetPriority, safeWait (+1) |
| `aslam_incremental_calibration/incremental_calibration/src/base/Timer.cpp` | getLeft, stop, wait, sleep |
| `aslam_offline_calibration/ethz_apriltag2/src/example/apriltags_demo.cpp` | setTagCodes, parseOptions |
| `aslam_incremental_calibration/incremental_calibration/src/base/Mutex.cpp` | lock, safeLock |
| `aslam_cv/aslam_cameras_april/src/GridCalibrationTargetAprilgrid.cpp` | computeObservation |
| `aslam_incremental_calibration/incremental_calibration/src/base/Condition.cpp` | safeWait |

## Entry Points

Start here when exploring this area:

- **`lock`** (Function) — `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp:88`
- **`computeObservation`** (Method) — `aslam_cv/aslam_cameras_april/src/GridCalibrationTargetAprilgrid.cpp:142`
- **`interrupt`** (Method) — `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp:265`
- **`exit`** (Method) — `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp:278`
- **`setTagCodes`** (Method) — `aslam_offline_calibration/ethz_apriltag2/src/example/apriltags_demo.cpp:172`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `lock` | Function | `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp` | 88 |
| `computeObservation` | Method | `aslam_cv/aslam_cameras_april/src/GridCalibrationTargetAprilgrid.cpp` | 142 |
| `interrupt` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp` | 265 |
| `exit` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp` | 278 |
| `setTagCodes` | Method | `aslam_offline_calibration/ethz_apriltag2/src/example/apriltags_demo.cpp` | 172 |
| `parseOptions` | Method | `aslam_offline_calibration/ethz_apriltag2/src/example/apriltags_demo.cpp` | 190 |
| `safeSetPriority` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp` | 121 |
| `safeWait` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp` | 292 |
| `safeIsBusy` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Thread.cpp` | 324 |
| `getLeft` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Timer.cpp` | 91 |
| `stop` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Timer.cpp` | 105 |
| `wait` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Timer.cpp` | 115 |
| `sleep` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Timer.cpp` | 119 |
| `safeWait` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Condition.cpp` | 70 |
| `lock` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Mutex.cpp` | 72 |
| `safeLock` | Method | `aslam_incremental_calibration/incremental_calibration/src/base/Mutex.cpp` | 143 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `Main → Interrupt` | cross_community | 5 |
| `Lock → Round` | intra_community | 3 |
| `Lock → InvalidOperationException` | cross_community | 3 |
| `Lock → SafeIsBusy` | intra_community | 3 |
| `ComputeObservation → SizeStrings` | cross_community | 3 |
| `ComputeObservation → SizeMatrices` | cross_community | 3 |
| `ComputeObservation → Interrupt` | intra_community | 3 |
| `Stop → Timestamp` | intra_community | 3 |
| `Wait → Timestamp` | intra_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Algorithms | 2 calls |
| Cluster_59 | 1 calls |

## How to Explore

1. `context({name: "lock"})` — see callers and callees
2. `query({query: "base"})` — find related execution flows
3. Read key files listed above for implementation details
