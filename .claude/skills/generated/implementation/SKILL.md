---
name: implementation
description: "Skill for the Implementation area of kalibr-docker. 151 symbols across 34 files."
---

# Implementation

151 symbols | 34 files | Cohesion: 83%

## When to Use

- Working with code in `aslam_cv/`
- Understanding how rand, camera, J work
- Modifying implementation-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_cv/aslam_cameras/include/aslam/cameras/implementation/CameraGeometry.hpp` | vsEuclideanToKeypoint, vsHomogeneousToKeypoint, vsKeypointToEuclidean, vsKeypointToHomogeneous, euclideanToKeypoint (+19) |
| `aslam_cv/aslam_cameras/include/aslam/cameras/implementation/PinholeProjection.hpp` | euclideanToKeypointIntrinsicsJacobian, PinholeProjection, updateTemporaries, hypot, intersectCircles (+11) |
| `aslam_cv/aslam_cameras/include/aslam/cameras/implementation/OmniProjection.hpp` | createRandomKeypoint, createRandomVisiblePoint, euclideanToKeypointIntrinsicsJacobian, OmniProjection, updateTemporaries (+10) |
| `aslam_cv/aslam_cameras/include/aslam/cameras/implementation/DoubleSphereProjection.hpp` | createRandomKeypoint, createRandomVisiblePoint, euclideanToKeypointIntrinsicsJacobian, euclideanToKeypoint, euclideanToKeypoint (+7) |
| `aslam_cv/aslam_cameras/include/aslam/cameras/implementation/ExtendedUnifiedProjection.hpp` | createRandomKeypoint, createRandomVisiblePoint, euclideanToKeypointIntrinsicsJacobian, euclideanToKeypoint, euclideanToKeypoint (+7) |
| `aslam_optimizer/sparse_block_matrix/include/sparse_block_matrix/implementation/sparse_block_matrix.hpp` | block, transpose, transpose, atxpy, SparseBlockMatrix (+4) |
| `Schweizer-Messer/sm_kinematics/include/sm/kinematics/implementation/UncertainVector.hpp` | operator*, dot, dot, UncertainVector, operator+ (+3) |
| `aslam_optimizer/aslam_backend/include/aslam/backend/implementation/CompressedColumnMatrix.hpp` | clear, rows, cols, writeJacobians, fromDenseTolerance (+1) |
| `aslam_cv/aslam_time/src/time.cpp` | aslam_walltime, normalizeSecNSecUnsigned, aslam_nanosleep, sleepUntil |
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/QuaternionExpression.hpp` | getRealIndex, getIIndex, getJIndex, getKIndex |

## Entry Points

Start here when exploring this area:

- **`rand`** (Function) — `Schweizer-Messer/sm_random/src/random.cpp:72`
- **`camera`** (Function) — `aslam_cv/aslam_cameras/include/aslam/cameras/implementation/CameraGeometry.hpp:484`
- **`J`** (Function) — `aslam_nonparametric_estimation/bsplines/test/EuclideanBSplineTests.cpp:552`
- **`gettimeofday`** (Function) — `Schweizer-Messer/sm_logging/src/gettimeofday.hpp:30`
- **`aslam_walltime`** (Function) — `aslam_cv/aslam_time/src/time.cpp:81`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `GenericScalarExpressionNode` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/GenericScalarExpressionNode.hpp` | 16 |
| `GSEUnRes` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/GenericScalarExpression.hpp` | 27 |
| `GSEBinRes` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/GenericScalarExpression.hpp` | 41 |
| `SparseBlockMatrix` | Class | `aslam_optimizer/sparse_block_matrix/include/sparse_block_matrix/sparse_block_matrix.h` | 50 |
| `rand` | Function | `Schweizer-Messer/sm_random/src/random.cpp` | 72 |
| `camera` | Function | `aslam_cv/aslam_cameras/include/aslam/cameras/implementation/CameraGeometry.hpp` | 484 |
| `J` | Function | `aslam_nonparametric_estimation/bsplines/test/EuclideanBSplineTests.cpp` | 552 |
| `gettimeofday` | Function | `Schweizer-Messer/sm_logging/src/gettimeofday.hpp` | 30 |
| `aslam_walltime` | Function | `aslam_cv/aslam_time/src/time.cpp` | 81 |
| `normalizeSecNSecUnsigned` | Function | `aslam_cv/aslam_time/src/time.cpp` | 365 |
| `tic` | Function | `aslam_offline_calibration/ethz_apriltag2/src/example/apriltags_demo.cpp` | 85 |
| `p` | Function | `Schweizer-Messer/sm_timing/include/sm/timing/implementation/TimestampCorrector.hpp` | 28 |
| `isRealFirst` | Function | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/QuaternionExpression.hpp` | 37 |
| `getRealIndex` | Function | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/QuaternionExpression.hpp` | 17 |
| `getIIndex` | Function | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/QuaternionExpression.hpp` | 20 |
| `getJIndex` | Function | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/QuaternionExpression.hpp` | 23 |
| `getKIndex` | Function | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/implementation/QuaternionExpression.hpp` | 26 |
| `exportUV` | Function | `Schweizer-Messer/sm_python/src/exportUncertainVector.cpp` | 10 |
| `exportUS` | Function | `Schweizer-Messer/sm_python/src/exportUncertainVector.cpp` | 67 |
| `exportUncertainVector` | Function | `Schweizer-Messer/sm_python/src/exportUncertainVector.cpp` | 119 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `InitializeIntrinsics → IsUndistortedKeypointValid` | cross_community | 4 |
| `CreateRandomVisiblePoint → Instance` | intra_community | 4 |
| `CreateRandomVisiblePoint → Instance` | intra_community | 4 |
| `CreateRandomVisiblePoint → Instance` | intra_community | 4 |
| `AngularVelocityAndJacobian → IsAboveLine` | intra_community | 4 |
| `Undistort → J` | cross_community | 4 |
| `VsEuclideanToKeypoint → IsValid` | cross_community | 3 |
| `VsKeypointToEuclidean → IsValid` | intra_community | 3 |
| `VsKeypointToHomogeneous → IsValid` | intra_community | 3 |
| `InitializeIntrinsics → Hypot` | intra_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Test | 6 calls |
| Aslam | 1 calls |
| Boost | 1 calls |
| Cluster_56 | 1 calls |
| Io | 1 calls |

## How to Explore

1. `context({name: "rand"})` — see callers and callees
2. `query({query: "implementation"})` — find related execution flows
3. Read key files listed above for implementation details
