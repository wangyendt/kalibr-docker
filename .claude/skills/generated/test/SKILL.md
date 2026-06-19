---
name: test
description: "Skill for the Test area of kalibr-docker. 176 symbols across 73 files."
---

# Test

176 symbols | 73 files | Cohesion: 83%

## When to Use

- Working with code in `Schweizer-Messer/`
- Understanding how crossMx, boxMinus, inverseTransform work
- Modifying test-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `Schweizer-Messer/sm_kinematics/src/quaternion_algebra.cpp` | isLessThenEpsilons4thRoot, r2quat, quatOPlus, axisAngle2quat, arcSinXOverX (+11) |
| `Schweizer-Messer/sm_kinematics/src/transformations.cpp` | boxMinus, inverseTransform, fromTEuler, toTEuler, transformationAndJacobian (+2) |
| `aslam_optimizer/aslam_backend/test/SampleDvAndError.hpp` | Point2d, LinearErr, LinearErr2, LinearErr3, buildSystem (+2) |
| `Schweizer-Messer/sm_common/test/serialization_macros.cpp` | SimpleEntry, setRandom, ComplexEntry, setRandom, isBinaryEqual (+2) |
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/test/ExpressionTests.hpp` | getPrintResult, getTolerance, getJacobian, testJacobian, getExp (+2) |
| `aslam_cv/aslam_cameras/include/aslam/cameras/test/CameraGeometryTestHarness.hpp` | testKeypointToEuclidean, testKeypointToHomogeneous, testEuclideanToKeypoint, testIntrinsicsJacobian, testAll (+1) |
| `aslam_optimizer/aslam_backend_expressions/src/EuclideanExpressionNode.cpp` | toEuclidean, evaluateJacobians, evaluateJacobians, toEuclideanImplementation, evaluateJacobiansImplementation (+1) |
| `aslam_nonparametric_estimation/bsplines/test/DiffManifoldBSplineTests.hpp` | operator<<, copyKnots, disturbePointInto, updateControlVertex, updateSpline (+1) |
| `Schweizer-Messer/sm_kinematics/src/Transformation.cpp` | operator*, Transformation, inverse, operator*, slerpTransformations |
| `Schweizer-Messer/sm_kinematics/test/RotationalKinematicsTests.cpp` | testParametersToRotationMatrix, testSMatrix, testAngularVelocity, testAll, TEST |

## Entry Points

Start here when exploring this area:

- **`crossMx`** (Function) — `Schweizer-Messer/sm_kinematics/src/rotations.cpp:77`
- **`boxMinus`** (Function) — `Schweizer-Messer/sm_kinematics/src/transformations.cpp:44`
- **`inverseTransform`** (Function) — `Schweizer-Messer/sm_kinematics/src/transformations.cpp:54`
- **`fromTEuler`** (Function) — `Schweizer-Messer/sm_kinematics/src/transformations.cpp:67`
- **`toTEuler`** (Function) — `Schweizer-Messer/sm_kinematics/src/transformations.cpp:75`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `ErrorTermFs` | Class | `aslam_optimizer/aslam_backend/include/aslam/backend/ErrorTerm.hpp` | 182 |
| `OptimizationProblem` | Class | `aslam_optimizer/aslam_backend/include/aslam/backend/OptimizationProblem.hpp` | 20 |
| `Point2d` | Class | `aslam_optimizer/aslam_backend/test/SampleDvAndError.hpp` | 8 |
| `LinearErr` | Class | `aslam_optimizer/aslam_backend/test/SampleDvAndError.hpp` | 49 |
| `LinearErr2` | Class | `aslam_optimizer/aslam_backend/test/SampleDvAndError.hpp` | 84 |
| `LinearErr3` | Class | `aslam_optimizer/aslam_backend/test/SampleDvAndError.hpp` | 124 |
| `MEstimator` | Class | `aslam_optimizer/aslam_backend/include/aslam/backend/MEstimatorPolicies.hpp` | 8 |
| `GemanMcClureMEstimator` | Class | `aslam_optimizer/aslam_backend/include/aslam/backend/MEstimatorPolicies.hpp` | 22 |
| `SimpleEntry` | Class | `Schweizer-Messer/sm_common/test/serialization_macros.cpp` | 99 |
| `ComplexEntry` | Class | `Schweizer-Messer/sm_common/test/serialization_macros.cpp` | 125 |
| `CameraGeometryDesignVariableContainer` | Class | `aslam_cv/aslam_cv_backend/include/aslam/CameraGeometryDesignVariableContainer.hpp` | 16 |
| `crossMx` | Function | `Schweizer-Messer/sm_kinematics/src/rotations.cpp` | 77 |
| `boxMinus` | Function | `Schweizer-Messer/sm_kinematics/src/transformations.cpp` | 44 |
| `inverseTransform` | Function | `Schweizer-Messer/sm_kinematics/src/transformations.cpp` | 54 |
| `fromTEuler` | Function | `Schweizer-Messer/sm_kinematics/src/transformations.cpp` | 67 |
| `toTEuler` | Function | `Schweizer-Messer/sm_kinematics/src/transformations.cpp` | 75 |
| `transformationAndJacobian` | Function | `Schweizer-Messer/sm_kinematics/src/transformations.cpp` | 102 |
| `inverseTransformationAndJacobian` | Function | `Schweizer-Messer/sm_kinematics/src/transformations.cpp` | 113 |
| `boxTimes` | Function | `Schweizer-Messer/sm_kinematics/src/transformations.cpp` | 131 |
| `randomTransformation` | Function | `Schweizer-Messer/sm_kinematics/test/transformations.cpp` | 68 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `AngularVelocityBodyFrameAndJacobian → Fabs` | cross_community | 6 |
| `AngularAccelerationBodyFrameAndJacobian → Fabs` | cross_community | 6 |
| `Knots → Fabs` | cross_community | 5 |
| `InverseTransformationAndJacobian → Fabs` | cross_community | 5 |
| `KnotsVector → Fabs` | cross_community | 4 |
| `UOplus → CrossMx` | intra_community | 4 |
| `QuatLogJacobian2 → CrossMx` | cross_community | 4 |
| `QuatLogJacobian2 → IsLessThenEpsilons4thRoot` | intra_community | 4 |
| `QuatLogJacobian2 → Fabs` | intra_community | 4 |
| `FimOrig → Fabs` | cross_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Kinematics | 8 calls |
| Implementation | 4 calls |
| Boost | 2 calls |
| Rph2 | 2 calls |
| Cluster_41 | 1 calls |

## How to Explore

1. `context({name: "crossMx"})` — see callers and callees
2. `query({query: "test"})` — find related execution flows
3. Read key files listed above for implementation details
