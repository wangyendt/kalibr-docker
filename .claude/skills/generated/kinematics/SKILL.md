---
name: kinematics
description: "Skill for the Kinematics area of kalibr-docker. 44 symbols across 21 files."
---

# Kinematics

44 symbols | 21 files | Cohesion: 66%

## When to Use

- Working with code in `Schweizer-Messer/`
- Understanding how boxMinus, f, quatOPlus work
- Modifying kinematics-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | quatOPlus, quatInv, quat2r, updateQuat, axisAngle2quat (+4) |
| `aslam_optimizer/aslam_backend_expressions/src/HomogeneousExpressionNode.cpp` | evaluateJacobians, evaluateJacobiansImplementation, evaluateJacobiansImplementation |
| `aslam_optimizer/aslam_backend_expressions/src/MappedRotationQuaternion.cpp` | minimalDifferenceImplementation, minimalDifferenceAndJacobianImplementation, dpv |
| `aslam_optimizer/aslam_backend_expressions/src/RotationQuaternion.cpp` | dpv, minimalDifferenceImplementation, minimalDifferenceAndJacobianImplementation |
| `aslam_optimizer/aslam_backend_expressions/src/TransformationExpressionNode.cpp` | evaluateJacobians, evaluateJacobiansImplementation, evaluateJacobiansImplementation |
| `Schweizer-Messer/sm_kinematics/include/sm/kinematics/transformations.hpp` | boxMinus, boxTimes |
| `aslam_nonparametric_estimation/bsplines/test/BSplinePoseTests.cpp` | f, TEST |
| `Schweizer-Messer/sm_kinematics/include/sm/kinematics/rotations.hpp` | crossMx, R2AxisAngle |
| `Schweizer-Messer/sm_kinematics/src/EulerRodriguez.cpp` | parametersToRotationMatrix, parametersToSMatrix |
| `Schweizer-Messer/sm_kinematics/src/RotationVector.cpp` | parametersToSMatrix, angularVelocityAndJacobian |

## Entry Points

Start here when exploring this area:

- **`boxMinus`** (Function) — `Schweizer-Messer/sm_kinematics/include/sm/kinematics/transformations.hpp:78`
- **`f`** (Function) — `aslam_nonparametric_estimation/bsplines/test/BSplinePoseTests.cpp:120`
- **`quatOPlus`** (Function) — `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp:30`
- **`quatInv`** (Function) — `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp:32`
- **`crossMx`** (Function) — `Schweizer-Messer/sm_kinematics/include/sm/kinematics/rotations.hpp:57`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `EulerAnglesYawPitchRoll` | Class | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/EulerAnglesYawPitchRoll.hpp` | 29 |
| `EulerAnglesZYX` | Class | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/EulerAnglesZYX.hpp` | 7 |
| `RotationalKinematics` | Class | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/RotationalKinematics.hpp` | 9 |
| `EulerRodriguez` | Class | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/EulerRodriguez.hpp` | 7 |
| `RotationVector` | Class | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/RotationVector.hpp` | 7 |
| `boxMinus` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/transformations.hpp` | 78 |
| `f` | Function | `aslam_nonparametric_estimation/bsplines/test/BSplinePoseTests.cpp` | 120 |
| `quatOPlus` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | 30 |
| `quatInv` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | 32 |
| `crossMx` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/rotations.hpp` | 57 |
| `quat2r` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | 17 |
| `updateQuat` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | 41 |
| `dpv` | Function | `aslam_optimizer/aslam_backend_expressions/src/MappedRotationQuaternion.cpp` | 26 |
| `dpv` | Function | `aslam_optimizer/aslam_backend_expressions/src/RotationQuaternion.cpp` | 32 |
| `boxTimes` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/transformations.hpp` | 172 |
| `TEST` | Function | `aslam_nonparametric_estimation/bsplines/test/BSplinePoseTests.cpp` | 97 |
| `axisAngle2quat` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | 20 |
| `qexp` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | 46 |
| `qplus` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | 29 |
| `quatJacobian` | Function | `Schweizer-Messer/sm_kinematics/include/sm/kinematics/quaternion_algebra.hpp` | 40 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `Marginalize → CrossMx` | cross_community | 4 |
| `InverseTransformationAndJacobian → CrossMx` | cross_community | 4 |
| `ParametersToRotationMatrix → CrossMx` | intra_community | 4 |
| `AngularVelocityAndJacobian → CrossMx` | intra_community | 4 |
| `QuatLogJacobian2 → CrossMx` | cross_community | 4 |
| `MinimalDifferenceAndJacobianImplementation → Qplus` | intra_community | 3 |
| `MinimalDifferenceAndJacobianImplementation → QuatInv` | cross_community | 3 |
| `Qslerp → AxisAngle2quat` | cross_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Implementation | 1 calls |

## How to Explore

1. `context({name: "boxMinus"})` — see callers and callees
2. `query({query: "kinematics"})` — find related execution flows
3. Read key files listed above for implementation details
