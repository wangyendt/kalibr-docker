---
name: sm
description: "Skill for the Sm area of kalibr-docker. 16 symbols across 8 files."
---

# Sm

16 symbols | 8 files | Cohesion: 85%

## When to Use

- Working with code in `Schweizer-Messer/`
- Understanding how TEST, randLU, cameraDv work
- Modifying sm-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `Schweizer-Messer/sm_common/include/sm/hash_id.hpp` | random, hexString, randomize, isValid |
| `Schweizer-Messer/sm_matrix_archive/include/sm/MatrixArchive.hpp` | validateName, setMatrix, setVector |
| `Schweizer-Messer/sm_kinematics/src/Transformation.cpp` | setRandom, setRandom |
| `Schweizer-Messer/sm_random/include/sm/random.hpp` | randLU, randLUi |
| `Schweizer-Messer/sm_property_tree/include/sm/BoostPropertyTreeImplementation.hpp` | get, set |
| `Schweizer-Messer/sm_common/test/hash_id.cpp` | TEST |
| `aslam_cv/aslam_cv_error_terms/test/TestReprojectionError.cpp` | cameraDv |
| `aslam_cv/aslam_cameras/include/aslam/KeypointIdentifier.hpp` | setRandom |

## Entry Points

Start here when exploring this area:

- **`TEST`** (Function) — `Schweizer-Messer/sm_common/test/hash_id.cpp:9`
- **`randLU`** (Function) — `Schweizer-Messer/sm_random/include/sm/random.hpp:21`
- **`cameraDv`** (Function) — `aslam_cv/aslam_cv_error_terms/test/TestReprojectionError.cpp:88`
- **`randLUi`** (Function) — `Schweizer-Messer/sm_random/include/sm/random.hpp:24`
- **`hexString`** (Method) — `Schweizer-Messer/sm_common/include/sm/hash_id.hpp:43`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `TEST` | Function | `Schweizer-Messer/sm_common/test/hash_id.cpp` | 9 |
| `randLU` | Function | `Schweizer-Messer/sm_random/include/sm/random.hpp` | 21 |
| `cameraDv` | Function | `aslam_cv/aslam_cv_error_terms/test/TestReprojectionError.cpp` | 88 |
| `randLUi` | Function | `Schweizer-Messer/sm_random/include/sm/random.hpp` | 24 |
| `hexString` | Method | `Schweizer-Messer/sm_common/include/sm/hash_id.hpp` | 43 |
| `randomize` | Method | `Schweizer-Messer/sm_common/include/sm/hash_id.hpp` | 74 |
| `isValid` | Method | `Schweizer-Messer/sm_common/include/sm/hash_id.hpp` | 109 |
| `setRandom` | Method | `Schweizer-Messer/sm_kinematics/src/Transformation.cpp` | 124 |
| `setRandom` | Method | `Schweizer-Messer/sm_kinematics/src/Transformation.cpp` | 157 |
| `validateName` | Method | `Schweizer-Messer/sm_matrix_archive/include/sm/MatrixArchive.hpp` | 109 |
| `setMatrix` | Method | `Schweizer-Messer/sm_matrix_archive/include/sm/MatrixArchive.hpp` | 121 |
| `setVector` | Method | `Schweizer-Messer/sm_matrix_archive/include/sm/MatrixArchive.hpp` | 131 |
| `get` | Method | `Schweizer-Messer/sm_property_tree/include/sm/BoostPropertyTreeImplementation.hpp` | 101 |
| `set` | Method | `Schweizer-Messer/sm_property_tree/include/sm/BoostPropertyTreeImplementation.hpp` | 130 |
| `setRandom` | Method | `aslam_cv/aslam_cameras/include/aslam/KeypointIdentifier.hpp` | 29 |
| `random` | Method | `Schweizer-Messer/sm_common/include/sm/hash_id.hpp` | 34 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Test | 5 calls |
| Io | 1 calls |

## How to Explore

1. `context({name: "TEST"})` — see callers and callees
2. `query({query: "sm"})` — find related execution flows
3. Read key files listed above for implementation details
