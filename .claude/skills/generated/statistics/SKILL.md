---
name: statistics
description: "Skill for the Statistics area of kalibr-docker. 14 symbols across 14 files."
---

# Statistics

14 symbols | 14 files | Cohesion: 90%

## When to Use

- Working with code in `aslam_incremental_calibration/`
- Understanding how Serializable, Timestamp, VectorDesignVariable work
- Modifying statistics-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/base/Serializable.h` | Serializable |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/base/Timestamp.h` | Timestamp |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/data-structures/VectorDesignVariable.h` | VectorDesignVariable |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/functions/Function.h` | Function |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/Distribution.h` | Distribution |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/GammaDistribution.h` | GammaDistribution |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/NormalDistribution1v.h` | NormalDistribution |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/NormalDistributionMv.h` | NormalDistribution |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/SampleDistribution.h` | SampleDistribution |
| `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/UniformDistribution1v.h` | UniformDistribution |

## Entry Points

Start here when exploring this area:

- **`Serializable`** (Class) — `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/base/Serializable.h:35`
- **`Timestamp`** (Class) — `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/base/Timestamp.h:36`
- **`VectorDesignVariable`** (Class) — `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/data-structures/VectorDesignVariable.h:40`
- **`Function`** (Class) — `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/functions/Function.h:34`
- **`Distribution`** (Class) — `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/Distribution.h:34`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `Serializable` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/base/Serializable.h` | 35 |
| `Timestamp` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/base/Timestamp.h` | 36 |
| `VectorDesignVariable` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/data-structures/VectorDesignVariable.h` | 40 |
| `Function` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/functions/Function.h` | 34 |
| `Distribution` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/Distribution.h` | 34 |
| `GammaDistribution` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/GammaDistribution.h` | 39 |
| `NormalDistribution` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/NormalDistribution1v.h` | 35 |
| `NormalDistribution` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/NormalDistributionMv.h` | 38 |
| `SampleDistribution` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/SampleDistribution.h` | 37 |
| `UniformDistribution` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/UniformDistribution1v.h` | 38 |
| `UniformDistribution` | Class | `aslam_incremental_calibration/incremental_calibration/include/aslam/calibration/statistics/UniformDistributionMv.h` | 38 |
| `DesignVariable` | Class | `aslam_optimizer/aslam_backend/include/aslam/backend/DesignVariable.hpp` | 17 |
| `log` | Method | `Schweizer-Messer/sm_logging/src/Logger.cpp` | 35 |
| `setVariance` | Method | `aslam_incremental_calibration/incremental_calibration/src/statistics/NormalDistribution1v.cpp` | 94 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Boost | 1 calls |

## How to Explore

1. `context({name: "Serializable"})` — see callers and callees
2. `query({query: "statistics"})` — find related execution flows
3. Read key files listed above for implementation details
