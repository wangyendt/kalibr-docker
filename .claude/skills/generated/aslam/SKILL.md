---
name: aslam
description: "Skill for the Aslam area of kalibr-docker. 17 symbols across 6 files."
---

# Aslam

17 symbols | 6 files | Cohesion: 92%

## When to Use

- Working with code in `aslam_cv/`
- Understanding how aslam_wallsleep, exportGenericProjectionDesignVariable, exportShutterDesignVariable work
- Modifying aslam-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_cv/aslam_time/include/aslam/Time.hpp` | operator+, isZero, useSystemTime, TimeBase, fromSec (+2) |
| `aslam_cv/aslam_time/include/aslam/Duration.hpp` | DurationBase, fromSec, Duration, WallDuration |
| `aslam_cv/aslam_time/src/time.cpp` | aslam_wallsleep, sleep |
| `aslam_cv/aslam_cv_backend_python/include/aslam/ExportCameraDesignVariable.hpp` | exportGenericProjectionDesignVariable, exportShutterDesignVariable |
| `aslam_cv/aslam_cv_backend_python/src/module.cpp` | BOOST_PYTHON_MODULE |
| `aslam_optimizer/aslam_backend_python/include/aslam/python/ExportDesignVariableAdapter.hpp` | exportDesignVariableAdapter |

## Entry Points

Start here when exploring this area:

- **`aslam_wallsleep`** (Function) — `aslam_cv/aslam_time/src/time.cpp:190`
- **`exportGenericProjectionDesignVariable`** (Function) — `aslam_cv/aslam_cv_backend_python/include/aslam/ExportCameraDesignVariable.hpp:38`
- **`exportShutterDesignVariable`** (Function) — `aslam_cv/aslam_cv_backend_python/include/aslam/ExportCameraDesignVariable.hpp:44`
- **`BOOST_PYTHON_MODULE`** (Function) — `aslam_cv/aslam_cv_backend_python/src/module.cpp:18`
- **`exportDesignVariableAdapter`** (Function) — `aslam_optimizer/aslam_backend_python/include/aslam/python/ExportDesignVariableAdapter.hpp:13`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `DurationBase` | Class | `aslam_cv/aslam_time/include/aslam/Duration.hpp` | 71 |
| `Duration` | Class | `aslam_cv/aslam_time/include/aslam/Duration.hpp` | 121 |
| `WallDuration` | Class | `aslam_cv/aslam_time/include/aslam/Duration.hpp` | 149 |
| `TimeBase` | Class | `aslam_cv/aslam_time/include/aslam/Time.hpp` | 118 |
| `Time` | Class | `aslam_cv/aslam_time/include/aslam/Time.hpp` | 180 |
| `WallTime` | Class | `aslam_cv/aslam_time/include/aslam/Time.hpp` | 232 |
| `aslam_wallsleep` | Function | `aslam_cv/aslam_time/src/time.cpp` | 190 |
| `exportGenericProjectionDesignVariable` | Function | `aslam_cv/aslam_cv_backend_python/include/aslam/ExportCameraDesignVariable.hpp` | 38 |
| `exportShutterDesignVariable` | Function | `aslam_cv/aslam_cv_backend_python/include/aslam/ExportCameraDesignVariable.hpp` | 44 |
| `BOOST_PYTHON_MODULE` | Function | `aslam_cv/aslam_cv_backend_python/src/module.cpp` | 18 |
| `exportDesignVariableAdapter` | Function | `aslam_optimizer/aslam_backend_python/include/aslam/python/ExportDesignVariableAdapter.hpp` | 13 |
| `operator+` | Method | `aslam_cv/aslam_time/include/aslam/Time.hpp` | 137 |
| `isZero` | Method | `aslam_cv/aslam_time/include/aslam/Time.hpp` | 165 |
| `useSystemTime` | Method | `aslam_cv/aslam_time/include/aslam/Time.hpp` | 206 |
| `sleep` | Method | `aslam_cv/aslam_time/src/time.cpp` | 291 |
| `fromSec` | Method | `aslam_cv/aslam_time/include/aslam/Duration.hpp` | 108 |
| `fromSec` | Method | `aslam_cv/aslam_time/include/aslam/Time.hpp` | 154 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `BOOST_PYTHON_MODULE → ExportDesignVariableAdapter` | intra_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Implementation | 5 calls |

## How to Explore

1. `context({name: "aslam_wallsleep"})` — see callers and callees
2. `query({query: "aslam"})` — find related execution flows
3. Read key files listed above for implementation details
