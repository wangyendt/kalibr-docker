---
name: apriltags
description: "Skill for the Apriltags area of kalibr-docker. 16 symbols across 10 files."
---

# Apriltags

16 symbols | 10 files | Cohesion: 78%

## When to Use

- Working with code in `aslam_offline_calibration/`
- Understanding how fimOrig, r, edgeCost work
- Modifying apriltags-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_offline_calibration/ethz_apriltag2/include/apriltags/MathUtil.h` | distance2D, mod2pi, fast_atan2, square |
| `aslam_offline_calibration/ethz_apriltag2/src/Edge.cc` | edgeCost, mergeEdges |
| `aslam_offline_calibration/ethz_apriltag2/include/apriltags/Gaussian.h` | makeGaussianFilter, convolveSymmetricCentered |
| `aslam_offline_calibration/ethz_apriltag2/include/apriltags/UnionFindSimple.h` | getSetSize, getRepresentative |
| `aslam_offline_calibration/ethz_apriltag2/src/Quad.cc` | search |
| `aslam_offline_calibration/ethz_apriltag2/src/TagDetection.cc` | overlapsTooMuch |
| `aslam_offline_calibration/ethz_apriltag2/include/apriltags/Gridder.h` | add |
| `aslam_offline_calibration/ethz_apriltag2/src/TagDetector.cc` | fimOrig |
| `aslam_offline_calibration/ethz_apriltag2/src/FloatImage.cc` | r |
| `aslam_offline_calibration/ethz_apriltag2/src/GLine2D.cc` | lsqFitXYW |

## Entry Points

Start here when exploring this area:

- **`fimOrig`** (Function) — `aslam_offline_calibration/ethz_apriltag2/src/TagDetector.cc:48`
- **`r`** (Function) — `aslam_offline_calibration/ethz_apriltag2/src/FloatImage.cc:47`
- **`edgeCost`** (Method) — `aslam_offline_calibration/ethz_apriltag2/src/Edge.cc:13`
- **`search`** (Method) — `aslam_offline_calibration/ethz_apriltag2/src/Quad.cc:51`
- **`overlapsTooMuch`** (Method) — `aslam_offline_calibration/ethz_apriltag2/src/TagDetection.cc:53`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `fimOrig` | Function | `aslam_offline_calibration/ethz_apriltag2/src/TagDetector.cc` | 48 |
| `r` | Function | `aslam_offline_calibration/ethz_apriltag2/src/FloatImage.cc` | 47 |
| `edgeCost` | Method | `aslam_offline_calibration/ethz_apriltag2/src/Edge.cc` | 13 |
| `search` | Method | `aslam_offline_calibration/ethz_apriltag2/src/Quad.cc` | 51 |
| `overlapsTooMuch` | Method | `aslam_offline_calibration/ethz_apriltag2/src/TagDetection.cc` | 53 |
| `makeGaussianFilter` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/Gaussian.h` | 17 |
| `add` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/Gridder.h` | 69 |
| `getSetSize` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/UnionFindSimple.h` | 20 |
| `getRepresentative` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/UnionFindSimple.h` | 22 |
| `mergeEdges` | Method | `aslam_offline_calibration/ethz_apriltag2/src/Edge.cc` | 68 |
| `convolveSymmetricCentered` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/Gaussian.h` | 29 |
| `lsqFitXYW` | Method | `aslam_offline_calibration/ethz_apriltag2/src/GLine2D.cc` | 53 |
| `distance2D` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/MathUtil.h` | 22 |
| `mod2pi` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/MathUtil.h` | 29 |
| `fast_atan2` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/MathUtil.h` | 43 |
| `square` | Method | `aslam_offline_calibration/ethz_apriltag2/include/apriltags/MathUtil.h` | 20 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `FimOrig → Fabs` | cross_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Test | 1 calls |

## How to Explore

1. `context({name: "fimOrig"})` — see callers and callees
2. `query({query: "apriltags"})` — find related execution flows
3. Read key files listed above for implementation details
