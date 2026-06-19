---
name: interp-rotation
description: "Skill for the Interp_rotation area of kalibr-docker. 29 symbols across 5 files."
---

# Interp_rotation

29 symbols | 5 files | Cohesion: 82%

## When to Use

- Working with code in `aslam_nonparametric_estimation/`
- Understanding how qexp, qdot, qinv work
- Modifying interp_rotation-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | qexp, qdot, qinv, cumQuat, invS (+8) |
| `aslam_nonparametric_estimation/bsplines/interp_rotation/invariance.py` | qlog, qdot, cumQuat2, qexp, qinv (+1) |
| `aslam_nonparametric_estimation/bsplines/interp_rotation/invariance2.py` | qdot, qinv, cumQuat, cumQuat2 |
| `aslam_nonparametric_estimation/bsplines/interp_rotation/testThreeManifold.py` | gotoSplinePos, step, goto |
| `aslam_nonparametric_estimation/bsplines/interp_rotation/threeManifoldVisual/__init__.py` | setCurrentPos, __step, startInteractionThread |

## Entry Points

Start here when exploring this area:

- **`qexp`** (Function) — `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py:57`
- **`qdot`** (Function) — `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py:71`
- **`qinv`** (Function) — `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py:74`
- **`cumQuat`** (Function) — `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py:113`
- **`invS`** (Function) — `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py:150`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `qexp` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 57 |
| `qdot` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 71 |
| `qinv` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 74 |
| `cumQuat` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 113 |
| `invS` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 150 |
| `qfunc` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 173 |
| `qfuncJac` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 180 |
| `dqinv` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 219 |
| `gotoSplinePos` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/testThreeManifold.py` | 186 |
| `qeps` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 46 |
| `qeta` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 49 |
| `qlog` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 52 |
| `ljac` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 139 |
| `qlog2` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/jacobians.py` | 233 |
| `qdot` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/invariance2.py` | 50 |
| `qinv` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/invariance2.py` | 53 |
| `cumQuat` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/invariance2.py` | 92 |
| `cumQuat2` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/invariance2.py` | 127 |
| `qlog` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/invariance.py` | 49 |
| `qdot` | Function | `aslam_nonparametric_estimation/bsplines/interp_rotation/invariance.py` | 63 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `QfuncJac → Qeta` | cross_community | 3 |
| `QfuncJac → Qeps` | cross_community | 3 |
| `CumQuat → Qeta` | cross_community | 3 |
| `CumQuat → Qeps` | cross_community | 3 |
| `CumQuat → Qeta` | cross_community | 3 |
| `CumQuat → Qeps` | cross_community | 3 |
| `Ljac → Qeta` | intra_community | 3 |
| `Ljac → Qeps` | intra_community | 3 |
| `StartInteractionThread → SetCurrentPos` | intra_community | 3 |
| `StartInteractionThread → GetCurrentVecPos` | intra_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| DiffManifolds | 1 calls |

## How to Explore

1. `context({name: "qexp"})` — see callers and callees
2. `query({query: "interp_rotation"})` — find related execution flows
3. Read key files listed above for implementation details
