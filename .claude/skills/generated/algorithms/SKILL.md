---
name: algorithms
description: "Skill for the Algorithms area of kalibr-docker. 34 symbols across 8 files."
---

# Algorithms

34 symbols | 8 files | Cohesion: 95%

## When to Use

- Working with code in `aslam_incremental_calibration/`
- Understanding how columnSubmatrix, rowSubmatrix, colNorm work
- Modifying algorithms-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | columnSubmatrix, rowSubmatrix, colNorm, columnScalingMatrix, cholmodSparseToEigenDenseCopy (+11) |
| `aslam_incremental_calibration/incremental_calibration/src/core/LinearSolver.cpp` | ~LinearSolver, solveSystem, solve, analyzeMarginal, clear |
| `aslam_incremental_calibration/incremental_calibration/src/core/OptimizationProblem.cpp` | getGroupId, isGroupInProblem, addDesignVariable, isDesignVariableInProblem, addErrorTerm |
| `aslam_incremental_calibration/incremental_calibration/src/exceptions/InvalidOperationException.cpp` | InvalidOperationException, InvalidOperationException |
| `aslam_incremental_calibration/incremental_calibration/src/algorithms/marginalize.cpp` | marginalJacobian, marginalize |
| `aslam_incremental_calibration/incremental_calibration/src/base/Mutex.cpp` | unlock, safeUnlock |
| `aslam_incremental_calibration/incremental_calibration/src/core/IncrementalOptimizationProblem.cpp` | remove |
| `aslam_incremental_calibration/incremental_calibration/src/exceptions/NullPointerException.cpp` | NullPointerException |

## Entry Points

Start here when exploring this area:

- **`columnSubmatrix`** (Function) — `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp:41`
- **`rowSubmatrix`** (Function) — `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp:74`
- **`colNorm`** (Function) — `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp:106`
- **`columnScalingMatrix`** (Function) — `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp:128`
- **`cholmodSparseToEigenDenseCopy`** (Function) — `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp:154`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `columnSubmatrix` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 41 |
| `rowSubmatrix` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 74 |
| `colNorm` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 106 |
| `columnScalingMatrix` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 128 |
| `cholmodSparseToEigenDenseCopy` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 154 |
| `eigenDenseToCholmodDenseView` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 173 |
| `eigenDenseToCholmodSparseCopy` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 211 |
| `estimateNumericalRank` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 244 |
| `rankTol` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 256 |
| `qrTol` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 263 |
| `svGap` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 274 |
| `reduceLeftHandSide` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 284 |
| `reduceRightHandSide` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 339 |
| `analyzeSVD` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 412 |
| `solveSVD` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 426 |
| `solveQR` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/linalg.cpp` | 445 |
| `marginalJacobian` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/marginalize.cpp` | 52 |
| `marginalize` | Function | `aslam_incremental_calibration/incremental_calibration/src/algorithms/marginalize.cpp` | 130 |
| `remove` | Method | `aslam_incremental_calibration/incremental_calibration/src/core/IncrementalOptimizationProblem.cpp` | 387 |
| `~LinearSolver` | Method | `aslam_incremental_calibration/incremental_calibration/src/core/LinearSolver.cpp` | 79 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `SolveSystem → NullPointerException` | intra_community | 5 |
| `SolveSystem → InvalidOperationException` | intra_community | 4 |
| `Marginalize → CrossMx` | cross_community | 4 |
| `Marginalize → QuatIdentity` | cross_community | 4 |
| `Lock → InvalidOperationException` | cross_community | 3 |
| `Add → InvalidOperationException` | cross_community | 3 |
| `SolveSystem → Clear` | intra_community | 3 |
| `AnalyzeMarginal → NullPointerException` | intra_community | 3 |
| `AnalyzeMarginal → InvalidOperationException` | intra_community | 3 |
| `Marginalize → InvalidOperationException` | intra_community | 3 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Cluster_36 | 1 calls |

## How to Explore

1. `context({name: "columnSubmatrix"})` — see callers and callees
2. `query({query: "algorithms"})` — find related execution flows
3. Read key files listed above for implementation details
