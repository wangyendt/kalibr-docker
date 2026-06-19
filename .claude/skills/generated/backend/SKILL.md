---
name: backend
description: "Skill for the Backend area of kalibr-docker. 89 symbols across 31 files."
---

# Backend

89 symbols | 31 files | Cohesion: 93%

## When to Use

- Working with code in `aslam_optimizer/`
- Understanding how testSpline, toErrorTerm, toErrorTermSqrt work
- Modifying backend-related functionality

## Key Files

| File | Symbols |
|------|---------|
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | EuclideanExpressionNode, EuclideanExpressionNodeMultiply, EuclideanExpressionNodeMatrixMultiply, EuclideanExpressionNodeCrossEuclidean, EuclideanExpressionNodeAddEuclidean (+10) |
| `aslam_nonparametric_estimation/aslam_splines/include/aslam/splines/BSplineExpressions.hpp` | BSplinePositionExpressionNode, BSplineVelocityExpressionNode, BSplineAccelerationExpressionNode, BSplineAccelerationBodyFrameExpressionNode, BSplineAngularVelocityBodyFrameExpressionNode (+4) |
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/ScalarExpressionNode.hpp` | ScalarExpressionNode, ScalarExpressionNodeMultiply, ScalarExpressionNodeDivide, ScalarExpressionNodeAdd, ScalarExpressionNodeConstant (+1) |
| `aslam_optimizer/aslam_backend/src/Optimizer2.cpp` | Optimizer2, Optimizer2, initializeTrustRegionPolicy, initializeLinearSolver, evaluateError (+1) |
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/QuaternionExpression.hpp` | QuaternionExpression, createQuatVal, UnitQuaternionExpression, assertUnitNorm, inverse |
| `aslam_optimizer/aslam_backend_expressions/src/ScalarExpression.cpp` | operator-, operator/, operator+, operator* |
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/TransformationExpressionNode.hpp` | TransformationExpressionNode, TransformationExpressionNodeMultiply, TransformationExpressionNodeInverse, TransformationExpressionNodeConstant |
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/RotationExpressionNode.hpp` | RotationExpressionNode, RotationExpressionNodeMultiply, RotationExpressionNodeInverse, RotationExpressionNodeTransformation |
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/HomogeneousExpressionNode.hpp` | HomogeneousExpressionNode, HomogeneousExpressionNodeMultiply, HomogeneousExpressionNodeConstant, HomogeneousExpressionNodeEuclidean |
| `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/GenericMatrixExpression.hpp` | getDesignVariables, getDesignVariablesImplementation, root, create |

## Entry Points

Start here when exploring this area:

- **`testSpline`** (Function) — `aslam_nonparametric_estimation/aslam_splines/test/TestOPTBSpline.cpp:484`
- **`toErrorTerm`** (Function) — `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/ExpressionErrorTerm.hpp:118`
- **`toErrorTermSqrt`** (Function) — `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/ExpressionErrorTerm.hpp:129`
- **`convertToGME`** (Function) — `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/VectorExpressionToGenericMatrixTraits.hpp:18`
- **`qExp`** (Function) — `aslam_optimizer/aslam_backend_expressions/test/QuaternionExpression.cpp:36`

## Key Symbols

| Symbol | Type | File | Line |
|--------|------|------|------|
| `BSplinePositionExpressionNode` | Class | `aslam_nonparametric_estimation/aslam_splines/include/aslam/splines/BSplineExpressions.hpp` | 61 |
| `BSplineVelocityExpressionNode` | Class | `aslam_nonparametric_estimation/aslam_splines/include/aslam/splines/BSplineExpressions.hpp` | 80 |
| `BSplineAccelerationExpressionNode` | Class | `aslam_nonparametric_estimation/aslam_splines/include/aslam/splines/BSplineExpressions.hpp` | 101 |
| `BSplineAccelerationBodyFrameExpressionNode` | Class | `aslam_nonparametric_estimation/aslam_splines/include/aslam/splines/BSplineExpressions.hpp` | 119 |
| `BSplineAngularVelocityBodyFrameExpressionNode` | Class | `aslam_nonparametric_estimation/aslam_splines/include/aslam/splines/BSplineExpressions.hpp` | 143 |
| `BSplineAngularAccelerationBodyFrameExpressionNode` | Class | `aslam_nonparametric_estimation/aslam_splines/include/aslam/splines/BSplineExpressions.hpp` | 162 |
| `EuclideanExpressionNode` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 21 |
| `EuclideanExpressionNodeMultiply` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 53 |
| `EuclideanExpressionNodeMatrixMultiply` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 83 |
| `EuclideanExpressionNodeCrossEuclidean` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 112 |
| `EuclideanExpressionNodeAddEuclidean` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 138 |
| `EuclideanExpressionNodeConstant` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 189 |
| `EuclideanExpressionNodeNegated` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 239 |
| `EuclideanExpressionNodeScalarMultiply` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 262 |
| `VectorExpression2EuclideanExpressionAdapter` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 286 |
| `EuclideanExpressionNodeTranslation` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 301 |
| `EuclideanExpressionNodeRotationParameters` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 319 |
| `EuclideanExpressionNodeFromHomogeneous` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 338 |
| `EuclideanExpressionNodeElementwiseMultiplyEuclidean` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/EuclideanExpressionNode.hpp` | 361 |
| `ScalarExpressionNode` | Class | `aslam_optimizer/aslam_backend_expressions/include/aslam/backend/ScalarExpressionNode.hpp` | 17 |

## Execution Flows

| Flow | Type | Steps |
|------|------|-------|
| `Optimize → SparseCholeskyLinearSolverOptions` | cross_community | 5 |
| `Optimize → LevenbergMarquardtTrustRegionPolicy` | cross_community | 4 |
| `Optimize → DogLegTrustRegionPolicy` | cross_community | 4 |

## Connected Areas

| Area | Connections |
|------|-------------|
| Test | 2 calls |

## How to Explore

1. `context({name: "testSpline"})` — see callers and callees
2. `query({query: "backend"})` — find related execution flows
3. Read key files listed above for implementation details
