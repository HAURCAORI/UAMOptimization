# Framework Overview

## 1. Current Status

The framework now has two connected layers:

- Stage 1: design optimization
- Stage 2: mission validation

Stage 1 is the main working optimization layer.
Stage 2 is implemented and usable, but it is currently a secondary
validation layer rather than the primary optimization loop.

---

## 2. What Is Implemented

### Stage 1

- ACS-based fault-tolerance evaluation
- single-motor-fault hover screening
- parameterized mass and inertia model
- cost surrogate tied to the vehicle model
- SOO workflow
- MOO workflow
- Pareto analysis and design comparison

### Stage 2

- hover simulation path
- figure-8 mission simulation path
- normalized simulation config handling
- analytic gain scaling for fairer cross-design evaluation
- mission verification block in `main_mdo.m`

---

## 3. Current Design Variables

Active by default:

- `Lx`
- `Lyi`
- `Lyo`
- `T_max`

Reserved but frozen by default:

- `d_prop`

The code is structured so more variables can be added later without
rewriting the evaluation or optimization flow.

---

## 4. Current Objectives

### Stage 1 objectives

Current default:

- `FII`
- `hover`
- `cost`

Configured through:

```matlab
cfg.objectives.stage1.names
cfg.objectives.stage1.weights
cfg.objectives.stage1.moo_names
```

### Stage 2 objective

Current default:

- `mission`

Configured through:

```matlab
cfg.objectives.stage2.names
cfg.objectives.stage2.weights
```

At the current stage, Stage 2 objectives are used for validation and
comparison, not for the main optimization loop.

---

## 5. Optimization Structure

### Stage 1

Primary optimization problem.

Meaning:

- optimize design variables
- screen infeasible designs early
- compare design trade-offs before mission-level tuning

### Stage 2

Secondary validation problem.

Meaning:

- test whether selected Stage 1 designs are controllable
- evaluate mission tracking and recovery
- support later controller tuning if needed

This keeps the project focused on MDO rather than turning the main problem
into controller co-design too early.

---

## 6. Current Interpretation

What can be claimed safely now:

- Stage 1 optimization is functioning
- candidate designs can be ranked by fault-tolerance and cost-related metrics
- Stage 2 mission evaluation exists for practical validation

What should not be overstated:

- that the final problem is already a full UAM mission optimization
- that controller optimization is the main contribution
- that the current cost model is a complete structural design model

---

## 7. Recommended Team Decision Points

The next team decisions should be:

1. final Stage 1 design variables
2. final Stage 1 objective set
3. whether any additional propulsion or structural variables should be added
4. how much of Stage 2 should remain validation-only versus later becoming
   part of the optimization

---

## 8. Near-Term Direction

Recommended workflow:

1. keep Stage 1 as the primary optimization path
2. use Stage 2 to validate representative designs
3. decide final decision variables and objectives with the team
4. then extend the framework only where the final formulation requires it

This is the most explainable and course-appropriate structure for the
current project.
