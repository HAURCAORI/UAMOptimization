# UAMOptimization TODO

## Scope

This file tracks the implementation roadmap for evolving the current
hexacopter fault-tolerant framework into an extensible MDO project for a
parameterized UAM/eVTOL-like vehicle model.

Current agreement:

- Keep the current basic design variables first and make the code extensible.
- Use `Src/sim_path_following.m` as the mission reference for now.
- Treat the project as MDO-focused, not control-only.
- Build the optimization in stages:
  1. Stage 1 design optimization
  2. Stage 2 controller tuning / mission validation
  3. Fault-model expansion
- Controller optimization is needed, but it is not the primary optimization
  problem. It should be used as validation / secondary tuning after Stage 1.

## Priority 0: Architecture Cleanup

- [x] Define the framework around extensible parameter sets instead of a fixed
      four-variable assumption.
- [x] Keep `main_mdo.m` allowed to stay hard-coded for orchestration only.
- [x] Remove hard-coded assumptions from core/evaluation/optimization
      functions where possible.
- [x] Standardize config naming and flow across all modules.
- [ ] Fix text encoding corruption in comments, manual files, and printed
      console output.

## Priority 1: Parameterized Design Model

- [x] Keep current implemented parameters as the baseline active set:
      `Lx`, `Lyi`, `Lyo`, `T_max`.
- [x] Refactor the design/config pipeline so new parameters can be added
      without rewriting evaluator logic.
- [ ] Separate:
      geometry parameters,
      propulsion parameters,
      structural/inertial surrogate parameters,
      safety/mission settings.
- [x] Upgrade the current mass/inertia/cost surrogate into a clearer
      parameterized design model.
- [ ] Document which outputs are derived from design variables and which stay
      fixed assumptions.

## Priority 2: Rough Optimization Implementation

### Stage 1: Primary Design Optimization

- [x] Keep Stage 1 as the primary optimization problem.
- [x] Use current active variables as the rough first implementation:
      `Lx`, `Lyi`, `Lyo`, `T_max`.
- [x] Keep the code extensible so new design variables can be added later.
- [x] Use ACS / hover / cost objectives for the rough optimization pass.
- [x] Keep Stage 1 fast and stable enough for repeated SOO / MOO runs.
- [x] Make Stage 1 objective and constraints configurable from
      `config/mdo_config.m`.

Candidate Stage 1 checks:

- [x] Single-fault hover feasibility
- [ ] Hover allocation feasibility
- [ ] Worst-case utilization / saturation margin
- [x] ACS retention / survivability metrics
- [x] Cost / mass surrogate consistency

Current rough Stage 1 objective direction:

- [x] `FII`
- [x] `J_hover`
- [x] `J_cost`

### Stage 2: Secondary Controller Tuning / Mission Validation

- [x] Do not treat controller gains as the primary design variables.
- [x] Use Stage 2 mainly to validate selected Stage 1 candidate designs.
- [x] Allow controller gain adjustment as a secondary tuning problem only.
- [x] Keep Stage 2 separate from the main Stage 1 design optimization loop.
- [x] Build a mission evaluator inside `Framework/evaluation` using
      `Src/sim_path_following.m` as the starting reference.
- [x] Use mission/path-following results to compare promising Stage 1
      candidates, not to replace Stage 1.

## Priority 3: Mission Integration

- [x] Port figure-8 tracking logic from `Src/sim_path_following.m` into the
      framework as a reusable function.
- [x] Avoid leaving mission simulation as a standalone legacy script.
- [x] Define mission metrics suitable for optimization, not only plotting.

Candidate mission metrics:

- [x] Path tracking RMSE
- [x] Altitude tracking RMSE
- [x] Attitude excursion during mission
- [x] Recovery after fault injection during mission
- [x] Control effort / thrust usage

Stage 2 role:

- [ ] validation of Stage 1 candidate designs
- [ ] secondary controller tuning if needed
- [ ] practical mission-capability check

## Priority 4: Control Allocation Upgrade

- [ ] Review allocation approaches in the cited fault-tolerant references.
- [ ] Make the allocation strategy selectable/configurable.
- [ ] Include allocation residual or infeasibility measures in evaluation.

Verification items:

- [ ] Confirm how negative thrust commands are handled.
- [ ] Confirm whether post-clipping behavior distorts feasibility claims.
- [ ] Check whether mission and hover conclusions change under improved
      allocation logic.

## Priority 5: Fault Model Expansion

### Phase 1

- [x] Full single-motor failure

### Phase 2

- [ ] Partial loss-of-effectiveness (degraded motor)
- [ ] Support configurable LOE magnitudes instead of only 0/1 faults

### Phase 3

- [ ] Multi-fault scenarios
- [ ] Expand ACS and simulation evaluation to cover these scenarios

## Priority 6: Verification and Metric Review

- [x] Verify the current recovery-time definition.
- [x] Check whether recovery means "first re-entry" or "re-entry and stay
      recovered".
- [x] Verify that analytic gain scaling is used consistently in simulation.
- [x] Check whether optimized Stage 1 designs remain stable under scaled
      gains.
- [ ] Reassess the current novelty claims and keep only what can be defended.
- [x] Verify that simulation settings from config are actually respected.
- [x] Fix the known `fault_time` vs `t_fault` config mismatch.

## Priority 7: Practical / Report-Quality Improvements

- [ ] Clean up comments, naming, and user-facing text.
- [ ] Improve documentation for the staged optimization workflow.
- [ ] Make final figures and tables consistent with the MDO story.
- [ ] Keep the framework usable even before the full parameter set is decided.

## Immediate Next Implementation Steps

- [x] Finalize rough Stage 1 optimization with the current active variables.
- [x] Keep Stage 1 objectives limited to ACS / hover / cost terms.
- [ ] Share Stage 1 results with teammates to decide final design variables.
- [ ] After that decision, extend the design-variable set if needed.
- [ ] Integrate path-following mission simulation into `Framework`.
- [ ] Use Stage 2 to validate or tune selected Stage 1 candidates.
- [ ] Upgrade control allocation logic after rough Stage 1 is stabilized.

## Notes

- The current repository is a strong prototype, not the final project form.
- The first implementation pass should preserve current functionality while
  making extension easier.
- New design variables will be added later after the parameterized model
  direction is finalized.
