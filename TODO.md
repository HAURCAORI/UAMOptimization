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
  1. Dynamics/safety feasibility optimization
  2. Mission-based optimization on feasible designs
  3. Fault-model expansion

## Priority 0: Architecture Cleanup

- [ ] Define the framework around extensible parameter sets instead of a fixed
      four-variable assumption.
- [ ] Keep `main_mdo.m` allowed to stay hard-coded for orchestration only.
- [ ] Remove hard-coded assumptions from core/evaluation/optimization
      functions where possible.
- [ ] Standardize config naming and flow across all modules.
- [ ] Fix text encoding corruption in comments, manual files, and printed
      console output.

## Priority 1: Parameterized Design Model

- [ ] Keep current implemented parameters as the baseline active set:
      `Lx`, `Lyi`, `Lyo`, `T_max`.
- [ ] Refactor the design/config pipeline so new parameters can be added
      without rewriting evaluator logic.
- [ ] Separate:
      geometry parameters,
      propulsion parameters,
      structural/inertial surrogate parameters,
      safety/mission settings.
- [ ] Upgrade the current mass/inertia/cost surrogate into a clearer
      parameterized design model.
- [ ] Document which outputs are derived from design variables and which stay
      fixed assumptions.

## Priority 2: Two-Stage Optimization Workflow

### Stage 1: Feasibility / Dynamics / Safety

- [ ] Formalize a Stage 1 evaluator that screens infeasible designs before
      mission optimization.
- [ ] Include ACS-based survivability and dynamic feasibility checks.
- [ ] Define Stage 1 outputs so they can be reused directly by Stage 2.
- [ ] Make Stage 1 objective and constraints configurable from
      `config/mdo_config.m`.

Candidate Stage 1 checks:

- [ ] Single-fault hover feasibility
- [ ] Hover allocation feasibility
- [ ] Worst-case utilization / saturation margin
- [ ] Dynamic non-divergence after fault
- [ ] Attitude excursion / recovery screening

### Stage 2: Mission-Based Optimization

- [ ] Build a mission evaluator inside `Framework/evaluation` using
      `Src/sim_path_following.m` as the starting reference.
- [ ] Run mission optimization only on designs that pass Stage 1.
- [ ] Support combined objectives across fault tolerance, mission tracking,
      and practical design penalties.
- [ ] Keep the implementation general so additional mission scenarios can be
      added later.

## Priority 3: Mission Integration

- [ ] Port figure-8 tracking logic from `Src/sim_path_following.m` into the
      framework as a reusable function.
- [ ] Avoid leaving mission simulation as a standalone legacy script.
- [ ] Define mission metrics suitable for optimization, not only plotting.

Candidate mission metrics:

- [ ] Path tracking RMSE
- [ ] Altitude tracking RMSE
- [ ] Attitude excursion during mission
- [ ] Recovery after fault injection during mission
- [ ] Control effort / thrust usage

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

- [ ] Full single-motor failure

### Phase 2

- [ ] Partial loss-of-effectiveness (degraded motor)
- [ ] Support configurable LOE magnitudes instead of only 0/1 faults

### Phase 3

- [ ] Multi-fault scenarios
- [ ] Expand ACS and simulation evaluation to cover these scenarios

## Priority 6: Verification and Metric Review

- [ ] Verify the current recovery-time definition.
- [ ] Check whether recovery means "first re-entry" or "re-entry and stay
      recovered".
- [ ] Reassess the current novelty claims and keep only what can be defended.
- [ ] Verify that simulation settings from config are actually respected.
- [ ] Fix the known `fault_time` vs `t_fault` config mismatch.

## Priority 7: Practical / Report-Quality Improvements

- [ ] Clean up comments, naming, and user-facing text.
- [ ] Improve documentation for the staged optimization workflow.
- [ ] Make final figures and tables consistent with the MDO story.
- [ ] Keep the framework usable even before the full parameter set is decided.

## Immediate Next Implementation Steps

- [ ] Create a clean staged evaluation architecture:
      feasibility evaluator,
      mission evaluator,
      combined design evaluator.
- [ ] Refactor config handling so the same config structure drives all stages.
- [ ] Integrate path-following mission simulation into `Framework`.
- [ ] Upgrade control allocation logic.
- [ ] Rework objective/constraint definitions for Stage 1 and Stage 2.

## Notes

- The current repository is a strong prototype, not the final project form.
- The first implementation pass should preserve current functionality while
  making extension easier.
- New design variables will be added later after the parameterized model
  direction is finalized.
