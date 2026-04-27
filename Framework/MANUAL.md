# UAM Hexacopter MDO Framework Manual

## 1. Purpose

This framework supports a staged multidisciplinary design study for a
fault-tolerant hexacopter.

Current project structure:

- Stage 1: design optimization
- Stage 2: mission-level validation

The current implementation is intentionally centered on Stage 1. Stage 2 is
implemented as a secondary validation layer, not the primary optimization
loop.

---

## 2. Main Idea

The framework separates the problem into two parts.

### Stage 1: design-side screening and optimization

Stage 1 evaluates whether a design is promising from the vehicle side:

- control-effectiveness balance under fault
- single-motor-fault hover survivability
- simple parameterized cost and mass effects

This stage is fast and is the default optimization path.

### Stage 2: mission and controller validation

Stage 2 evaluates whether a selected design can perform a closed-loop mission:

- hover-recovery response
- figure-8 mission tracking
- attitude excursion
- control effort
- mission-level divergence or recovery

This stage is slower and is used after Stage 1 candidate designs are found.

---

## 3. Directory Structure

```text
Framework/
  main_mdo.m
  OVERVIEW.md
  MANUAL.md
  config/
    mdo_config.m
  core/
    design_default.m
    vehicle_model.m
    hexacopter_params.m
    build_B_matrix.m
    eom_hex.m
  evaluation/
    eval_stage1.m
    eval_stage2.m
    eval_design.m
    eval_acs.m
    eval_simulation.m
    eval_mission_figure8.m
    figure8_reference.m
    normalize_sim_config.m
    tune_sim_gains.m
  metrics/
    compute_acs_volume.m
    hover_feasibility.m
    compute_hover_threshold.m
    fault_isotropy_index.m
  optimization/
    objective_fcn.m
    constraint_fcn.m
    run_soo.m
    run_cmaes.m
    run_moo.m
  analysis/
    sweep_design_space.m
    pareto_analysis.m
    compare_designs.m
    visualize_results.m
```

---

## 4. Vehicle Model

The baseline design is defined in `design_default.m`.

The active geometry is a tandem hexacopter:

- `Lx`: fore-aft arm length
- `Lyi`: inner lateral arm length
- `Lyo`: outer lateral arm length
- `T_max`: maximum thrust per motor
- `d_prop`: optional propeller diameter variable, frozen by default

The default path uses a parameterized vehicle model in `vehicle_model.m`.
That means:

- motor mass changes with `T_max`
- frame mass changes with arm span
- total mass changes with the design
- inertia changes with the design

This is the main novelty direction of the current implementation: a compact
parameterized design model embedded into optimization.

---

## 5. Current Design Variables

The default active Stage 1 variables are:

- `Lx`
- `Lyi`
- `Lyo`
- `T_max`

The default reserved variable is:

- `d_prop`

All variable settings are controlled from `config/mdo_config.m`.

Relevant config fields:

```matlab
cfg.vars.names
cfg.vars.lb
cfg.vars.ub
cfg.vars.x0
cfg.vars.units
cfg.vars.active
```

To freeze or activate a variable, edit `cfg.vars.active`.

---

## 6. Objectives

### Stage 1 objectives

Stage 1 is objective-registry based.

The default Stage 1 objective set is:

```matlab
cfg.objectives.stage1.names   = {'FII', 'hover', 'cost'};
cfg.objectives.stage1.weights = [0.35, 0.40, 0.25];
```

Current meanings:

- `FII`: fault isotropy index
- `hover`: hover survivability penalty
- `cost`: parameterized design cost proxy

For multi-objective optimization:

```matlab
cfg.objectives.stage1.moo_names = {'FII', 'hover', 'cost'};
```

### Stage 2 objectives

Stage 2 is currently used for validation.

The current Stage 2 objective registry is:

```matlab
cfg.objectives.stage2.names   = {'mission'};
cfg.objectives.stage2.weights = 1.0;
```

This is used in the mission verification block in `main_mdo.m`.

### Legacy weights

`cfg.weights.*` is still present for backward compatibility with older code,
but the current framework direction is:

- use `cfg.objectives.stage1.*` for Stage 1
- use `cfg.objectives.stage2.*` for Stage 2

---

## 7. Stage 1 Evaluation

Stage 1 is implemented through `eval_stage1.m` and used in `eval_design.m`.

Main Stage 1 outputs:

- `FII`
- `WCFR`
- `PFWAR`
- `hover_margin`
- `hover_ok`
- `J_cost`
- `feasible`

Stage 1 feasibility currently means:

- geometry is valid
- ACS screening passes
- single-motor-fault hover screening passes

Stage 1 is the main optimization layer.

It should be described as:

- fault-tolerant design screening
- ACS/hover feasibility optimization

It should not be described as:

- full mission feasibility

---

## 8. Stage 2 Evaluation

Stage 2 is implemented through `eval_stage2.m`.

It supports two scenarios:

- `hover`
- `figure8`

Configured through:

```matlab
cfg.sim.scenario = 'hover';   % or 'figure8'
```

### Hover scenario

This uses `eval_simulation.m` and evaluates:

- altitude tracking
- attitude excursion
- recovery time
- control effort
- divergence

### Figure-8 scenario

This uses `eval_mission_figure8.m` and evaluates:

- `path_rmse`
- `alt_rmse`
- `total_rmse`
- `att_rmse_phi`
- `att_rmse_theta`
- `max_att_excurs`
- `ctrl_effort`
- `recovery_time`
- `diverged`

Mission settings are stored in:

```matlab
cfg.sim.mission.A
cfg.sim.mission.T_period
cfg.sim.mission.z_cruise
cfg.sim.mission.t_start
cfg.sim.mission.n_laps
```

The simulation config path is normalized by `normalize_sim_config.m`, so both
`fault_time` and `t_fault` are supported.

---

## 9. Controller Handling

The simulation controller uses analytic gain scaling rather than fixed gains
for every design.

Current intent:

- keep controller bandwidth approximately comparable across designs
- avoid unfairly destabilizing low-inertia or high-mass designs

This is handled by `tune_sim_gains.m`.

At the current project stage:

- controller gains are not Stage 1 optimization variables
- controller tuning is part of Stage 2 validation logic

---

## 10. Main Functions

### `mdo_config`

Creates the master configuration:

- variables
- objectives
- optimizer settings
- simulation settings

### `design_default`

Creates the baseline design struct.

### `hexacopter_params`

Builds the `UAM`, `Prop`, and `Env` parameter structs from a design.

### `eval_design`

Main entry point for evaluation.

Supported modes:

- `'acs'`
- `'sim'`
- `'full'`

Meaning:

- `'acs'`: Stage 1 only
- `'sim'`: Stage 2 only
- `'full'`: Stage 1 + Stage 2

### `run_soo`

Single-objective optimization driver.

Used for the main Stage 1 optimization run.

### `run_moo`

Multi-objective optimization driver.

Used for Pareto analysis of the current Stage 1 objective set.

### `compare_designs`

Runs a consistent side-by-side evaluation for multiple designs.

### `visualize_results`

Plots ACS, simulation, sweep, and Pareto results.

For mission data it also shows mission tracking plots.

---

## 11. How To Run

### Full framework script

```matlab
cd C:\local\project\UAMOptimization\Framework
main_mdo
```

### Stage 1 baseline evaluation

```matlab
addpath('config','core','evaluation','metrics','optimization','analysis');
d = design_default();
r = eval_design(d, struct('mode','acs','verbose',true));
```

### Stage 1 single-objective optimization

```matlab
cfg = mdo_config();
cfg.eval.mode = 'acs';
[d_opt, J_opt, hist] = run_soo(design_default(), cfg);
```

### Stage 1 multi-objective optimization

```matlab
cfg = mdo_config();
cfg.eval.mode = 'acs';
[pareto_designs, pareto_J] = run_moo(design_default(), cfg);
```

### Stage 2 mission validation

```matlab
cfg = mdo_config();
cfg.sim.scenario = 'figure8';
r = eval_design(design_default(), struct( ...
    'mode', 'full', ...
    'sim_config', cfg.sim, ...
    'loe_for_sim', [1;0;0;0;0;0]));
```

---

## 12. What `main_mdo.m` Does

`main_mdo.m` is a demonstration and reporting script.

Current sections:

1. baseline evaluation
2. design sweeps
3. Stage 1 SOO
4. Stage 1 MOO / Pareto analysis
5. design comparison
6. Stage 2 mission verification
7. ACS visualization

Important note:

- Stage 1 optimization and Stage 2 validation are both present in the script
- the primary optimization remains Stage 1

The Stage 2 mission verification block currently uses:

- `scenario = 'figure8'`
- a short mission
- motor 1 fault
- comparison of baseline / SOO-optimal / Pareto-knee

The Pareto-knee design is then plotted in more detail as the representative
mission case.

---

## 13. Interpretation Guide

### When Stage 1 results are reasonable

Typical healthy trends:

- `FII` decreases for better balanced designs
- `hover_ok` improves when `T_max` rises past the threshold
- SOO prefers lighter/cheaper feasible solutions
- Pareto-knee trades cost for better balance and hover margin

### What Stage 1 does not prove

Stage 1 does not prove:

- mission tracking quality
- controller robustness
- degraded-fault handling
- multifault handling

Those belong to Stage 2 or later extensions.

### What Stage 2 currently means

Stage 2 is currently:

- a secondary validation layer
- useful for checking mission capability trends
- not yet the primary optimization problem

---

## 14. Recommended Team Usage

For team discussion, use the framework in this order:

1. decide candidate Stage 1 design variables
2. run Stage 1 SOO and MOO
3. choose promising candidate designs
4. run Stage 2 mission validation on those candidates
5. decide whether Stage 2 should later become part of the optimization

This keeps the project centered on MDO while still showing practical
controller and mission verification.

---

## 15. Current Limitations

- Stage 1 variable set is still compact
- the cost model is still a surrogate, not a full structural model
- allocation logic is still simplified
- degraded-fault and multifault optimization are not yet the main path
- `main_mdo.m` still contains some legacy comment/encoding clutter outside
  the core logic

These are known limitations, not hidden assumptions.

---

## 16. Short Summary

Current framework status:

- Stage 1 design optimization is implemented and usable
- Stage 2 mission validation is implemented and usable as a secondary check
- the framework is structured to remain extensible as final variables and
  objectives are decided

This is the correct way to present the current codebase to teammates.
