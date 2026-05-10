# UAM Hexacopter MDO Framework Manual

## 1. Purpose

This framework supports a staged multidisciplinary design study for a
fault-tolerant hexacopter.

- Stage 1: design optimization
- Stage 2: mission validation

Stage 1 is the primary optimization layer. Stage 2 is the secondary
validation layer.

## 2. Public Interface

The framework uses two top-level concepts:

- `cfg = mdo_config()`
  framework configuration
- `opt = UAMOptions(cfg, ...)`
  call-level behavior

Use `cfg` to set variables, objectives, optimizer settings, and base
simulation settings. Use `UAMOptions` to choose how one function call should
run.

## 3. Main Files

```text
Framework/
  main_mdo.m
  config/mdo_config.m
  core/UAMOptions.m
  core/hexacopter_params.m
  evaluation/eval_design.m
  evaluation/eval_stage1.m
  evaluation/eval_stage2.m
  evaluation/eval_simulation.m
  evaluation/eval_mission_figure8.m
  optimization/run_soo.m
  optimization/run_moo.m
  analysis/compare_designs.m
  analysis/sweep_design_space.m
  analysis/pareto_analysis.m
  analysis/visualize_results.m
```

## 4. Stage Structure

### Stage 1

Stage 1 evaluates design-side fault-tolerance:

- ACS metrics
- single-motor-fault hover feasibility
- parameterized cost and mass effects

Main outputs:

- `FII`
- `WCFR`
- `PFWAR`
- `hover_margin`
- `J_cost`
- `J_combined`
- `feasible`

### Stage 2

Stage 2 evaluates mission-level closed-loop behavior:

- hover recovery
- figure-8 tracking
- altitude error
- attitude excursion
- control effort
- divergence

Main outputs:

- `path_rmse`
- `alt_rmse`
- `total_rmse`
- `recovery_time`
- `max_att_excurs`
- `ctrl_effort`
- `diverged`
- `J_mission`

## 5. Design Variables

Default active variables:

- `Lx`
- `Lyi`
- `Lyo`
- `T_max`

Reserved but frozen by default:

- `d_prop`

These are controlled in `cfg.vars.*`.

## 6. Objectives

### Stage 1

Configured in `cfg.objectives.stage1`:

```matlab
cfg.objectives.stage1.names     = {'mass', 'power', 'fault_thrust', 'fault_alloc', 'hover_nom'};
cfg.objectives.stage1.weights   = [0.20, 0.20, 0.25, 0.25, 0.10];
cfg.objectives.stage1.moo_names = {'mass', 'power', 'fault'};
```

### Stage 2

Configured in `cfg.objectives.stage2`:

```matlab
cfg.objectives.stage2.names   = {'mission'};
cfg.objectives.stage2.weights = 1.0;
```

## 7. Vehicle Model

The default framework path uses the parameterized vehicle model in
`vehicle_model.m`.

Current coupling:

- motor mass varies with `T_max`
- frame mass varies with geometry
- total mass varies with design
- inertia varies with design

This is the main parameterized-design contribution of the current framework.

## 8. Controller Handling

Stage 2 simulation uses analytically scaled gains through
`tune_sim_gains.m`.

Intent:

- keep controller bandwidth roughly comparable across designs
- avoid penalizing compact or heavy designs only because of fixed baseline gains

Controller gains are not Stage 1 optimization variables.

## 9. Main Usage

### Baseline Stage 1 evaluation

```matlab
cfg = mdo_config();
d = design_default();
opt = UAMOptions(cfg, 'Mode', 'acs', 'Verbose', true);
r = eval_design(d, opt);
```

### Stage 1 SOO

```matlab
cfg = mdo_config();
cfg.eval.mode = 'acs';
[d_opt, J_opt, hist] = run_soo(design_default(), cfg);
```

### Stage 1 MOO

```matlab
cfg = mdo_config();
cfg.eval.mode = 'acs';
[pareto_designs, pareto_J] = run_moo(design_default(), cfg);
```

### Stage 2 mission validation

```matlab
cfg = mdo_config();
mission_cfg = cfg.sim;
mission_cfg.scenario = 'figure8';
opt = UAMOptions(cfg, ...
    'Mode', 'full', ...
    'Fault', [1;0;0;0;0;0], ...
    'SimConfig', mission_cfg, ...
    'ObjectiveSet', 'stage2');
r = eval_design(design_default(), opt);
```

## 10. `main_mdo.m`

`main_mdo.m` is the canonical demonstration script.

Current sections:

1. baseline evaluation
2. Stage 1 sweeps
3. Stage 1 SOO
4. Stage 1 MOO
5. design comparison
6. Stage 2 mission verification
7. ACS visualization

It now uses `UAMOptions` consistently for all call-level behavior.

## 11. Team Interpretation

Safe current claims:

- Stage 1 design optimization is implemented and usable
- Stage 2 mission validation is implemented and usable as a secondary check
- the code is structured to extend variables and objectives later

Claims to avoid:

- that Stage 2 is already the primary optimization loop
- that the current cost model is a complete structural design model
- that controller tuning is the primary contribution

## 12. Current Limitations

- Stage 1 variable set is still compact
- the cost model is still a surrogate
- allocation logic is still simplified
- degraded-fault and multifault optimization are future extensions

## 13. Recommended Workflow

1. decide candidate Stage 1 variables
2. run Stage 1 SOO and MOO
3. choose representative candidate designs
4. run Stage 2 validation on those candidates
5. decide whether Stage 2 should later move into the optimization loop
