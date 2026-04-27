# Stage 1 Overview

## Purpose

Stage 1 is the current **fast screening / optimization layer** for the UAM
hexacopter project.

It is intended to answer:

- Which designs are structurally and actuator-wise plausible?
- Which designs can maintain single-fault hover?
- Which designs distribute fault impact more evenly?
- What is the cost trade-off for improving fault tolerance?

It is **not** the final mission-validation layer.

## What Stage 1 Evaluates

Stage 1 is based on ACS and hover-feasibility analysis.

Main metrics:

- `FII`
  Fault Isotropy Index. Lower is better.
  Measures how unevenly different motor failures damage control authority.

- `WCFR`
  Worst-Case Fault Retention. Higher is better.
  Minimum ACS volume retention across single-motor failures.

- `PFWAR`
  Probabilistic Fault-Weighted ACS Retention.
  In practice this stays near `1/3` for this configuration, so it is not a
  strong optimization lever.

- `hover_ok`
  Number of single-motor fault cases that can still hover.

- `hover_margin`
  Margin above the worst single-fault hover threshold.
  Positive means all single-fault hover cases are feasible.

- `J_cost`
  Current Stage 1 cost proxy.
  In the current model this comes from the parameterized vehicle model
  (motor mass + frame mass), normalized to baseline.

## Current Design Variables

Current active variables:

- `Lx`
- `Lyi`
- `Lyo`
- `T_max`

Currently present but frozen by default:

- `d_prop`

The code is now structured so more variables can be added later without
rewriting the whole framework.

## Current Modeling Assumption

The current default path uses a **parameterized vehicle model**:

- total mass is not fixed
- motor mass scales with `T_max`
- frame mass scales with arm span
- inertia is recomputed from the design

This is the current Stage 1 novelty direction: a compact parameterized
design model embedded inside optimization.

## How To Run Stage 1

### Baseline evaluation

```matlab
addpath('config','core','evaluation','metrics','optimization','analysis');
d = design_default();
r = eval_design(d, struct('mode','acs','verbose',true));
```

### Single-objective optimization

```matlab
cfg = mdo_config();
cfg.eval.mode = 'acs';
[d_opt, J_opt, hist] = run_soo(design_default(), cfg);
```

### Multi-objective optimization

```matlab
d0 = design_default();
moo_opts.var_names = {'Lx','Lyi','Lyo','T_max'};
moo_opts.lb = [1.0, 1.0, 2.5, 8000];
moo_opts.ub = [5.0, 5.0, 9.0, 16000];
moo_opts.eval_mode = 'acs';
moo_opts.pop_size = 100;
moo_opts.max_gen = 150;
[pareto_designs, pareto_J] = run_moo(d0, moo_opts);
```

## How To Read Current Results

Reasonable Stage 1 trends:

- `FII` decreases as geometry becomes more balanced for fault tolerance
- `hover_ok` improves as `T_max` increases past the threshold
- Pareto knee should usually have:
  larger geometry than the SOO optimum,
  higher cost/mass than SOO,
  slightly better `FII`

Important interpretation rule:

- `acs`-only `J_combined` values are the valid Stage 1 optimization scores
- `full`-mode `J_combined` values are not Stage 1 scores, because they also
  include the mission penalty

So if Section 5 and the final summary show different `J_combined` scales,
that is expected.

## Current Status

What is working:

- Stage 1 optimization behavior is plausible
- ACS metrics are consistent
- single-fault hover screening is now enforced in Stage 1 feasibility
- parameterized mass/inertia/cost model is wired into evaluation

What is not ready:

- mission/path-following validation
- dynamic proof that Stage 1 candidates can actually recover in closed-loop
- degraded-fault and multifault optimization

## Known Issues / Limitations

These should be shared clearly with teammates.

### 1. Stage 1 is not mission feasibility

A Stage 1 design can be:

- ACS-feasible
- hover-feasible under single fault

and still fail the current simulation.

So Stage 1 should be described as:

- `fault-tolerant hover/ACS screening`

not:

- `full dynamic feasibility`

### 2. Current simulation still diverges for tested designs

At the moment, Stage 2 / `full` validation is still failing for baseline,
SOO-optimal, and Pareto-knee designs in the tested motor-3 fault case.

That means:

- Stage 1 is useful for screening
- Stage 2 still needs controller / mission-layer work

### 3. PFWAR is not a useful main design lever here

It remains close to `0.3333` for this configuration.

For discussion and reporting, focus more on:

- `FII`
- `hover_margin`
- `hover_ok`
- `J_cost`

### 4. Current cost model is still a compact surrogate

It is better than the old fixed-mass proxy, but it is still not a full
structural or propulsion design model.

It should be presented as a parameterized surrogate model.

## Recommended Team Message

Use wording close to this:

> Stage 1 is now a working optimization/screening layer for parameterized
> hexacopter design under single-motor fault conditions. It evaluates
> fault-tolerance balance, single-fault hover survivability, and a
> parameterized mass/cost surrogate. The current optimized designs are
> reasonable for ACS/hover screening, but they are not yet mission-validated
> because the present closed-loop simulation still diverges in the tested
> fault case.

## Immediate Next Step

After sharing Stage 1 with teammates, the next technical task should be:

- consolidate Stage 2 mission validation
- integrate path-following evaluation into the framework
- improve control/allocation logic enough to test whether Stage 1 candidates
  are practically recoverable
