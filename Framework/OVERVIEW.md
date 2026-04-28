# Framework Overview

## 1. Current State

The framework has two connected layers:

- Stage 1: design optimization
- Stage 2: mission validation

Stage 1 is the main optimization layer. Stage 2 is the secondary validation
layer.

## 2. Interface Design

The code now uses one public option model:

- `cfg = mdo_config()` for framework configuration
- `opt = UAMOptions(cfg, ...)` for call-level behavior

The public evaluation and analysis helpers use `UAMOptions` consistently:

- `eval_design`
- `compare_designs`
- `sweep_design_space`
- `pareto_analysis`

The optimization drivers remain cfg-driven:

- `run_soo`
- `run_moo`

## 3. Implemented Capability

### Stage 1

- ACS-based fault-tolerance evaluation
- single-motor-fault hover screening
- parameterized mass and inertia model
- SOO workflow
- MOO workflow
- Pareto analysis and design comparison

### Stage 2

- hover simulation path
- figure-8 mission path
- normalized simulation config handling
- analytically scaled gains
- mission verification block in `main_mdo.m`

## 4. Current Variables

Active by default:

- `Lx`
- `Lyi`
- `Lyo`
- `T_max`

Reserved but frozen:

- `d_prop`

## 5. Current Objectives

### Stage 1

- `mass`
- `power`
- `fault_thrust`
- `fault_alloc`
- `hover_nom`

### Stage 2

- `mission`

Stage 2 objectives are currently used for validation and comparison, not for
the main optimization loop.

## 6. Recommended Interpretation

Safe current statement:

- the framework can optimize Stage 1 designs and then validate selected
  designs in Stage 2

Unsafe overstatement:

- the framework is already a full mission-coupled optimization loop

## 7. Near-Term Direction

1. keep Stage 1 as the main optimization layer
2. use Stage 2 to validate representative candidate designs
3. decide final variables and objectives with the team
4. extend the framework only where the final formulation requires it
