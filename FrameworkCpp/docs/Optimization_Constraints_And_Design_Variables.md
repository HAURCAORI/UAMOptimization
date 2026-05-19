# FrameworkCpp Optimization Constraints And Design Variables

This file is the detailed optimization-variable and hard-constraint reference for `FrameworkCpp`, based on the live code in `src/core`, `src/evaluation`, and `src/optimization`.

Use it together with:

- `docs/Optimization.md` for the overall evaluation and optimizer flow
- `docs/Phase2_Powertrain_Battery.md` for Phase 2 power and battery formulas

## Scope

- Design variables are defined in `HexacopterArchitecture::registerDefaultParameters()`.
- Hard constraints are defined partly in `HexacopterArchitecture::registerDefaultConstraints()` and partly by individual elements in `Elements.cpp`.
- The optimizer only sees active parameters.
- Feasibility is determined by nominal hover trim, all-fault hover requirements, and all active hard constraints.

## Optimization Design Variables

These parameters are active by default, so they are included in the pagmo design vector.

| Name | Unit | Default | Bounds | Role |
|---|---:|---:|---:|---|
| `Lx` | m | 2.65 | [2.0, 5.0] | Fore/aft arm length |
| `Lyi` | m | 2.65 | [2.0, 5.0] | Inner lateral arm length |
| `Lyo` | m | 5.50 | [2.5, 9.0] | Outer lateral arm length |
| `T_max` | N | 12000.0 | [8000.0, 20000.0] | Maximum thrust per motor |
| `arm_outer_radius` | m | 0.08 | [0.02, 0.15] | Arm tube outer radius |
| `arm_wall_thickness` | m | 0.005 | [0.001, 0.020] | Arm tube wall thickness |
| `m_bat` | kg | 400.0 | [100.0, 1000.0] | Battery pack mass |

## Fixed But Registered Parameters

These parameters exist in the architecture and affect evaluation, but they are inactive by default and therefore not optimized unless code/config changes activate them.

| Name | Unit | Default | Bounds | Role |
|---|---:|---:|---:|---|
| `cT` | - | 0.03 | [0.01, 0.08] | Yaw moment to thrust ratio |
| `d_prop` | m | 0.40 | [0.20, 1.20] | Propeller diameter |
| `m_payload` | kg | 800.0 | [400.0, 2000.0] | Payload mass |

## Design Vector Mapping

For each active parameter:

```text
normalized = ((value - lower_bound) / (upper_bound - lower_bound)) * scale
value = lower_bound + (upper_bound - lower_bound) * (normalized / scale)
```

Current parameters use `scale = 1.0`, so pagmo sees a normalized search box of `[0, 1]` per active variable.

The optimizer stores and exports physical bounds separately:

- `OptimizationProblem.lower_bounds[i] = parameter.lower_bound`
- `OptimizationProblem.upper_bounds[i] = parameter.upper_bound`

## Constraint Structure

All listed constraints below are hard constraints with penalties applied through `PagmoProblemAdapter::constraintPenalty()`.

If a constraint is registered once per element, the same stable ID appears multiple times with different owners.

## System-Level Hard Constraints

| Stable ID | Sense | Threshold | Penalty | Meaning |
|---|---|---:|---:|---|
| `parameter_bounds` | `<=` | 0.0 | 1000 | Maximum parameter over/under-bound violation must be zero |
| `minimum_geometry_margin` | `>=` | 0.0 | 1000 | Enforces `min(Lx - min_arm_length, Lyi - min_arm_length, Lyo - (Lyi + min_outer_arm_delta)) >= 0` |
| `rotor_clearance` | `>=` | 0.0 | 1000 | Minimum rotor-to-rotor geometric clearance must be nonnegative |
| `failed_hover_gamma` | `>=` | 1.5 | 2000 | Worst single-fault thrust margin must exceed `gamma_thrust_required` |
| `fault_allocation_ratio` | `>=` | 0.05 | 1500 | `sigma_worst / sigma_reference >= minimum_fault_allocation_ratio` |
| `arm_yield_failure` | `>=` | 1.5 | 2000 | Minimum structural safety factor across arms must exceed `minimum_arm_safety_factor` |
| `battery_energy_reserve` | `>=` | 0.0 | 2000 | Battery available energy must exceed nominal plus emergency mission demand |
| `battery_crate_limit` | `<=` | 0.0 | 1500 | Enforces `(bat_c_rate / battery_crate_limit) - 1 <= 0` |
| `all_faults_hover_feasible` | `>=` | 0.0 | 20000 | ACS-based worst-fault hover margin must be nonnegative when enabled |
| `fault_directional_margin` | `>=` | 0.0 | 5000 | ACS worst-fault directional margin must exceed `eps_acs_fault_margin` |

## Element-Level Hard Constraints

### Body

| Stable ID | Sense | Threshold | Penalty | Meaning |
|---|---|---:|---:|---|
| `body_span_order` | `>=` | 0.0 | 1000 | `min(Lx, Lyi, Lyo - Lyi) >= 0` |

### Payload

| Stable ID | Sense | Threshold | Penalty | Meaning |
|---|---|---:|---:|---|
| `payload_mass_nonnegative` | `>=` | 0.0 | 1000 | `m_payload >= 0` |

### Battery

| Stable ID | Sense | Threshold | Penalty | Meaning |
|---|---|---:|---:|---|
| `battery_mass_positive` | `>=` | 0.0 | 500 | `m_bat >= 0` |

### Arms

Registered once per arm.

| Stable ID | Sense | Threshold | Penalty | Meaning |
|---|---|---:|---:|---|
| `arm_length_positive` | `>=` | 0.0 | 750 | Each arm length must be nonnegative |

Arm length depends on rotor index:

```text
outer lateral arms (indices 2, 3): L_arm = Lyo
diagonal arms (indices 0, 1, 4, 5): L_arm = sqrt(Lx^2 + Lyi^2)
```

### Motors

Registered once per motor.

| Stable ID | Sense | Threshold | Penalty | Meaning |
|---|---|---:|---:|---|
| `motor_thrust_positive` | `>=` | 0.0 | 1000 | `T_max >= 0` |

### Rotors

Registered once per rotor.

| Stable ID | Sense | Threshold | Penalty | Meaning |
|---|---|---:|---:|---|
| `rotor_diameter_positive` | `>=` | 0.0 | 750 | `d_prop >= 0` |

## Constraint Inputs And Evaluation Context

The most important configuration values used by constraints are in `EvaluationContext`:

| Context field | Default | Used by |
|---|---:|---|
| `gamma_thrust_required` | 1.5 | `failed_hover_gamma`, `fault_thrust` objective |
| `minimum_fault_allocation_ratio` | 0.05 | `fault_allocation_ratio` |
| `minimum_arm_length` | 0.5 m | `minimum_geometry_margin` |
| `minimum_outer_arm_delta` | 0.1 m | `minimum_geometry_margin` |
| `minimum_arm_safety_factor` | 1.5 | `arm_yield_failure`, `structural_safety` objective |
| `require_all_fault_acs_feasible` | `true` | `all_faults_hover_feasible` |
| `eps_acs_fault_margin` | 50.0 | `fault_directional_margin` |
| `battery_crate_limit` | 5.0 | `battery_crate_limit` |
| `mission_time_nominal_min` | 6.0 min | `battery_energy_reserve` |
| `mission_time_emergency_min` | 1.0 min | `battery_energy_reserve` |
| `battery_specific_energy_wh_per_kg` | 250.0 | battery energy model |
| `battery_dod_usable` | 0.85 | battery energy model |
| `battery_pack_efficiency` | 0.95 | battery energy model |
| `power_auxiliary_w` | 500.0 W | battery energy and C-rate model |

## Feasibility Rule

A candidate is marked feasible only if all of the following hold:

1. Nominal hover trim is feasible.
2. All-fault hover trim is feasible when `require_all_fault_acs_feasible == true`.
3. Every active hard constraint evaluates feasible.

If not feasible, the pagmo adapter applies:

```text
objective = 1e6 + summed_constraint_penalty + extra_infeasibility_penalty
```

with an additional `1e4` added whenever `result.feasible == false`, plus each violated constraint's own weighted penalty contribution.

## Objectives Relevant To Optimization

The question was about variables and constraints, but the constraint set is easier to interpret with the active objective set:

| Objective | Default weight | Notes |
|---|---:|---|
| `mass` | 0.20 | Total vehicle mass normalized by reference mass |
| `power` | 0.20 | Hover power proxy normalized by baseline |
| `fault_thrust` | 0.25 | Squared shortfall below required fault thrust margin |
| `fault_alloc` | 0.25 | Baseline-to-current scaled control effectiveness ratio |
| `hover_nom` | 0.10 | Nominal hover thrust utilization proxy |
| `acs_margin` | 0.10 | Penalty based on negative ACS overall margin |
| `structural` | 0.0 | Computed but inactive by default |
| `packaging` | 0.0 | Computed but inactive by default |
| `structural_safety` | 0.0 | Computed but inactive by default |

For MOO runs, `main.cpp` currently uses:

```text
mass, power, fault_alloc
```

## Practical Reading

In the current codebase, the optimization problem is effectively:

- Geometry sizing: `Lx`, `Lyi`, `Lyo`
- Propulsion sizing: `T_max`
- Structural sizing: `arm_outer_radius`, `arm_wall_thickness`
- Energy sizing: `m_bat`

subject to:

- geometry and clearance constraints,
- fault-hover and controllability constraints,
- structural arm safety constraints,
- battery energy reserve and C-rate constraints,
- and basic nonnegativity / bound consistency constraints.
