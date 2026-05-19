# FrameworkCpp Phase 2: Powertrain And Battery

This document covers the Phase 2 extensions only:

- powertrain electrical hover-power estimation
- battery energy sizing
- battery-related hard constraints
- Phase 2 metrics exposed in `Stage1Metrics`

For the full optimization problem, see:

- `docs/Optimization.md`
- `docs/Optimization_Constraints_And_Design_Variables.md`

## Scope in code

Main implementation files:

- [PowertrainEvaluator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/physics/PowertrainEvaluator.cpp:1)
- [BatteryEvaluator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/physics/BatteryEvaluator.cpp:1)
- [Stage1Evaluator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/evaluation/Stage1Evaluator.cpp:1)
- [EvaluationContext.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/evaluation/EvaluationContext.hpp:1)

## Battery design variable

Phase 2 adds one active design variable:

| Name | Unit | Default | Bounds |
|---|---:|---:|---:|
| `m_bat` | kg | 400.0 | [100.0, 1000.0] |

`BatteryElement` now contributes real mass through:

```text
mass_ = m_bat
```

That means battery sizing feeds back into:

- total mass
- hover thrust requirements
- hover power
- mission energy demand
- C-rate

## Powertrain model

`PowertrainEvaluator` uses the nominal trim thrust vector and the worst-fault trim thrust vector.

### Effective disk area

The effective rotor disk is derived from arm geometry:

```text
r_eff = max(max_arm_length * 0.5, 0.1)
A_eff = pi * r_eff^2
```

This is intentionally not driven directly by `d_prop`. In the current code, `d_prop` remains relevant to rotor geometry and yaw-torque scaling, while the power model uses a geometry-derived effective disk size.

### Electrical power model

Combined efficiency:

```text
eta_total = figure_of_merit * motor_efficiency * esc_efficiency
denom = eta_total * sqrt(2 * air_density * A_eff)
```

Per-rotor electrical power:

```text
P_i = T_i^(3/2) / denom
```

Total powers:

```text
P_nominal_total = sum(P_i from nominal trim)
P_faulted_total = sum(P_i from worst-fault trim)
```

### Continuous-power reference

The per-rotor continuous-power reference is electrical power at `T_max`:

```text
P_cont = T_max^(3/2) / denom
```

Nominal utilization metrics:

```text
worst_thrust_utilization = max(T_i / T_max)
worst_power_utilization = max(P_i / P_cont)
```

## Battery model

`BatteryEvaluator` uses the powertrain result plus `m_bat`.

### Available energy

```text
E_avail = battery_pack_efficiency
        * battery_dod_usable
        * m_bat
        * battery_specific_energy_wh_per_kg
```

Default context values:

| Field | Default |
|---|---:|
| `battery_specific_energy_wh_per_kg` | 250.0 |
| `battery_dod_usable` | 0.85 |
| `battery_pack_efficiency` | 0.95 |

### Mission energy demand

Auxiliary power is included in both phases:

```text
P_nom_total = total_power_nominal_w + power_auxiliary_w
P_fault_total = total_power_faulted_w + power_auxiliary_w
```

Mission energies:

```text
E_req_nominal = P_nom_total * (mission_time_nominal_min / 60)
E_req_fault   = P_fault_total * (mission_time_emergency_min / 60)
E_req_total   = E_req_nominal + E_req_fault
```

Default context values:

| Field | Default |
|---|---:|
| `mission_time_nominal_min` | 6.0 |
| `mission_time_emergency_min` | 1.0 |
| `power_auxiliary_w` | 500.0 |

### Reserve fraction

```text
energy_reserve_fraction = (E_avail - E_req_total) / E_avail
```

Interpretation:

- `>= 0`: feasible on energy
- `< 0`: deficit

### C-rate

Peak power uses the larger of nominal and worst-fault hover power:

```text
P_peak = max(total_power_nominal_w, total_power_faulted_w) + power_auxiliary_w
c_rate = P_peak / E_avail
```

Voltage is not needed in the final formula because it cancels.

## Phase 2 hard constraints

These constraints are registered in `HexacopterArchitecture::registerDefaultConstraints()` except where noted.

| Stable ID | Sense | Condition | Penalty |
|---|---|---|---:|
| `battery_energy_reserve` | `>= 0` | `bat_energy_reserve_fraction >= 0` | 2000 |
| `battery_crate_limit` | `<= 0` | `(bat_c_rate / battery_crate_limit) - 1 <= 0` | 1500 |
| `battery_mass_positive` | `>= 0` | `m_bat >= 0` | 500 |

Default battery C-rate limit from `EvaluationContext`:

```text
battery_crate_limit = 5.0
```

## Phase 2 metrics in `Stage1Metrics`

### Powertrain

| Field | Meaning |
|---|---|
| `pt_total_power_nominal_w` | total electrical power at nominal hover |
| `pt_total_power_faulted_w` | total electrical power at worst-fault hover |
| `pt_worst_thrust_utilization` | maximum nominal `T_i / T_max` |
| `pt_worst_power_utilization` | maximum nominal `P_i / P_cont` |

### Battery

| Field | Meaning |
|---|---|
| `bat_available_energy_wh` | available battery energy |
| `bat_required_energy_wh` | nominal plus emergency mission energy demand |
| `bat_energy_reserve_fraction` | signed reserve fraction |
| `bat_c_rate` | peak discharge rate in `1/h` |
| `bat_mass_fraction` | `m_bat / m_total` |

## Evaluation order

The current evaluation order in `Stage1Evaluator` is:

1. `VehicleScalingModel`
2. `StructuralAnalyzer`
3. `AttainableControlSetAnalyzer`
4. identify the worst-fault case by highest total faulted thrust demand
5. `PowertrainEvaluator`
6. `BatteryEvaluator`
7. copy Phase 2 outputs into `Stage1Metrics`
8. evaluate hard constraints
9. aggregate objectives

This order matters because the battery constraints read values already written into `Stage1Metrics`.

## Practical interpretation

Phase 2 creates the main energy tradeoff in the current optimizer:

- increasing `m_bat` helps the energy reserve and C-rate constraints
- increasing `m_bat` hurts total mass
- increasing `m_bat` also increases hover thrust and therefore power demand

So the optimizer is not simply "make the battery larger". It searches for the smallest battery mass that remains compatible with the geometry, controllability, structural, and mission-energy constraints.
