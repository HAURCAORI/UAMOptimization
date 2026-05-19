# Phase 2 — Powertrain Limits and Battery Sizing

## Overview

Phase 2 adds powertrain power analysis (D) and battery energy sizing (E) per the
`ImplementationAdd.md` spec. It introduces one new design variable (`m_bat`), two new hard
constraints (`battery_energy_reserve`, `battery_crate_limit`), two new evaluator classes
(`PowertrainEvaluator`, `BatteryEvaluator`), and a set of Phase 2 metrics in `Stage1Metrics`.

---

## New Design Parameter

| Stable ID | Unit | Default | Bounds | Active |
|---|---|---|---|---|
| `m_bat` | kg | 400 | [100, 1000] | **true** |

`m_bat` is the battery pack mass. It contributes directly to total vehicle mass via `BatteryElement`
(which now sets `mass_ = m_bat->value`), creating a feedback loop: heavier battery → more hover
thrust needed → more power → but also more stored energy.

**Calibration**: At baseline geometry (`Lyo = 5.5 m`, `T_max = 8000 N`, `m_payload = 800 kg`,
`m_bat = 400 kg`): total mass ≈ 1850 kg, nominal hover power ≈ 280 kW, E_avail ≈ 80.8 kWh
(at 250 Wh/kg), E_req ≈ 32 kWh → reserve ≈ 60%. C_rate ≈ 3.5 h⁻¹ < 5.0 h⁻¹ limit.
The battery energy constraint becomes infeasible below approximately `m_bat ≈ 150 kg`.

---

## Power Model

### Effective Disk Area

The power model uses **arm geometry** for the effective rotor disk, NOT `d_prop`.

`d_prop = 0.40 m` is frozen for the cT (yaw-torque) computation only (Phase 1 MATLAB alignment
fix). Using it for power would give physically unrealistic results at the vehicle mass scale.
Instead:

```
r_eff = max_arm_length / 2   [m]
A_eff = π × r_eff²           [m²]   (per rotor)
```

At baseline (`Lyo = 5.5 m`): `r_eff = 2.75 m`, `A_eff ≈ 23.76 m²` per rotor.

### Actuator Disk Formula

Per-motor ideal (mechanical) power from momentum theory:
```
P_ideal_i = T_i^{3/2} / sqrt(2 × ρ × A_eff)   [W]
```

Electrical power including figure of merit and drivetrain losses:
```
P_elec_i = P_ideal_i / (η_FM × η_mot × η_ESC)  [W]
```

Default efficiency factors (all overridable in `EvaluationContext`):

| Parameter | Symbol | Default |
|---|---|---|
| `figure_of_merit` | η_FM | 0.65 |
| `motor_efficiency` | η_mot | 0.85 |
| `esc_efficiency` | η_ESC | 0.95 |
| `air_density` | ρ | 1.225 kg/m³ |

### Continuous Power Limit

Defined as electrical power at T_max (same motor, full thrust):
```
P_cont = T_max^{3/2} / denom
```

Thrust and power utilization ratios:
```
u_T = T_i / T_max               (worst-case over all motors at nominal hover)
u_P = P_i / P_cont = u_T^{3/2}  (proportional to thrust utilization cubed)
```

### Worst-Fault Motor Selection

The powertrain evaluator uses the **highest total-thrust fault case** as the worst fault. This
represents the fault requiring the most energy redistribution and corresponds to the highest fault
power draw.

---

## Battery Model

### Available Energy

```
E_avail = η_pack × DoD × m_bat × e_spec   [Wh]
```

| Parameter | Default | Description |
|---|---|---|
| `battery_specific_energy_wh_per_kg` | 250.0 | NMC811 cell-level specific energy, 2025-era [Wh/kg] |
| `battery_dod_usable` | 0.85 | Usable depth of discharge |
| `battery_pack_efficiency` | 0.95 | Pack-level energy efficiency |

### Energy Requirement

The mission has two phases, each with auxiliary power included:

```
P_nom_total = P_elec_nom + P_aux    [W]
P_fault_total = P_elec_fault + P_aux [W]

E_req_nom   = P_nom_total   × (t_nom_min / 60)   [Wh]
E_req_fault = P_fault_total × (t_emg_min / 60)   [Wh]
E_req_total = E_req_nom + E_req_fault             [Wh]
```

Default mission parameters:

| Parameter | Default | Description |
|---|---|---|
| `mission_time_nominal_min` | 6.0 | Required nominal hover time [min] |
| `mission_time_emergency_min` | 1.0 | Required fault-hover endurance [min] |
| `power_auxiliary_w` | 500.0 | Avionics + payload draw [W] |

### Energy Reserve Fraction

```
reserve_fraction = (E_avail - E_req_total) / E_avail   ∈ (-∞, 1]
```

Feasible when `reserve_fraction ≥ 0` (hard constraint `battery_energy_reserve`).

### C-Rate

Voltage cancels in the C-rate formula:
```
C_rate = I_peak / Q_pack
       = (P_peak / V_nom) / (E_avail / V_nom)
       = P_peak [W] / E_avail [Wh]   [h⁻¹]
```

Where `P_peak = max(P_nom_total, P_fault_total)`. Hard constraint `battery_crate_limit` enforces
`C_rate ≤ C_allow` (default 5.0 h⁻¹ — peak burst during hover phases).

---

## New Hard Constraints

| Stable ID | Sense | Threshold | Penalty | Registered in |
|---|---|---|---|---|
| `battery_energy_reserve` | ≥ | 0.0 | 2000 | `HexacopterArchitecture::registerDefaultConstraints()` |
| `battery_crate_limit` | ≤ | 0.0 | 1500 | `HexacopterArchitecture::registerDefaultConstraints()` |
| `battery_mass_positive` | ≥ | 0.0 | 500 | `BatteryElement::registerConstraints()` |

---

## New Phase 2 Metrics (`Stage1Metrics`)

### Powertrain

| Field | Unit | Description |
|---|---|---|
| `pt_total_power_nominal_w` | W | Total electrical power, nominal hover |
| `pt_total_power_faulted_w` | W | Total electrical power, worst-fault hover |
| `pt_worst_thrust_utilization` | — | max(T_i/T_max) at nominal hover ∈ [0,1] |
| `pt_worst_power_utilization` | — | max(P_i/P_cont) = u_T^{3/2} ∈ [0,1] |

### Battery

| Field | Unit | Description |
|---|---|---|
| `bat_available_energy_wh` | Wh | E_avail = η_pack × DoD × m_bat × e_spec |
| `bat_required_energy_wh` | Wh | E_req_total = E_req_nom + E_req_emg |
| `bat_energy_reserve_fraction` | — | (E_avail − E_req) / E_avail; ≥0 = feasible |
| `bat_c_rate` | 1/h | P_peak / E_avail; must be ≤ C_allow = 3.0 |
| `bat_mass_fraction` | — | m_bat / m_total ∈ [0,1] |

---

## Evaluation Order in `Stage1Evaluator`

```
1. VehicleScalingModel          → PhysicalModel (mass now includes m_bat from BatteryElement)
2. StructuralAnalyzer           → arm_structural, min_safety_factor
3. AttainableControlSetAnalyzer → AcsResult (trim thrusts for power model)
4. PowertrainEvaluator          → PowertrainResult (motor power W, utilizations)
5. BatteryEvaluator             → BatteryResult (energy Wh, reserve fraction, C-rate)
6. Stage1Metrics population     → all Phase 2 fields set before constraint evaluation
7. ConstraintRegistry           → reads bat_energy_reserve_fraction, bat_c_rate from Stage1Metrics
8. acs::all_faults_hover_feasible (appended directly)
9. ObjectiveAggregator          → combined objective (mass already captures battery mass effect)
```

---

## Design Trade-offs

The battery creates two opposing pressures:

| Pressure | Direction | Mechanism |
|---|---|---|
| `battery_energy_reserve` (hard) | ↑ m_bat | infeasible below ~290 kg at baseline |
| `mass` objective (soft, w=0.20) | ↓ m_bat | heavier battery penalizes mass objective |
| `power` objective (soft, w=0.20) | ↓ m_bat | more mass → more hover thrust → more power |

The optimizer resolves this by finding the minimum m_bat that keeps the energy constraint feasible,
subject to the geometry and arm configuration chosen by the other variables.

---

## Topology Constraint: Motor 3/4 Fault Feasibility

Motors 3 and 4 sit on the pure-roll axis (y-axis, zero pitch arm). When either fails, solving the
4-DOF hover trim LP analytically yields **T1 = T6 = mg/2** exactly — independent of arm lengths.

**Proof (motor 3 fails, remaining: 1, 2, 4, 5, 6):**
- Pitch balance → T1+T2 = T5+T6 = p
- Yaw balance → T4 = p + T5 - T6
- Roll balance → 5.3·T2 + 2.85·T4 = 0 → T2 = T4 = 0 (both ≥ 0)
- Thrust balance → T1 + T6 = mg → T1 = T6 = mg/2

**Feasibility condition:** `T_max ≥ mg/2`, equivalently `6·T_max/mg ≥ 3.0`.

At `m_payload = 800 kg`, `m_bat = 400 kg`, `structural ≈ 650 kg`: `mg/2 ≈ 9.1 kN`. This is
achievable at `T_max ≥ 9.5 kN` (well within the [8, 20] kN search range).

At the old `m_payload = 1500 kg`: `mg/2 ≈ 13.1 kN` — barely feasible only at near-maximum T_max
and minimum battery, leaving almost no optimizer bandwidth.

---

## Physical Note on Power Scale

At baseline geometry (Lyo = 5.5 m, T_max = 8000 N):
- Hover power per motor at nominal: ~35–42 kW depending on m_bat choice
- Total electrical hover power: ~210–250 kW (6 motors)
- For 7-min mission: ~24–30 kWh required
- At 200 Wh/kg with DoD=0.85, η_pack=0.95: m_bat ≥ ~147 kg per energy constraint

These numbers assume disk area from arm geometry. Using d_prop=0.40 m would give >100× higher
power (physically impossible disk loading at this mass scale). See `CLAUDE.md` for why d_prop is
frozen and how to interpret power metrics.

---

## Pending (Phase 5)

- `MetricRole` enum: tag each Phase 2 metric as `hard_constraint`, `soft_objective`, or
  `analysis_only`.
- Phase 2 soft objectives: `powertrain_util` and `battery_energy` could be added as weighted
  objectives when the `MetricRole` system is in place.
