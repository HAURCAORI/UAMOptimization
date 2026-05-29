# FrameworkCpp Optimization Constraints And Design Variables

Live reference derived from `src/core/HexacopterArchitecture.cpp`, `src/core/Elements.cpp`,
and `include/evaluation/EvaluationContext.hpp`. See `docs/Optimization.md` for the overall
evaluation flow and `docs/Phase2_Powertrain_Battery.md` for power/battery formula details.

---

## Design variables (active parameters — 16 total)

All parameters use `scale = 1.0`; pagmo sees bounds `[0, 1]` per variable.

### Geometry and propulsion (4)

| ID | Unit | Default | Bounds | Role |
|---|---|---|---|---|
| `hexa::Lx`    | m | 2.65  | [2.0, 5.0]      | Fore/aft arm length (motors 0,1,4,5) |
| `hexa::Lyi`   | m | 2.65  | [2.0, 5.0]      | Inner lateral arm length |
| `hexa::Lyo`   | m | 5.50  | [2.5, 9.0]      | Outer lateral arm length (motors 2,3) |
| `hexa::T_max` | N | 12000 | [8000, 20000]   | Maximum thrust per motor |

Lower bounds on Lx, Lyi ≥ 2.0 m prevent arm_1256 = √(Lx²+Lyi²) from collapsing to
1.4 m while Lyo→9 m (max arm-span ratio ≤ 3.2×). T_max ≥ 8000 N keeps motor-2-fault
hover trim feasible at minimum geometry.

### Structural cross-section (2)

| ID | Unit | Default | Bounds | Role |
|---|---|---|---|---|
| `hexa::arm_outer_radius`   | m | 0.08  | [0.02, 0.15]   | Arm tube outer radius |
| `hexa::arm_wall_thickness` | m | 0.005 | [0.001, 0.020] | Arm tube wall thickness |

At default geometry, baseline tip deflection ≈ 1.27 m (1270% over the 0.10 m limit).
The optimizer must raise t_wall or reduce arm length to satisfy `arm_tip_deflection`.

### Battery (1)

| ID | Unit | Default | Bounds | Role |
|---|---|---|---|---|
| `hexa::m_bat` | kg | 400 | [100, 1000] | Battery pack mass |

400 kg baseline is severely undersized for the 30-minute mission (≈ 80 kWh available vs
≈ 140 kWh required at baseline power). Optimizer converges toward 650–900 kg.

### Passenger placement (3)

| ID | Unit | Default | Bounds | Notes |
|---|---|---|---|---|
| `hexa::pax_x` | m | 0.00 | [−0.25, +0.25] | Fore/aft (forward = +x) |
| `hexa::pax_y` | m | 0.00 | [−0.42, +0.42] | Lateral (right = +y) |
| `hexa::pax_z` | m | −0.10 | [−0.50, +0.40] | Vertical (NED: −z = toward ceiling) |

Default pax_z = −0.10 m places the 4-seat group center 0.10 m above the hub plane.
Group z-extent at default: [−0.70, +0.50] m (ceiling → mid-cabin). Symmetric in x and y.

### Cargo placement (3)

| ID | Unit | Default | Bounds | Notes |
|---|---|---|---|---|
| `hexa::cargo_x` | m | 0.00 | [−0.50, +0.50] | |
| `hexa::cargo_y` | m | 0.00 | [−0.45, +0.45] | |
| `hexa::cargo_z` | m | +0.75 | [+0.30, +1.00] | +z = below hub (floor zone) |

Default cargo_z = +0.75 m places the cargo slab in the cabin floor zone, 0.07 m below
the passenger group bottom (z = +0.50). Stays within the cabin (half-z = 1.10 m).

### Battery placement (3)

| ID | Unit | Default | Bounds | Notes |
|---|---|---|---|---|
| `hexa::bat_x` | m | 0.00 | [−0.50, +0.50] | Symmetric |
| `hexa::bat_y` | m | 0.00 | [−0.55, +0.55] | Symmetric |
| `hexa::bat_z` | m | −0.90 | [−1.00, +0.85] | −z = ceiling zone |

Lower bound −1.00 keeps slim slab (half_z ≤ 0.08 m) inside cabin ceiling at −1.10 m.
The `packaging::battery_in_cabin` constraint enforces this continuously.

---

## Fixed (inactive) parameters

| ID | Unit | Default | Reason |
|---|---|---|---|
| `hexa::cT`           | —  | 0.03 | MATLAB reference; yaw-torque cT only |
| `hexa::d_prop`       | m  | 0.40 | Visual/cT scaling; powertrain uses r_eff |
| `hexa::m_pax`        | kg | 600  | Fixed UAM mission requirement |
| `hexa::m_cargo`      | kg | 200  | Fixed UAM mission requirement |
| `hexa::m_instrument` | kg | 50   | Fixed avionics mass |

Total fixed payload mass: 600 + 200 + 50 = **850 kg**.

---

## Hard constraints

### Architecture-level (`registerDefaultConstraints`)

| Stable ID | Sense | Threshold | Penalty | Expression |
|---|---|---|---|---|
| `parameter_bounds` | ≤ | 0 | 1 000 | max over-bound violation across all params |
| `minimum_geometry_margin` | ≥ | 0 | 1 000 | min(Lx − 0.5, Lyi − 0.5, Lyo − Lyi − 0.1) |
| `rotor_clearance` | ≥ | 0 | 1 000 | min inter-rotor disk clearance [m] |
| `packaging::payload_in_cabin` | ≤ | 0 | 1 000 | max containment violation: pax/cargo/instrument in cabin |
| `packaging::battery_in_cabin` | ≤ | 0 | 1 000 | battery containment violation in cabin |
| `packaging::battery_payload_nonoverlap` | ≤ | 0 | 1 000 | max(bat∩pax, bat∩cargo, bat∩instr) overlap |
| `packaging::occupant_in_cabin` | ≤ | 0 | 1 000 | occupant envelope in cabin |
| `packaging::payload_components_nonoverlap` | ≤ | 0 | 1 000 | max(pax∩cargo, pax∩instr, cargo∩instr) overlap |
| `cg_envelope` | ≤ | 0 | 800 | max(\|CG_x\| − 0.40, \|CG_y\| − 0.25) [m] |
| `failed_hover_gamma` | ≥ | 1.5 | 2 000 | γ_worst = 5·T_max / (m·g) |
| `fault_allocation_ratio` | ≥ | 0.05 | 1 500 | σ_worst / σ_ref |
| `arm_yield_failure` | ≥ | 1.5 | 2 000 | struct_net_min_safety_factor (Phase 3 network) |
| `battery_energy_reserve` | ≥ | 0 | 2 000 | (E_avail − E_req) / E_avail |
| `battery_crate_limit` | ≤ | 0 | 1 500 | bat_c_rate / 5.0 − 1 |
| `arm_tip_deflection` | ≤ | 0 | 1 500 | (δ_max / 0.10 m) − 1 |
| `arm_tip_rotation` | ≤ | 0 | 1 500 | (θ_max / 0.10 rad) − 1 |
| `all_faults_hover_feasible` | ≥ | 0 | 20 000 | acs_hover_margin = T_max / T_hover_worst − 1 |
| `fault_directional_margin` | ≥ | 0 | 5 000 | (acs_worst_fault_min_margin − 50) / 50 |

Notes:
- `cg_envelope` checks **both** fore/aft (\|CG_x\| ≤ 0.40 m) and lateral (\|CG_y\| ≤ 0.25 m).
- `arm_yield_failure` reads from `StructuralNetworkAnalyzer` (Phase 3), not the legacy `StructuralAnalyzer`.
- `all_faults_hover_feasible` has the highest penalty (20 000) — LP hover feasibility is the primary gate.
- `fault_directional_margin` penalty 5 000 < 20 000 so LP feasibility takes priority in the search.
- Packaging constraints use the world-frame AABB of each element's `localEnvelope()`, transformed by
  the assembled `world_pose`. Elements own their placement via `local_pose_` (no attachment lambda).

### Element-level constraints

| Stable ID | Owner | Sense | Threshold | Penalty | Expression |
|---|---|---|---|---|---|
| `body_span_order` | body | ≥ | 0 | 1 000 | min(Lx, Lyi, Lyo − Lyi) |
| `battery_mass_positive` | battery | ≥ | 0 | 500 | m_bat |
| `arm_length_positive` | arm_i (×6) | ≥ | 0 | 750 | L_arm(i) |
| `motor_thrust_positive` | motor_i (×6) | ≥ | 0 | 1 000 | T_max |
| `rotor_diameter_positive` | rotor_i (×6) | ≥ | 0 | 750 | d_prop |

Arm lengths: diagonal arms (0,1,4,5) L = √(Lx²+Lyi²); outer lateral arms (2,3) L = Lyo.

---

## EvaluationContext configuration

All constraint thresholds and model parameters live in `EvaluationContext`. Key defaults:

| Field | Default | Consumed by |
|---|---|---|
| `gamma_thrust_required` | 1.5 | `failed_hover_gamma`, `fault_thrust` objective |
| `minimum_fault_allocation_ratio` | 0.05 | `fault_allocation_ratio` |
| `minimum_arm_length` | 0.5 m | `minimum_geometry_margin` |
| `minimum_outer_arm_delta` | 0.1 m | `minimum_geometry_margin` |
| `minimum_arm_safety_factor` | 1.5 | `arm_yield_failure` |
| `arm_tip_deflection_limit_m` | 0.10 m | `arm_tip_deflection` |
| `arm_tip_rotation_limit_rad` | 0.10 rad | `arm_tip_rotation` |
| `eps_acs_fault_margin` | 50.0 Nm | `fault_directional_margin` |
| `require_all_fault_acs_feasible` | true | `all_faults_hover_feasible` |
| `battery_crate_limit` | 5.0 h⁻¹ | `battery_crate_limit` |
| `mission_time_nominal_min` | **30.0 min** | `battery_energy_reserve` |
| `mission_time_emergency_min` | 1.0 min | `battery_energy_reserve` |
| `battery_specific_energy_wh_per_kg` | 250.0 | BatteryEvaluator |
| `battery_dod_usable` | 0.85 | BatteryEvaluator |
| `battery_pack_efficiency` | 0.95 | BatteryEvaluator |
| `power_auxiliary_w` | 500 W | BatteryEvaluator, C-rate |
| `cg_envelope_half_x` | 0.40 m | `cg_envelope` |
| `cg_envelope_half_y` | 0.25 m | `cg_envelope` |
| `figure_of_merit` | 0.65 | PowertrainEvaluator, CruisePowerModel |
| `motor_efficiency` | 0.85 | PowertrainEvaluator |
| `esc_efficiency` | 0.95 | PowertrainEvaluator |
| `parasite_drag_area_m2` | 0.6 m² | CruisePowerModel (mission mode) |
| `mission_profile` | null | MissionEvaluator (optional) |

---

## Objectives

### Weighted-sum SOO objective

```
J = Σ(w_i · f_i) / Σ(w_i)
```

| Objective | Weight | Formula | What it minimizes |
|---|---|---|---|
| `mass` | 0.20 | m_total / 2240.73 | Total vehicle mass |
| `power` | 0.20 | P_nom_w / P_ref_w | Hover power (actuator-disk) |
| `fault_thrust` | 0.25 | max(0, 1.5 − γ_worst)² | Thrust margin shortfall |
| `fault_alloc` | 0.25 | σ_ref / σ_worst | Control authority degradation |
| `hover_nom` | 0.10 | (T_avg / T_max)² | Nominal rotor saturation |
| `acs_margin_penalty` | 0.10 | max(0, −m_min) / (m·g) | ACS margin violation |
| `structural` | 0.00 | BI_norm | Bending index (analysis only) |
| `packaging` | 0.00 | rotor overlap penalty | Rotor overlap (analysis only) |
| `structural_safety` | 0.00 | SF_target / SF_actual | Structural safety ratio (analysis only) |

Objective key name `acs_margin_penalty` must match exactly in `EvaluationContext::objective_weights`.

### Infeasibility penalty

```
J_infeasible = 1e6 + Σ(penalty_i · |violation_i|) + 1e4
```

The extra 1e4 is added whenever `result.feasible == false`, providing a discontinuous
jump at the feasibility boundary that CMA-ES can detect.

### MOO objective set (CLI default)

`mass`, `power`, `fault_alloc` — three objectives forming a tractable Pareto front.
Configurable via `MooRunConfig::objective_names` or the `--moo-*` CLI flags.

---

## Feasibility rule

A design is **feasible** when all of the following hold:

1. Nominal hover trim LP is solvable.
2. All 6 single-motor-fault hover trims are solvable.
3. Every active hard constraint is feasible.

Conditions 1 and 2 are tested directly in `Stage1Evaluator`. Condition 3 is evaluated
by `ConstraintRegistry`. Any violation in any condition sets `result.feasible = false`.

---

## Practical interpretation of the active problem

The optimizer jointly sizes:

| Group | Variables | Driven by constraints |
|---|---|---|
| Geometry | Lx, Lyi, Lyo, T_max | `minimum_geometry_margin`, `rotor_clearance`, `all_faults_hover_feasible`, `failed_hover_gamma`, `fault_directional_margin` |
| Structure | arm_outer_radius, arm_wall_thickness | `arm_yield_failure`, `arm_tip_deflection`, `arm_tip_rotation` |
| Energy | m_bat | `battery_energy_reserve`, `battery_crate_limit` |
| Placement | pax_x/y/z, cargo_x/y/z, bat_x/y/z | `packaging::*`, `cg_envelope` |

The energy group (m_bat) and geometry group (T_max, Lx, Lyi, Lyo) are coupled through the
hover power model: larger arms → larger r_eff → lower induced velocity → lower hover power
→ smaller battery requirement. The optimizer trades arm length against battery mass under
the structural (SF, deflection) and ACS (fault hover, directional margin) constraints.
