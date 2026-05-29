# FrameworkCpp Optimization Overview

Top-level optimization reference. For the full variable and constraint table see
`docs/Optimization_Constraints_And_Design_Variables.md`. For power/battery formulas see
`docs/Phase2_Powertrain_Battery.md`. For implementation checklists see `docs/Action.md`.

---

## Architecture-to-objective flow

```text
active parameters (16)
  -> DesignVectorMapper / BoundsBuilder   [normalized [0,1] per param]
  -> candidate HexacopterArchitecture
  -> VehicleScalingModel                  [mass, COM, inertia, B-matrix]
  -> AttainableControlSetAnalyzer         [ACS trim, margins, fault trims]
  -> PowertrainEvaluator                  [hover power, utilization]
  -> BatteryEvaluator                     [energy reserve, C-rate]
       if context.mission_profile:
         MissionEvaluator -> BatteryEvaluator::evaluateWithMission
  -> StructuralNetworkAnalyzer            [SF, deflection — 8 load cases]
  -> ArchitecturePackagingEvaluator       [rotor clearance, containment, overlap]
  -> ConstraintRegistry evaluation        [all hard constraints]
  -> ObjectiveAggregator                  [weighted-sum combined_objective]
  -> pagmo fitness
```

Order is fixed. ACS must precede structural (fault trim thrusts are structural load cases).
Packaging runs last because it needs the assembled positions post-ACS.

---

## Design vector (16 active parameters)

### Geometry (4)

| Name  | Unit | Default | Bounds       |
|-------|------|---------|--------------|
| `Lx`  | m    | 2.65    | [2.0, 5.0]   |
| `Lyi` | m    | 2.65    | [2.0, 5.0]   |
| `Lyo` | m    | 5.50    | [2.5, 9.0]   |
| `T_max` | N  | 12 000  | [8 000, 20 000] |

### Structure (2)

| Name                 | Unit | Default | Bounds          |
|----------------------|------|---------|-----------------|
| `arm_outer_radius`   | m    | 0.08    | [0.02, 0.15]    |
| `arm_wall_thickness` | m    | 0.005   | [0.001, 0.020]  |

### Energy (1)

| Name    | Unit | Default | Bounds       |
|---------|------|---------|--------------|
| `m_bat` | kg   | 400     | [100, 1000]  |

### Passenger placement (3)

| Name    | Unit | Default | Bounds         |
|---------|------|---------|----------------|
| `pax_x` | m    | 0.00    | [−0.25, +0.25] |
| `pax_y` | m    | 0.00    | [−0.42, +0.42] |
| `pax_z` | m    | −0.10   | [−0.50, +0.40] |

Body-frame NED: z-down (−z = toward cabin ceiling, +z = toward cabin floor).
Default places passenger group center 0.10 m above hub plane.

### Cargo placement (3)

| Name      | Unit | Default | Bounds         |
|-----------|------|---------|----------------|
| `cargo_x` | m    | 0.00    | [−0.50, +0.50] |
| `cargo_y` | m    | 0.00    | [−0.45, +0.45] |
| `cargo_z` | m    | +0.75   | [+0.30, +1.00] |

Default places cargo 0.75 m below hub (near cabin floor), vertically separated from passengers.

### Battery placement (3)

| Name    | Unit | Default | Bounds         |
|---------|------|---------|----------------|
| `bat_x` | m    | 0.00    | [−0.50, +0.50] |
| `bat_y` | m    | 0.00    | [−0.55, +0.55] |
| `bat_z` | m    | −0.90   | [−1.00, +0.85] |

Default places battery 0.90 m above hub (ceiling zone). Lower bound −1.00 keeps the slim
slab (half_z ≤ 0.08 m) inside the cabin ceiling at z = −1.10 m.

### Fixed (inactive) parameters

| Name            | Unit | Default | Reason frozen |
|-----------------|------|---------|---------------|
| `cT`            | —    | 0.03    | MATLAB reference; do not optimize |
| `d_prop`        | m    | 0.40    | Visual/cT scaling only; power uses r_eff |
| `m_pax`         | kg   | 600     | Fixed mission requirement |
| `m_cargo`       | kg   | 200     | Fixed mission requirement |
| `m_instrument`  | kg   | 50      | Fixed avionics mass |

---

## Normalization

Each active parameter maps to pagmo's [0, 1] box:

```
x_norm = (x − x_lower) / (x_upper − x_lower)    (scale = 1.0 for all)
```

`OptimizationProblem.lower_bounds[i]` / `upper_bounds[i]` store **physical** bounds for
CSV/JSON export. `BoundsBuilder` returns normalized bounds to pagmo only.

---

## Evaluation stages

### 1. Vehicle model — `VehicleScalingModel`

- Total mass: sum of all element masses
- COM and inertia (parallel-axis, with body correction constants Ix_0=12000, Iy_0=9400 kg·m²)
- B-matrix (4×6, rows: Fz, Mx, My, Mz)
- Faulted allocation matrices (6 single-motor variants)
- Structural aggregate: arm span, bending index (normalized)
- Rotor disk geometry: r_eff = max_arm_length / 2 (used by powertrain, NOT d_prop)

### 2. ACS — `AttainableControlSetAnalyzer`

Computes hover trim LP for nominal and each of 6 single-motor-fault cases:

- Nominal hover trim thrust vector
- Per-fault trim thrust vectors (used as structural load cases in §5)
- Directional margin m(d) = h_U(d) − d·u_req for 11 directions
- Volume metrics: PFWAR, FII, WCFR
- hover_margin = T_max / T_hover_worst − 1

### 3. Powertrain — `PowertrainEvaluator`

Actuator-disk electrical power per rotor using **r_eff = max_arm_length / 2**:

```
A_eff  = π r_eff²
P_i    = T_i^1.5 / (FM · η_mot · η_esc · √(2 ρ A_eff))
```

FM = 0.65, η_mot = 0.85, η_esc = 0.95, ρ = 1.225 kg/m³.

### 4. Battery — `BatteryEvaluator`

**Legacy hover mode** (no mission profile):

```
E_avail = η_pack · DoD · m_bat · e_spec
E_req   = (P_nom + P_aux) · t_nom + (P_fault + P_aux) · t_emg
```

e_spec = 250 Wh/kg, DoD = 0.85, η_pack = 0.95, t_nom = **30 min**, t_emg = 1 min,
P_aux = 500 W. The 30-minute nominal mission gives a non-trivial energy constraint that
drives m_bat and arm geometry simultaneously.

**Mission mode** (if `context.mission_profile` is set):
`MissionEvaluator` computes per-segment energy (hover, cruise, climb, descent, emergency).
`BatteryEvaluator::evaluateWithMission()` replaces the fixed-time energy budget.

Battery mass feedback loop: larger m_bat → heavier vehicle → more hover thrust → more power
→ more energy needed → drives m_bat higher. Optimizer converges around 650–900 kg for 30 min.

### 5. Structural network — `StructuralNetworkAnalyzer`

Runs **after** ACS (needs fault trim thrusts). 8 load cases:
`max_thrust`, `nominal_hover`, `fault_0` … `fault_5`.

Horizontal cantilever model per arm. Tube: r_o = arm_outer_radius, r_i = r_o − t_wall.

```
I  = π/4 · (r_o⁴ − r_i⁴)     A = π · (r_o² − r_i²)
σ_vm = M · r_o / I              SF = σ_yield / σ_vm
δ    = P·L³/(3EI) + q·L⁴/(8EI)
θ    = P·L²/(2EI) + q·L³/(6EI)
```

Outputs: `struct_net_min_safety_factor`, `struct_net_max_tip_deflection_m`,
`struct_net_max_tip_rotation_rad`, `struct_net_max_sigma_vm_pa`.

### 6. Packaging — `ArchitecturePackagingEvaluator`

Uses AABB containment checks on assembled element positions (world_pose × localEnvelope).
Internal elements own their placement via `local_pose_` (no attachment lambda).

| Check | Result field |
|---|---|
| Rotor-to-rotor disk clearance | `minimum_rotor_clearance` |
| Passenger / cargo / instrument in cabin | `payload_containment_violation` |
| Battery in cabin | `battery_containment_violation` |
| Battery vs payload elements overlap | `battery_payload_overlap` |
| Occupant envelope in cabin | `occupant_containment_violation` |
| Passenger / cargo / instrument mutual overlap | `payload_internal_overlap` |

### 7. Constraint evaluation

Builds `ConstraintEvaluationContext{arch, physical_model, stage1_metrics, eval_context}`.
Runs all constraints registered in `HexacopterArchitecture::registerDefaultConstraints()`
and element `registerConstraints()` methods. Results go to `EvaluationResult.constraint_results`.

### 8. Objective aggregation — `ObjectiveAggregator`

```
J = Σ(w_i · f_i) / Σ(w_i)
```

Only objectives with w_i > 0 enter J. Inactive objectives (w=0) are still computed and exported.

---

## Feasibility

A result is feasible when all three hold:
1. Nominal hover trim LP is feasible.
2. All 6 single-fault hover trims are feasible (`require_all_fault_acs_feasible = true`).
3. Every active hard constraint evaluates feasible.

Infeasible points are evaluated, penalized, and exported. Pagmo receives:

```
fitness = 1e6 + Σ(penalty_i · |violation_i|) + 1e4   [when result.feasible == false]
```

---

## Objectives

### SOO combined objective (all minimized)

| Objective           | Weight | Formula | Notes |
|---------------------|--------|---------|-------|
| `mass`              | 0.20   | m_total / 2240.73 | Normalized by MATLAB reference |
| `power`             | 0.20   | P_nom / P_ref | P from actuator-disk; ref from default arch |
| `fault_thrust`      | 0.25   | max(0, 1.5 − γ_worst)² | Squared thrust-margin shortfall |
| `fault_alloc`       | 0.25   | σ_ref / σ_worst | Scaled control effectiveness ratio |
| `hover_nom`         | 0.10   | (T_avg / T_max)² | Nominal hover saturation proxy |
| `acs_margin_penalty`| 0.10   | max(0, −margin_min) / (m·g) | Zero when ACS healthy |
| `structural`        | 0.00   | BI_norm | Analysis only; raise weight to activate |
| `packaging`         | 0.00   | rotor overlap penalty | Analysis only |
| `structural_safety` | 0.00   | SF_min_ratio | Analysis only |

Weight key in `EvaluationContext::objective_weights`. The name `acs_margin_penalty` must match
exactly — the `ObjectiveAggregator` looks up by name.

`fault_thrust` and `acs_margin_penalty` overlap with hard constraints `failed_hover_gamma` and
`all_faults_hover_feasible` intentionally: the soft terms provide a gradient in the infeasible
region, while the hard constraints gate final feasibility.

### MOO objective vector

Default set (CLI):  `mass`,  `power`,  `fault_alloc`  — configured in `main.cpp`.
Full code default: `mass`, `power`, `fault_thrust`, `fault_alloc`, `hover_nom`.

---

## Allocation matrix

```
u = B · T,   B ∈ R^{4×6},   T ∈ [0, T_max]^6

Row 0 (Fz):  −1    −1    −1    −1    −1    −1
Row 1 (Mx):  −Lyi  +Lyi  −Lyo  +Lyo  −Lyi  +Lyi
Row 2 (My):  +Lx   +Lx    0     0    −Lx   −Lx
Row 3 (Mz):  −cT   −cT   +cT   +cT   −cT   +cT
```

Motor world positions (all at z = 0):
`(+Lx, −Lyi)`, `(+Lx, +Lyi)`, `(0, −Lyo)`, `(0, +Lyo)`, `(−Lx, −Lyi)`, `(−Lx, +Lyi)`.

For faulted rotor f: column f of B set to zero.

---

## Hover feasibility LP

Finds T ∈ [0, T_ub] with B·T = [−m·g, 0, 0, 0]ᵀ.
Enumerates all C(6,2)×4 = 60 basis vertex configurations:

1. Fix T_i and T_j at boundary values {0, T_ub}.
2. Solve 4×4 linear system for the remaining four.
3. Accept if all six thrusts lie in [−1e-6, T_ub + 1e-6].
4. Keep the feasible candidate minimising Σ T_k.

---

## Structural proxy (bending index)

```
S_arm   = Σ L_arm_i             (total arm span = 2Lx + 2Lyi + 2Lyo at baseline)
BI      = T_max · S_arm / 6
BI_norm = BI / (7327 · 5.50)    (normalized by MATLAB calibration reference)
```

Used as the `structural` soft objective (weight 0 by default).

---

## Pagmo integration

`PagmoProblemAdapter` is the single pagmo integration point.

- Clones the architecture, unpacks normalized vector, runs Stage 1 evaluation.
- SOO: returns scalar `combined_objective`; adds `penalty_weight × violation` per violated constraint.
- MOO: returns objective vector; penalties added per objective dimension.

### SOO defaults (CMA-ES)

| Parameter | Default |
|---|---|
| Population | 24 |
| Generations | 40 |
| Seed | 42 |
| Tolerance | 1e-6 |

### MOO defaults (NSGA-II)

| Parameter | Default |
|---|---|
| Population | 48 |
| Generations | 60 |
| Seed | 42 |
| Crossover η | 10 |
| Mutation η | 50 |
| Mutation rate | 1 / n_vars |

---

## Calibration constants (do not optimize)

Fixed in `VehicleScalingModel.cpp` and `Elements.cpp`:

| Constant | Value | Purpose |
|---|---|---|
| m_ref | 2240.73 kg | Mass objective denominator |
| m_frame_baseline_payload | 1500 kg | Frame mass allometric calibration |
| m_mot_0 | 74.07 kg | Motor mass allometric baseline |
| T_max_0 | 7327 N | Motor mass / BI normalization |
| S_arm_0 | 21.60 m | Frame mass scaling reference |
| Ix_body | 12 000 kg·m² | Body inertia correction |
| Iy_body | 9 400 kg·m² | Body inertia correction |

`m_frame_baseline_payload = 1500 kg` is the **calibration** constant in the frame mass formula,
not the mission payload. Mission payload = m_pax + m_cargo + m_instrument = 850 kg (fixed).

---

## Internal spatial elements (element-owned placement)

Internal payload elements own their placement via `local_pose_`. Attachments to body are rigid mounts
(no lambda). `world_pose = body_pose × local_pose_`.

| Element | Type | Mass | Placement DOF | Geometry |
|---|---|---|---|---|
| `battery` | `BatteryElement` | m_bat (active) | bat_x/y/z | Slim ceiling slab 1.30×1.50×0.16 m (max) |
| `passenger` | `PassengerElement` | m_pax = 600 kg | pax_x/y/z | 4 seat boxes (0.50×0.44×1.20 m each) |
| `cargo` | `CargoElement` | m_cargo = 200 kg | cargo_x/y/z | Floor slab 0.60×0.84×0.36 m |
| `instrument_panel` | `InstrumentPanelElement` | 50 kg | fixed x=+0.68 | Panel 0.16×1.24×0.70 m |
| `cabin_envelope` | `CabinEnvelopeElement` | 0 | fixed at body center | Wireframe 1.60×1.80×2.20 m |
| `occupant_envelope` | `OccupantEnvelopeElement` | 0 | fixed at cabin center | Wireframe 1.06×0.94×1.20 m |

Battery geometry is physics-based: `half_z = m_bat / (1500 · 4 · 0.65 · 0.75)`, capped at 0.08 m
for slim appearance. Energy physics still use m_bat directly.

Default z-stratification (no visual overlap at baseline):
- Battery: z ∈ [−0.98, −0.82] (ceiling)
- Passengers: z ∈ [−0.70, +0.50] (mid-cabin)
- Cargo: z ∈ [+0.57, +0.93] (floor slab)

---

## Body hull geometry (optimization-coupled)

`BodyHullElement` scales with arm geometry so body volume is not decoupled from the optimized design:

```
half_x = kCabinHalfX + 0.05 + 0.05 · Lx
half_y = kCabinHalfY + 0.05 + 0.05 · Lyi
half_z = 1.25 m  (fixed; contains cabin at ±1.10 m)
```

At Lx = Lyi = 2.65 m (default): half_x ≈ 0.98 m, half_y ≈ 1.08 m — well clear of the rotor
inner edge (≈ 1.96 m from center). Rendered as wireframe.
