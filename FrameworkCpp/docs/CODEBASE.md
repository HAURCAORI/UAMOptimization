# CODEBASE.md — FrameworkCpp Architecture Reference

Read this file at the start of every implementation session before modifying any code.
Update the relevant section after completing an implementation.

---

## Directory Layout

```
FrameworkCpp/
├── app/
│   └── main.cpp                    CLI entry point (eval/soo/moo/compare/visualize modes)
├── data/
│   ├── example_flight_data.csv     Synthetic hover + cruise data for calibration smoke-tests
│   └── example_mission_profile.json  City-taxi round-trip profile (12 segments)
├── tools/
│   └── run_demo.ps1                One-click demo: eval / mission / calibrate / soo+mission
├── include/
│   ├── analysis/
│   │   ├── AcsPlotter.hpp          SVG-based ACS visualization (no external deps)
│   │   ├── ComparisonReporter.hpp  Console summary and table helpers
│   │   ├── CsvExporter.hpp         All file export declarations
│   │   └── ParetoAnalyzer.hpp      Knee point and dominance analysis
│   ├── calibration/
│   │   ├── CalibrationProblem.hpp  Residual cost + pack/unpack helpers; wraps FlightDataPoints
│   │   ├── Calibrator.hpp          Nelder–Mead box-projected optimizer; CalibrationOutcome
│   │   ├── FlightData.hpp          loadFlightDataCsv() — CSV parser returning FlightDataPoints
│   │   └── FlightDataPoint.hpp     One measured operating point (hover or cruise)
│   ├── mission/
│   │   ├── CruisePowerModel.hpp    Forward-flight power (momentum-theory + parasite drag)
│   │   ├── MissionEvaluator.hpp    Multi-segment UAM mission energy evaluator; MissionResult
│   │   └── MissionProfile.hpp      MissionProfile / MissionSegment structs + JSON loader
│   ├── core/
│   │   ├── AppliedLoad.hpp         AppliedLoad struct + ILoadReceiver interface
│   │   ├── AssemblyState.hpp       Computed assembly: element positions/orientations
│   │   ├── Attachment.hpp          Attachment (parent/child anchor wiring)
│   │   ├── Constraint.hpp          Constraint struct and ConstraintSense enum
│   │   ├── ConstraintRegistry.hpp  Registry for all constraints
│   │   ├── DefaultHexacopterBuilder.hpp  DefaultHexacopterParameters struct + builder decls
│   │   ├── DesignParameter.hpp     DesignParameter struct (value/bounds/scale/unit)
│   │   ├── ElementCapabilities.hpp Capability interfaces: IPropulsionRotor, IStructuralMember,
│   │   │                           IStructuralBeam (extends IStructuralMember), ILoadReceiver,
│   │   │                           IMotorMassContributor, IPayloadMassContributor,
│   │   │                           IEnergyStorage (Phase 2)
│   │   ├── Elements.hpp            Concrete elements: BodyHullElement (renamed from BodyElement),
│   │   │                           BodyFrameElement (Task 5, hub chassis, zero-mass), ArmElement,
│   │   │                           MotorElement, PropellerElement, PayloadElement,
│   │   │                           BatteryElement (Phase 2), CabinEnvelopeElement,
│   │   │                           OccupantEnvelopeElement (Step 3, zero-mass),
│   │   │                           KeepOutZoneElement (Task 4, rotor keep-out cylinder with safety margin)
│   │   ├── GeometryPrimitive.hpp   GeometryPrimitive struct (AABB, sphere, etc.)
│   │   ├── HexacopterArchitecture.hpp  Root object; owns registry, elements, builder
│   │   ├── ParameterRegistry.hpp   Registry for all DesignParameters
│   │   ├── SpatialElement.hpp      SpatialElement + BasicSpatialElement base classes
│   │   └── Timestamp.hpp           currentTimestamp() helper
│   ├── evaluation/
│   │   ├── ArchitectureEvaluator.hpp  Top-level evaluation entry point
│   │   ├── EvaluationContext.hpp   Weights, material, constraint thresholds, seeds
│   │   ├── EvaluationResult.hpp    Stage1Metrics, EvaluationResult, ConstraintResult, ObjectiveValue
│   │   ├── MetricRole.hpp          MetricRole enum + MetricDescriptor + stage1MetricDescriptors() (Phase 5)
│   │   ├── ObjectiveAggregator.hpp Weighted sum aggregator
│   │   └── Stage1Evaluator.hpp     Stage 1 metric computation
│   ├── optimization/
│   │   ├── BoundsBuilder.hpp       Normalized [0,scale] bounds for pagmo (DO NOT use for display)
│   │   ├── DesignVectorMapper.hpp  Pack/unpack normalized design vectors
│   │   ├── MooRunner.hpp           MooRunResult, MooRunConfig, MooRunner class
│   │   ├── OptimizationProblem.hpp OptimizationProblem struct (PHYSICAL bounds after fix)
│   │   ├── PagmoProblemAdapter.hpp Pagmo2 interface adapter
│   │   └── SooRunner.hpp           SooRunResult, SooRunConfig, SooRunner class
│   └── physics/
│       ├── AllocationMatrixBuilder.hpp  Motor allocation and fault analysis
│       ├── AttainableControlSetAnalyzer.hpp  ACS trim, directional margins, volume metrics
│       ├── BatteryEvaluator.hpp    Phase 2: battery energy budget (Wh, reserve, C-rate)
│       ├── Material.hpp            Material struct + Materials::Al7075/CFRP/Steel304
│       ├── PhysicsTypes.hpp        PhysicalModel, PropulsionProxy, StructuralProxy,
│       │                           ArmStructuralResult, MemberLoadResult (Phase 3),
│       │                           PackagingResult, PowertrainResult, BatteryResult (Phase 2)
│       ├── PowertrainEvaluator.hpp Phase 2: per-motor electrical power (actuator-disk model)
│       ├── PrimitiveDistance.hpp   Geometry collision/clearance helpers
│       ├── ArchitecturePackagingEvaluator.hpp  Phase 6: rotor clearance + future containment
│       ├── StructuralAnalyzer.hpp  Cantilever beam safety factor analysis (legacy; min_sf path)
│       ├── StructuralNetworkAnalyzer.hpp  Phase 3: multi-member network (von Mises, deflection)
│       └── VehicleScalingModel.hpp Mass/geometry/propulsion scaling
├── src/                            Mirrors include/ layout; each .cpp implements its .hpp
├── visualization/                  ImGui/Vulkan 3D viewer (ArchitectureViewerApp)
│   ├── ArchitectureSceneBuilder    Converts AssemblyState → vector<PrimitiveInstance>
│   ├── PrimitiveMeshFactory        Unit meshes per primitive kind (box/sphere/cylinder/disk/segment)
│   │                               segment → makeUnitCylinder() (not box); scale uses padding as radius
│   └── renderElementListPanel()    ImGui left panel showing all elements (ID, type, mass, position)
└── docs/
    ├── Action.md                   Step-by-step guide for adding parameters/metrics/constraints
    ├── CODEBASE.md                 This file
    └── Phase2_Powertrain_Battery.md  Phase 2 spec: power model, battery sizing, constraints
```

---

## Key Classes and Responsibilities

### `HexacopterArchitecture` (core)
- Root object. Owns `ParameterRegistry`, `ConstraintRegistry`, element list, `AssemblyState`.
- `registerDefaultParameters()`: adds all `DesignParameter` entries.
- `bindCanonicalParameters()`: caches typed pointers to well-known parameters.
- `registerDefaultConstraints()`: adds hard/soft constraints with lambda evaluators.
- `rebuildAssembly()`: calls builder → reassembles elements and attachments.
- `elements()`: returns the current element list (shared_ptrs to `SpatialElement`).
- `findElement(id)`: linear search by element ID.
- `cT()`, `dprop()`: cached canonical parameters.

### `DesignParameter` (core)
- Fields: `name`, `owner_id`, `unit`, `description`, `value`, `lower_bound`, `upper_bound`,
  `default_value`, `active`, `scale`, `consumer_ids`.
- `normalized()` = `((value - lower_bound) / span) * scale`. With `scale=1`, result is in [0, 1].
- `setFromNormalized(x)` = `lower_bound + span * (x / scale)`.
- `normalizedAt(raw)` computes normalized for an arbitrary raw value.

### `SpatialElement` / `BasicSpatialElement` (core)
- Abstract base for all vehicle components.
- Owns mass, geometry primitives, anchors, constraints.
- `updateFromParameters()`: called when parameter values change; updates geometry.
- `rebindParameters()`: re-caches raw pointers after architecture copy/move.

### Capability Interfaces (core/ElementCapabilities.hpp)
| Interface | Methods | Used by |
|---|---|---|
| `IPropulsionRotor` | `thrustAxis()`, `thrustCoeff()`, `maxThrust()` | MotorElement |
| `IStructuralMember` | `structuralSpanContribution()` | ArmElement |
| `IStructuralBeam` | `outerRadius()`, `innerRadius()`, `crossSectionArea()`, `secondMomentOfArea()`, `polarMomentOfArea()` | ArmElement |
| `ILoadReceiver` | `clearLoads()`, `addLoad()`, `loads()` | ArmElement |
| `IMotorMassContributor` | `motorMass()` | MotorElement |
| `IPayloadMassContributor` | `payloadMass()` | PayloadElement |

### `MissionEvaluator` (mission)
Evaluates a multi-segment UAM mission (hover, cruise, climb, descent, emergency_hover,
reserve_hover) and returns a `MissionResult` with per-segment energy and range breakdown.
Hover legs reuse the ACS trim thrusts from `PowertrainResult`; cruise/climb/descent legs
delegate to `CruisePowerModel`. Called from `Stage1Evaluator` when `context.mission_profile`
is set. `BatteryEvaluator::evaluateWithMission()` then uses the result instead of the legacy
fixed-time hover legs. Key context fields: `parasite_drag_area_m2`, `air_density`.

### `Calibrator` (calibration)
Nelder–Mead box-projected optimizer that identifies physics parameters
(`figure_of_merit`, `motor_efficiency`, `esc_efficiency`, `battery_specific_energy_wh_per_kg`,
`battery_pack_efficiency`, `parasite_drag_area_m2`) from flight-log CSVs. Residual =
mean-squared relative error in electrical power (hover: actuator-disk; cruise: CruisePowerModel).
Use `applyToContext()` to push fitted values into an `EvaluationContext` before a run.

### `Stage1Evaluator` (evaluation)
Evaluation pipeline in order:
1. `VehicleScalingModel::evaluate()` → fills `PhysicalModel` (mass now includes m_bat, propulsion, packaging)
2. `AttainableControlSetAnalyzer::analyze()` → fills `AcsResult` (trim, directional margins,
   volume metrics, per-axis reserves, fault degradation ratio)
3. **`PowertrainEvaluator::evaluate()`** (Phase 2) → fills `PowertrainResult` (motor power [W], utilizations)
4. **`BatteryEvaluator::evaluate/evaluateWithMission()`** (Phase 2) → fills `BatteryResult`. If
   `context.mission_profile` is set, runs `MissionEvaluator` first and uses `evaluateWithMission()`;
   otherwise falls back to the legacy fixed-time hover budget. Sets `stage1.mission_*` fields.
5. **`StructuralNetworkAnalyzer::analyze()`** (Phase 3) → fills `network_structural` (per-member per-load-case),
   `structural.min_safety_factor`, `structural.network_*` fields. Uses ACS fault trims as load cases.
6. Computes `Stage1Metrics` from `PhysicalModel` + `AcsResult` + `PowertrainResult` + `BatteryResult`
7. Calls `ConstraintRegistry` evaluators → fills `constraint_results`
8. Appends `acs::all_faults_hover_feasible` as a named hard constraint to `constraint_results`
9. Calls `ObjectiveAggregator::aggregate()` → fills `objectives`, `combined_objective`

> **StructuralAnalyzer** (the old single-load-case analyzer) is still registered in vcxproj but is no
> longer called by Stage1Evaluator. StructuralNetworkAnalyzer supersedes it and writes the same
> `structural.min_safety_factor` field that the `arm_yield_failure` constraint reads.

### `OptimizationProblem` (optimization)
Returned by `PagmoProblemAdapter::problem()`. Contains **physical** (not normalized) bounds:
- `lower_bounds[i]` = `parameter->lower_bound` (physical units)
- `upper_bounds[i]` = `parameter->upper_bound` (physical units)
- `parameter_scales[i]` = `parameter->scale` (typically 1.0)
- `parameter_units[i]` = `parameter->unit` (e.g., "m", "N", "")
- `parameter_ids[i]` = stable ID (e.g., `"hexa::arm_outer_radius"`)

The `BoundsBuilder` is only used internally by pagmo via `get_bounds()` (normalized).
The `problem()` accessor is for export/display; it must store physical bounds.

### `DesignVectorMapper` (optimization)
- `parameterIds(arch)`: returns stable IDs of active parameters in order.
- `unpackNormalized(arch, x)`: calls `parameter->setFromNormalized(x[i])` for each active param.
- `packNormalized(arch)`: returns `parameter->normalized()` for each active param.

### `CsvExporter` (analysis)
All export methods are static. Internally uses `denormalizeParameter(problem, i, norm)`:
```
physical = lower + (upper - lower) * (norm / scale)
```
This works correctly because `OptimizationProblem` now stores physical bounds.

Key export functions:
- `writeSooParametersCsv` — baseline/raw_best/best_feasible rows, columns = active params
- `writeParetoParametersCsv` — per-Pareto-point rows
- `writeSooJson` / `writeMooJson` — full JSON export including physical model
- `writeParetoCsv` — Pareto front with objectives and SF
- `writeComparisonCsv` / `summaryTable` in `ComparisonReporter` — 15-column metric table

### `ComparisonReporter` (analysis)
- `summarize(EvaluationResult)`: one-line metric summary with feasibility and SF
- `summarize(SooRunResult)` / `summarize(MooRunResult)`: run summary
- `compare(baseline, candidate)`: delta between two results
- `summaryTable(labeled_results)`: CSV-formatted 15-column table (feasible, objective, mass, ...)
- `parametersTable(SooRunResult)`: aligned table showing baseline/best physical param values
- `parametersTable(MooRunResult)`: aligned table showing baseline physical param values

---

## Data Flow

```
HexacopterArchitecture (parameters, elements, attachments)
    │
    ├── ParameterRegistry.activeParameters()
    │       └──► DesignVectorMapper packs/unpacks [normalized]
    │                └──► pagmo optimizer sees get_bounds() [normalized 0..scale]
    │
    └── ArchitectureEvaluator::evaluate(arch, context)
            └──► Stage1Evaluator::evaluate()
                    ├── VehicleScalingModel                → PhysicalModel (mass incl. m_bat, thrust, geometry)
                    ├── AttainableControlSetAnalyzer       → AcsResult (trim, margins, reserves)
                    ├── PowertrainEvaluator [Phase 2]      → PowertrainResult (motor power [W])
                    ├── BatteryEvaluator    [Phase 2]      → BatteryResult (energy [Wh], reserve, C-rate)
                    ├── StructuralNetworkAnalyzer [Phase 3] → network_structural, min_safety_factor,
                    │                                          network_min_sf, max_tip_deflection, max_sigma_vm
                    ├── ConstraintRegistry                 → constraint_results (arch constraints)
                    ├── [Stage1Evaluator]                  → appends acs::all_faults_hover_feasible
                    └── ObjectiveAggregator                → combined_objective
                            └──► EvaluationResult

SooRunner / MooRunner
    ├── PagmoProblemAdapter::problem() → OptimizationProblem [PHYSICAL bounds]
    ├── pagmo runs fitness() in normalized space
    └── SooRunResult / MooRunResult
            └──► CsvExporter, ComparisonReporter
                    └── denormalizeParameter(problem, i, x) → physical values
```

---

## Current Design Parameters (HexacopterArchitecture.cpp)

Baseline constants: `kBaselineLx=2.65`, `kBaselineLyi=2.65`, `kBaselineLyo=5.50`,
`kBaselineTmax=8000.0 N`, `kBaselineCT=0.03`,
`kBaselinePropDiameter=0.40`, `kBaselinePayload=800.0`, `kBaselineBatteryMass=400.0`.

| Stable ID (owner omitted) | Unit | Default | Bounds [lo, hi] | Active |
|---|---|---|---|---|
| `Lx` | m | 2.65 | [2.0, 5.0] | **true** |
| `Lyi` | m | 2.65 | [2.0, 5.0] | **true** |
| `Lyo` | m | 5.50 | [2.5, 9.0] | **true** |
| `T_max` | N | 8000 | [8000, 20000] | **true** |
| `cT` | — | 0.03 | [0.01, 0.08] | false |
| `d_prop` | m | 0.40 | [0.20, 1.20] | false |
| `m_payload` | kg | 800 | [400, 2000] | false |
| `arm_outer_radius` | m | 0.08 | [0.02, 0.15] | **true** |
| `arm_wall_thickness` | m | 0.005 | [0.001, 0.020] | **true** |
| `m_bat` | kg | 400 | [100, 1000] | **true** (Phase 2) |
| `z_bat_offset` | m | 0.0 | [−0.20, +0.50] | **true** (Task 3) |
| `x_payload` | m | 0.0 | [−0.30, +0.30] | **true** (Task 3) |
| `y_payload` | m | 0.0 | [−0.40, +0.40] | **true** (Task 3) |

Active parameter count for optimizer: **10**
(Lx, Lyi, Lyo, T_max, arm_outer_radius, arm_wall_thickness, m_bat, z_bat_offset, x_payload, y_payload).

> **Topology constraint (motor 3/4 fault):** Motors 3,4 are on the pure-roll axis (y-axis, zero
> pitch arm). When either fails, hover trim forces T1=T6=mg/2. ACS fault feasibility requires
> `T_max ≥ mg/2`. With `m_payload=800`, `m_bat≈400`, `structural≈650`: `mg/2 ≈ 9.1 kN`, so
> the optimizer must push T_max above ~9.5 kN. T_max upper bound raised to 20 kN accordingly.
>
> **Arm balance:** Lx/Lyi lower bound 2.0m prevents arm_1256=√(Lx²+Lyi²) from collapsing to
> 1.4m while Lyo→9m (old bound 1.0m allowed 6.4× imbalance; new bound caps it at ~3.2×).
>
> **Battery parameters (EvaluationContext):** `e_spec=250 Wh/kg` (NMC811 cell-level, 2025-era);
> `C_rate_max=5.0 h⁻¹` (peak burst during hover, not continuous cruise).
>
> `BatteryElement.mass_` is now set from `m_bat->value`, so battery mass contributes to total
> vehicle mass and changes the hover thrust requirement, creating a feedback loop with the battery
> energy constraint.

---

## Current Stage 1 Metrics (Stage1Metrics struct)

### Objective terms (weights in EvaluationContext)
| Field | Weight | Description |
|---|---|---|
| `mass` | 0.20 | Normalized total vehicle mass |
| `power` | 0.20 | Normalized nominal hover power proxy |
| `fault_thrust` | 0.25 | Squared thrust-margin violation penalty |
| `fault_alloc` | 0.25 | Control allocation degradation ratio |
| `hover_nom` | 0.10 | Nominal hover utilization proxy |
| `structural` | 0.00 | Normalized bending index (analysis only unless weight raised) |
| `packaging` | 0.00 | Overlap penalty (analysis only) |
| `structural_safety` | 0.00 | Safety factor ratio (analysis only) |
| `acs_margin_penalty` | 0.10 | `max(0, -overall_min_margin) / (m*g)` — zero when ACS healthy |

### ACS metrics (Phase 1 — spec §9.8 / §9.9)
| Field | Description |
|---|---|
| `acs_nominal_feasible` | Nominal hover trim LP feasible |
| `acs_all_faults_feasible` | All 6 single-motor fault hover trims feasible |
| `acs_nominal_min_margin` | Min directional margin (nominal case) |
| `acs_worst_fault_min_margin` | Min directional margin across all fault cases |
| `acs_overall_min_margin` | min(nominal, worst_fault) min margin |
| `acs_PFWAR` | Probabilistic Fault-Weighted ACS Retention (equal fault weights) |
| `acs_FII` | Fault Isotropy Index = std(retention) / mean(retention) |
| `acs_WCFR` | Worst-Case Fault Retention = min(retention) |
| `acs_hover_margin` | T_max / T_hover_worst − 1 (≥0 means all faults can hover) |
| `acs_yaw_reserve` | Margin in yaw_pos direction at nominal hover [Nm] |
| `acs_roll_reserve` | Margin in roll_pos direction at nominal hover [Nm] |
| `acs_pitch_reserve` | Margin in pitch_pos direction at nominal hover [Nm] |
| `acs_faulted_to_nominal_ratio` | worst_fault_min_margin / nominal_min_margin ∈ [0,1] |

### Powertrain metrics (Phase 2 — D)
| Field | Unit | Description |
|---|---|---|
| `pt_total_power_nominal_w` | W | Total electrical power, nominal hover trim |
| `pt_total_power_faulted_w` | W | Total electrical power, worst-fault hover trim |
| `pt_worst_thrust_utilization` | — | max(T_i/T_max) at nominal hover ∈ [0,1] |
| `pt_worst_power_utilization` | — | max(P_i/P_cont) = u_T^{3/2} ∈ [0,1] |

> Power uses actuator-disk with `r_eff = max_arm_length/2` (NOT d_prop). See `Phase2_Powertrain_Battery.md`.

### Battery metrics (Phase 2 — E)
| Field | Unit | Description |
|---|---|---|
| `bat_available_energy_wh` | Wh | E_avail = η_pack × DoD × m_bat × e_spec |
| `bat_required_energy_wh` | Wh | E_req_total = E_req_nom + E_req_emg |
| `bat_energy_reserve_fraction` | — | (E_avail − E_req) / E_avail; ≥0 = feasible |
| `bat_c_rate` | 1/h | P_peak / E_avail (voltage-invariant C-rate) |
| `bat_mass_fraction` | — | m_bat / m_total ∈ [0,1] |
| `bat_achievable_endurance_nom_min` | min | E_avail / (P_nom + P_aux) × 60; actual hover time |

### Mission profile metrics (when `context.mission_profile` is set)
| Field | Unit | Description |
|---|---|---|
| `mission_active` | bool | true when mission profile was evaluated |
| `mission_total_time_s` | s | total mission duration |
| `mission_total_distance_m` | m | total distance flown |
| `mission_cruise_distance_m` | m | cruise/climb/descent legs only |
| `mission_total_energy_wh` | Wh | propulsion energy only |
| `mission_energy_with_aux_wh` | Wh | propulsion + auxiliary |
| `mission_peak_power_w` | W | max per-segment electrical power |
| `mission_hover_energy_wh` | Wh | hover/emergency/reserve energy |
| `mission_cruise_energy_wh` | Wh | cruise/climb/descent energy |
| `mission_energy_reserve_fraction` | — | (E_avail − E_with_aux) / E_avail |

### Structural network metrics (Phase 3)
| Field | Unit | Description |
|---|---|---|
| `struct_net_min_safety_factor` | — | Worst safety factor over all arms × load cases (= `structural.min_safety_factor`) |
| `struct_net_max_tip_deflection_m` | m | Worst Euler-Bernoulli tip deflection over all arms × load cases |
| `struct_net_max_tip_rotation_rad` | rad | Worst Euler-Bernoulli tip rotation over all arms × load cases |
| `struct_net_max_sigma_vm_pa` | Pa | Worst von Mises stress over all arms × load cases |

**Load cases** (8 total): `max_thrust` (all rotors at T_max), `nominal_hover` (ACS LP trim), `fault_0`..`fault_5` (6 single-motor fault trims from ACS).

**Stress model** (horizontal cantilever arms): N=0 (vertical loads ⊥ to arm axis); T_torsion=0 (yaw torque about z-axis → zero projection onto horizontal arm). Yaw reaction manifests as horizontal bending M_h = cT·T. Von Mises: σ_vm = √((σ_ax + σ_b)² + 3τ²) ≈ σ_b.

---

## Current Hard Constraints

### From ConstraintRegistry (registered in `HexacopterArchitecture::registerDefaultConstraints()`)

| Stable ID | Sense | Threshold | Penalty | What it checks |
|---|---|---|---|---|
| `parameter_bounds` | ≤ | 0.0 | 1000 | max over-bound violation across all parameters |
| `minimum_geometry_margin` | ≥ | 0.0 | 1000 | min(Lx − min_arm, Lyi − min_arm, Lyo − Lyi − delta) |
| `rotor_clearance` | ≥ | 0.0 | 1000 | minimum rotor-to-rotor disk clearance |
| `failed_hover_gamma` | ≥ | 1.5 | 2000 | `gamma_worst` = (5·T_max)/mg (worst single-fault thrust ratio) |
| `fault_allocation_ratio` | ≥ | 0.05 | 1500 | `sigma_worst / sigma_reference` (control allocation under fault) |
| `arm_yield_failure` | ≥ | 1.5 (configurable) | 2000 | `structural.min_safety_factor` (now set by StructuralNetworkAnalyzer, Phase 3) |
| `arm_tip_deflection` | ≤ | 0.0 | 1500 | `(δ_max / δ_allow) − 1` where `δ_allow = arm_tip_deflection_limit_m` (Phase 4) |
| `arm_tip_rotation` | ≤ | 0.0 | 1500 | `(θ_max / θ_allow) − 1` where `θ_allow = arm_tip_rotation_limit_rad` (Phase 4) |
| `packaging::payload_in_cabin` | ≤ | 0.0 | 1000 | payload AABB containment violation inside cabin AABB |
| `packaging::battery_in_cabin` | ≤ | 0.0 | 1000 | battery AABB containment violation inside cabin AABB |
| `packaging::battery_payload_nonoverlap` | ≤ | 0.0 | 1000 | battery–payload AABB overlap magnitude |
| `packaging::occupant_in_cabin` | ≤ | 0.0 | 1000 | occupant envelope containment violation inside cabin AABB |
| `packaging::rotor_keepout` | ≤ | 0.0 | 1000 | max occupant/payload/battery overlap with any rotor keep-out cylinder (Task 4, reviewed) |
| `packaging::cg_y_window` | ≤ | 0.0 | 500 | `|cg_y| − 0.05 m` (lateral CG symmetry) |
| `battery_energy_reserve` | ≥ | 0.0 | 2000 | `bat_energy_reserve_fraction` (Phase 2) |
| `battery_crate_limit` | ≤ | 0.0 | 1500 | `bat_c_rate / C_allow − 1` (Phase 2) |

`minimum_arm_length` default = 0.5 m; `minimum_outer_arm_delta` default = 0.1 m.

### From element constraints (registered in `registerElementConstraints()`)

| Stable ID | Owner | Sense | Penalty | What it checks |
|---|---|---|---|---|
| `battery_mass_positive` | battery | ≥ 0.0 | 500 | m_bat > 0 (Phase 2) |

### From Stage1Evaluator (appended directly to `constraint_results`)

| Stable ID | Hard when | What it checks |
|---|---|---|
| `acs::all_faults_hover_feasible` | `require_all_fault_acs_feasible=true` (default) | all 6 single-fault hover LP trims feasible |

Appears in `ComparisonReporter::summarize()` when violated.

## ACS Analysis (AttainableControlSetAnalyzer)

**Direction samples (11):** thrust, roll±, pitch±, yaw±, roll_yaw_pp, pitch_yaw_pp, roll_yaw_pn, pitch_yaw_pn

**Support function:** `h_U(d) = Σ_j max(0, d'·b_j) · f_max_j`

**Directional margin:** `m(d) = h_U(d) − d'·u_req` (positive = feasible in direction d)

**Correctness invariants:**
- Origin `u=0`: `m(d) ≥ 0` for all d; `m([+1,0,0,0]) = 0` (on ACS boundary — NOT infeasible)
- Hover `u=[-mg,0,0,0]`: `m(d) > 0` for all d (strictly inside ACS)
- Wrong sign `d'·u_req − h_U(d)` gives `≤ 0` at origin — always incorrect

**AcsPlotter outputs** (generated by `--plot-acs` or interactive prompt):
```
acs_fig1_lm.svg       vertex scatter [L vs M]
acs_fig2_lfz.svg      vertex scatter [L vs Fz]
acs_fig3_hover_lm.svg hover slice (Fz=-W) [L vs M]
acs_fig4_n0_lfz.svg   N=0 slice [L vs Fz]
acs_fig5_hover_poly.svg  hover moment polygon with worst-fault overlay
acs_fig6_margins.svg  grouped bar: directional margins, nominal vs worst-fault
acs_retention.svg     per-motor ACS retention bars (PFWAR, WCFR reference lines)
acs_vertices.csv      full 4D vertex cloud (nominal + worst-fault motor)
```

---

## Invariants and Gotchas

1. **Normalization vs physical values**: Decision vectors are always in normalized [0, scale] space.
   `OptimizationProblem.lower_bounds/upper_bounds` store PHYSICAL values (not normalized).
   `BoundsBuilder` is for pagmo's `get_bounds()` only — it returns normalized bounds.

2. **Element ID convention**: Arms are `"arm_1"..."arm_6"`, motors are `"motor_1"..."motor_6"`.
   `StructuralAnalyzer` derives motor ID from arm ID by replacing `"arm"` prefix with `"motor"`.

3. **Parameter stable IDs**: `owner_id + "::" + name` (e.g., `"hexa::arm_outer_radius"`).
   Short name stripping: `stable_id.rfind("::")` then `substr(pos + 2)`.

4. **Phase 4 stiffness constraint limits**: `arm_tip_deflection_limit_m=0.10` and `arm_tip_rotation_limit_rad=0.10`
   (~5.7°) in `EvaluationContext`. For the default 5.5 m outer arm under max thrust (12 kN), Euler-Bernoulli
   gives δ ≈ 1.27 m, so violation is large (~1200%). The optimizer must push wall thickness or reduce arm span.

5. **StructuralNetworkAnalyzer call order (Phase 3)**: Must run AFTER `AttainableControlSetAnalyzer`
   (needs fault trim thrusts as load cases) and BEFORE constraint evaluation (constraint reads
   `structural.min_safety_factor`). The old `StructuralAnalyzer` is no longer called; it is superseded.

6. **`arm_outer_radius` effect on 3D view**: `ArmElement::updateFromParameters()` sets segment
   primitive `padding = r_o_->value`. The viewer's `modelMatrixForInstance()` (segment case) sets
   `diameter = 2.0 * padding` and rotates the unit cylinder by -90° around Z so the cylinder axis
   (normally Y) aligns with the arm axis (X). `PrimitiveMeshFactory::makeUnitSegmentProxy()` returns
   a cylinder mesh (not box). Arms appear as round tubes in the viewer.

7. **`objective_weights` defaults**: `structural`, `packaging`, `structural_safety` have weight 0.0 —
   computed and stored but not included in `combined_objective` unless weight is raised.
   `acs_margin_penalty` has weight 0.10 (name must be `"acs_margin_penalty"` in `objective_weights`
   to match the key used by `ObjectiveAggregator`).

8. **`arm_structural` empty guard**: If an architecture has no `IStructuralBeam` elements,
   `arm_structural` is empty and `min_safety_factor = 0`, causing `arm_yield_failure` to fire
   (conservative fail-safe).

9. **T_max parameter default vs. internal scaling references**: `kBaselineTmax = 12000.0 N` in
   `HexacopterArchitecture.cpp` is the parameter default value (optimizer start point). It is
   separate from the internal reference `baseline_tmax = 7327.0` in `VehicleScalingModel.cpp`
   and `kBaselineMotorTmax = 7327.0` in `Elements.cpp`, which are calibration constants for the
   bending index normalization and motor mass allometric scaling respectively. T_max optimizer
   bounds are [8000, 20000] N.

10. **Constraint ID mismatch risk**: The hard constraints in `ConstraintRegistry` use IDs like
    `failed_hover_gamma` and `fault_allocation_ratio`. Do not confuse with the Stage1Metrics field
    names `fault_thrust` and `fault_alloc`, which are soft objective proxies computed differently.

11. **Power model unification**: Both the `power` soft objective and the powertrain evaluator
    use `r_eff = max_arm_length / 2` as the effective rotor disk radius via `PowertrainEvaluator`.
    `d_prop = 0.40 m` is frozen for cT (yaw torque) only — do not use it for power.
    `Stage1Evaluator` computes `result.stage1.power = result.powertrain.total_power_nominal_w /
    s_reference_power_w`, where `s_reference_power_w` is the static reference architecture power
    from `PowertrainEvaluator` (initialized once at first call). Baseline self-normalizes to 1.0.

12. **`IEnvelopeProvider` capability (Phase 6/7)**: `ElementCapabilities.hpp` defines `LocalAABB`
    (axis-aligned bounding box with `containmentViolation()` and `overlapMagnitude()`) and
    `IEnvelopeProvider::localEnvelope()`. `BodyHullElement`, `BatteryElement`, `CabinEnvelopeElement`,
    `OccupantEnvelopeElement`, `PayloadElement`, and `KeepOutZoneElement` implement it. Containment checks are in
    `ArchitecturePackagingEvaluator`; active constraints are `packaging::battery_in_cabin`,
    `packaging::battery_payload_nonoverlap`, `packaging::occupant_in_cabin`, `packaging::payload_in_cabin`,
    `packaging::rotor_keepout`.

13. **Packaging evaluator separation (Phase 6)**: `ArchitecturePackagingEvaluator` owns all
    packaging analysis. `VehicleScalingModel` no longer touches `model.packaging`.
    `Stage1Evaluator` calls the evaluator after Phase 3 (structural network). The renamed field
    `PackagingReport::minimum_rotor_clearance` (was `minimum_clearance`) is exported as
    `pkg_rotor_clearance_m` in Stage1Metrics. Baseline value ≈ 3.47 m (no overlap).

18. **Packaging element geometry (Step 3 / Tasks 1–3)**:
    - `CabinEnvelopeElement`: zero-mass passenger cabin hull. kCabinHalfX=0.80, kCabinHalfY=0.90, kCabinHalfZ=0.90 m.
      Attached to body "center" at offset (0,0,0). Cabin world z ∈ [−0.90, +0.90].
    - `OccupantEnvelopeElement`: zero-mass seated-occupant keep-in zone. kOccupantHalfX=0.55, kOccupantHalfY=0.45, kOccupantHalfZ=0.60 m.
      Attached to cabin_envelope "center" at offset (0,0,0). Occupant always contained in cabin at default.
    - `BatteryElement`: base attachment offset 0.30 m from body bottom → battery center z=−0.70 m at z_bat_offset=0.
      Battery z ∈ [−0.84, −0.56] ⊂ cabin [−0.90, +0.90] with 0.06 m floor clearance at default.
    - `PayloadElement`: box primitive half-size (0.60, 0.60, 0.50) m. Centered at body center + (x_payload, y_payload, 0).
    - Both `CabinEnvelopeElement` and `OccupantEnvelopeElement` are inserted in `HexacopterArchitecture::rebuildElements()`
      (not in `DefaultHexacopterBuilder`) before the payload element.
    - `KeepOutZoneElement`: zero-mass rotor keep-out cylinder (Task 4, reviewed 2026-05-27).
      r_keepout = r_prop + kKeepOutRadialMargin (0.10 m for mfg tolerance + deflection + safety clearance).
      kKeepOutHalfZ = 0.20 m (blade root flapping + hub protrusion + dynamic allowance).
      Cylinder primitive for visualization. AABB valid because cylinder is rotationally symmetric in XY.
      Checks: occupant envelope, payload, battery (arm/motor/rotor exempt via bonded_overlap).
      Worst-case offending zone and element IDs stored in `PackagingReport` for diagnostics.
      One per motor (keepout_1..keepout_6). Added in `rebuildElements()` / `rebuildAttachments()`.

15. **Battery mass feedback loop**: `BatteryElement.mass_` is now set from `m_bat->value`, so
    adding battery mass increases hover thrust, which increases hover power, which increases energy
    requirement. The battery constraint (`battery_energy_reserve`) stabilises at ≈290 kg minimum
    at baseline geometry (6-min spec; now ≫290 kg at 30-min spec), creating a non-trivial
    optimization gradient for `m_bat`.

16. **Mission time and endurance metric (2026-05-27)**:
    - `mission_time_nominal_min = 30.0 min` (changed from 6 min) — represents a full UAM design
      mission (takeoff + transit hover). The 6-min value made `battery_energy_reserve` trivially
      satisfied at baseline and gave the optimizer no energy signal.
    - `mission_time_emergency_min = 1.0 min` — unchanged; emergency post-fault landing margin.
    - New `Stage1Metrics::bat_achievable_endurance_nom_min` (analysis_only, "min"):
      `E_avail / (P_nom + P_aux) × 60`. Shows actual flyable hover time given current battery and power.
      At baseline: ≈15.3 min (battery_energy_reserve violated since 15.3 < 30 min required).
    - Power model (`BatteryEvaluator`): `E_req = P_nom_total × t_nom + P_fault_total × t_emg` (correct sequential budget).
      C-rate: `c_rate = (max(P_nom, P_fault) + P_aux) / E_avail [1/h]` (voltage cancels). Both verified correct.

16. **MetricRole labeling (Phase 5)**: `MetricRole` enum (`hard_constraint`, `soft_objective`,
    `analysis_only`) is defined in `include/evaluation/MetricRole.hpp`. The `stage1MetricDescriptors()`
    static table maps all ~35 Stage1Metrics fields to their role. `ObjectiveValue.role` is always
    `soft_objective` (set by `ObjectiveAggregator`). `ConstraintResult.metricRole()` returns
    `hard_constraint` when `hard=true` and `active=true`, `analysis_only` when `active=false`.
    JSON output: `objectives[]` and `constraints[]` include `"role"` fields; `metric_descriptors[]`
    array emitted by `evaluationToJson()`. Phase 4 context params (`arm_tip_deflection_limit_m`,
    `arm_tip_rotation_limit_rad`) appear in the `evaluation_context` JSON block.

17. **Active parameter count (Phase 2)**: `m_bat` (active=true) brings the active parameter count
    to 7: Lx, Lyi, Lyo, T_max, arm_outer_radius, arm_wall_thickness, m_bat.
    Check at runtime with `arch.parameters().activeParameters().size()`.

19. **Placement DOF attachments (Task 3)**: Battery and payload attachments use
    `relative_transform` lambdas that read from the architecture at `assemble()` time:
    - Battery: `arch.batteryZOffset()` → translation (0, 0, 0.30 + z_bat_offset) from body bottom
    - Payload: `arch.payloadXOffset()`, `arch.payloadYOffset()` → translation (x, y, 0) from body center
    Lambdas capture no pointers — they use the `arch` argument, so copy-construction is safe.
    Active parameter count rises to **10** with z_bat_offset, x_payload, y_payload.
