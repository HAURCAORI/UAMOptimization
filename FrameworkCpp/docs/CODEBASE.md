# CODEBASE.md — FrameworkCpp Architecture Reference

Read this file at the start of every implementation session before modifying any code.
Update the relevant section after completing an implementation.

---

## Directory Layout

```
FrameworkCpp/
├── app/
│   └── main.cpp                    CLI entry point (eval/soo/moo/compare/visualize modes)
├── include/
│   ├── analysis/
│   │   ├── AcsPlotter.hpp          SVG-based ACS visualization (no external deps)
│   │   ├── ComparisonReporter.hpp  Console summary and table helpers
│   │   ├── CsvExporter.hpp         All file export declarations
│   │   └── ParetoAnalyzer.hpp      Knee point and dominance analysis
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
│   │   ├── Elements.hpp            Concrete elements: BodyElement, ArmElement, MotorElement,
│   │   │                           PropellerElement, PayloadElement, BatteryElement (Phase 2)
│   │   ├── GeometryPrimitive.hpp   GeometryPrimitive struct (AABB, sphere, etc.)
│   │   ├── HexacopterArchitecture.hpp  Root object; owns registry, elements, builder
│   │   ├── ParameterRegistry.hpp   Registry for all DesignParameters
│   │   ├── SpatialElement.hpp      SpatialElement + BasicSpatialElement base classes
│   │   └── Timestamp.hpp           currentTimestamp() helper
│   ├── evaluation/
│   │   ├── ArchitectureEvaluator.hpp  Top-level evaluation entry point
│   │   ├── EvaluationContext.hpp   Weights, material, constraint thresholds, seeds
│   │   ├── EvaluationResult.hpp    Stage1Metrics, EvaluationResult, ConstraintResult, ObjectiveValue
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
│       │                           ArmStructuralResult, PackagingResult,
│       │                           PowertrainResult, BatteryResult (Phase 2)
│       ├── PowertrainEvaluator.hpp Phase 2: per-motor electrical power (actuator-disk model)
│       ├── PrimitiveDistance.hpp   Geometry collision/clearance helpers
│       ├── StructuralAnalyzer.hpp  Cantilever beam safety factor analysis
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
| `IStructuralBeam` | `outerRadius()`, `innerRadius()`, `crossSectionArea()`, `secondMomentOfArea()` | ArmElement |
| `ILoadReceiver` | `clearLoads()`, `addLoad()`, `loads()` | ArmElement |
| `IMotorMassContributor` | `motorMass()` | MotorElement |
| `IPayloadMassContributor` | `payloadMass()` | PayloadElement |

### `Stage1Evaluator` (evaluation)
Evaluation pipeline in order:
1. `VehicleScalingModel::evaluate()` → fills `PhysicalModel` (mass now includes m_bat, propulsion, packaging)
2. `StructuralAnalyzer::analyze()` → fills `arm_structural`, `structural.min_safety_factor`
3. `AttainableControlSetAnalyzer::analyze()` → fills `AcsResult` (trim, directional margins,
   volume metrics, per-axis reserves, fault degradation ratio)
4. **`PowertrainEvaluator::evaluate()`** (Phase 2) → fills `PowertrainResult` (motor power [W], utilizations)
5. **`BatteryEvaluator::evaluate()`** (Phase 2) → fills `BatteryResult` (energy [Wh], reserve fraction, C-rate)
6. Computes `Stage1Metrics` from `PhysicalModel` + `AcsResult` + `PowertrainResult` + `BatteryResult`
7. Calls `ConstraintRegistry` evaluators → fills `constraint_results`
8. Appends `acs::all_faults_hover_feasible` as a named hard constraint to `constraint_results`
9. Calls `ObjectiveAggregator::aggregate()` → fills `objectives`, `combined_objective`

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
                    ├── VehicleScalingModel          → PhysicalModel (mass incl. m_bat, thrust, geometry)
                    ├── StructuralAnalyzer            → arm_structural, min_safety_factor
                    ├── AttainableControlSetAnalyzer  → AcsResult (trim, margins, reserves)
                    ├── PowertrainEvaluator [Phase 2] → PowertrainResult (motor power [W])
                    ├── BatteryEvaluator    [Phase 2] → BatteryResult (energy [Wh], reserve, C-rate)
                    ├── ConstraintRegistry            → constraint_results (arch constraints)
                    ├── [Stage1Evaluator]             → appends acs::all_faults_hover_feasible
                    └── ObjectiveAggregator           → combined_objective
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

Active parameter count for optimizer: **7**
(Lx, Lyi, Lyo, T_max, arm_outer_radius, arm_wall_thickness, m_bat).

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
| `arm_yield_failure` | ≥ | 1.5 (configurable) | 2000 | `min_safety_factor` from StructuralAnalyzer |
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

4. **StructuralAnalyzer call order**: Must run AFTER `VehicleScalingModel` (needs `thrust_max`, `gravity`)
   and BEFORE constraint evaluation (constraint reads `structural.min_safety_factor`).

5. **`arm_outer_radius` effect on 3D view**: `ArmElement::updateFromParameters()` sets segment
   primitive `padding = r_o_->value`. The viewer's `modelMatrixForInstance()` (segment case) sets
   `diameter = 2.0 * padding` and rotates the unit cylinder by -90° around Z so the cylinder axis
   (normally Y) aligns with the arm axis (X). `PrimitiveMeshFactory::makeUnitSegmentProxy()` returns
   a cylinder mesh (not box). Arms appear as round tubes in the viewer.

6. **`objective_weights` defaults**: `structural`, `packaging`, `structural_safety` have weight 0.0 —
   computed and stored but not included in `combined_objective` unless weight is raised.
   `acs_margin` has weight 0.10 and is always active as a soft penalty term.

7. **`arm_structural` empty guard**: If an architecture has no `IStructuralBeam` elements,
   `arm_structural` is empty and `min_safety_factor = 0`, causing `arm_yield_failure` to fire
   (conservative fail-safe).

8. **Baseline T_max vs bounds**: `kBaselineTmax ≈ 7327 N` is below the optimizer lower bound of
   8000 N. The pagmo optimizer starts from a population seeded within [8000, 16000], not from the
   baseline. The baseline evaluation uses the raw constant; the SOO/MOO optimization does not.

9. **Constraint ID mismatch risk**: The hard constraints in `ConstraintRegistry` use IDs like
   `failed_hover_gamma` and `fault_allocation_ratio`. Do not confuse with the Stage1Metrics field
   names `fault_thrust` and `fault_alloc`, which are soft objective proxies computed differently.

10. **Phase 2 power model uses arm geometry, not d_prop**: `PowertrainEvaluator` sets
    `r_eff = max_arm_length / 2` as the effective rotor disk radius. `d_prop = 0.40 m` is frozen
    for cT (yaw torque) only. Using d_prop for power would give >100× too much power at this
    vehicle mass. See `docs/Phase2_Powertrain_Battery.md`.

11. **Battery mass feedback loop**: `BatteryElement.mass_` is now set from `m_bat->value`, so
    adding battery mass increases hover thrust, which increases hover power, which increases energy
    requirement. The battery constraint (`battery_energy_reserve`) stabilises at ≈290 kg minimum
    at baseline geometry, creating a non-trivial optimization gradient for `m_bat`.

12. **Active parameter count changed (Phase 2)**: Adding `m_bat` (active=true) increases active
    parameters from 6 to 7. Any code that assumes a fixed active-parameter count must be updated.
    Check with `arch.parameters().activeParameters().size()` at runtime.
