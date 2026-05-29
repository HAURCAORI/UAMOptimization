# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

This repository is a Stage 1 reduced-order co-design framework for a fault-tolerant hexacopter UAM.
It has two sub-projects:

- **`Framework/`** — MATLAB reference implementation (optimization, evaluation, plotting)
- **`FrameworkCpp/`** — C++ production implementation (pagmo2 optimizer, ACS analysis, SVG plotter, 3D viewer)

---

## Build

**Environment:** Visual Studio 2022 on Windows. Dependencies managed by vcpkg.

```
# Open solution and build in VS:
FrameworkCpp/FrameworkCpp.sln   → Build > Build Solution (Release|x64)

# Run the compiled binary:
x64\Release\FrameworkCpp.exe eval
x64\Release\FrameworkCpp.exe soo --soo-pop 24 --soo-gen 40
x64\Release\FrameworkCpp.exe moo --moo-pop 48 --moo-gen 60
x64\Release\FrameworkCpp.exe compare --output-dir output/
x64\Release\FrameworkCpp.exe calibrate data\example_flight_data.csv
x64\Release\FrameworkCpp.exe mission data\example_mission_profile.json

# One-click demo (eval + mission + calibrate + SOO):
powershell -ExecutionPolicy Bypass -File tools\run_demo.ps1
```

There are no unit tests. Correctness is validated against the MATLAB reference (`Framework/main_test.m`) and by inspecting SVG/JSON outputs.

**Critical vcxproj rule:** Every new `.cpp` or `.hpp` under `FrameworkCpp/` must be added to `FrameworkCpp.vcxproj` in both `<ClCompile>` and `<ClInclude>` item groups. Missing this causes silent `LNK2019` at link time.

---

## Mandatory reads before modifying FrameworkCpp

Before touching any file under `FrameworkCpp/`:

```
FrameworkCpp/docs/CODEBASE.md   ← authoritative architecture reference (read first, always)
FrameworkCpp/docs/Action.md     ← checklists for adding params/constraints/metrics/elements
```

For the optimization problem specifically:
```
FrameworkCpp/docs/Optimization.md                               ← evaluation pipeline
FrameworkCpp/docs/Optimization_Constraints_And_Design_Variables.md  ← all variables and constraints
FrameworkCpp/docs/Phase2_Powertrain_Battery.md                  ← power/battery model
```

After every session that changes architecture, update `CODEBASE.md` to reflect what changed.

---

## Project context

### Vehicle
Fixed-topology hexacopter (6 rotors, positions fixed). NED convention: z-down, upward thrust = negative Fz.

Motor world positions (z=0 for all):

| Motor | x | y |
|---|---|---|
| 0 | +Lx | −Lyi |
| 1 | +Lx | +Lyi |
| 2 | 0 | −Lyo |
| 3 | 0 | +Lyo |
| 4 | −Lx | −Lyi |
| 5 | −Lx | +Lyi |

B-matrix row 0 = [−1, −1, −1, −1, −1, −1] (all rotors push upward in NED).

### Implementation phases (current status)

- **Phase 1 (done):** ACS analysis — B matrix, trim LP, directional margins, volume metrics, per-axis reserves, soft margin objective, named fault constraint.
- **Phase 2 (done):** Powertrain limits (actuator-disk power model) and battery sizing (`m_bat` design variable, `battery_energy_reserve` + `battery_crate_limit` constraints).
- **Phase 3 (done):** `StructuralNetworkAnalyzer` — multi-member network with 8 load cases (max_thrust, nominal_hover, fault_0–5), von Mises stress, Euler-Bernoulli tip deflection.
- **Phase 4 (done):** Stiffness / deflection hard constraints (`arm_tip_deflection`, `arm_tip_rotation`; limits 0.10 m and 0.10 rad).
- **Phase 5 (done):** `MetricRole` enum — every metric labeled `hard_constraint`, `soft_objective`, or `analysis_only`. JSON output includes `"role"` field and `metric_descriptors[]` array.
- **Phase 6 (done):** `ArchitecturePackagingEvaluator` — rotor clearance, payload/battery/occupant containment checks, rotor keep-out cylinder, CG symmetry window.
- **Mission subsystem (done):** Multi-segment UAM mission (hover, cruise, climb, descent, emergency) via `MissionEvaluator` + `CruisePowerModel`. JSON profiles in `data/`.
- **Calibration subsystem (done):** `Calibrator` (Nelder–Mead) identifies physics params from flight-log CSVs.

### Active design parameters (optimizer sees these)

| Parameter | Unit | Default | Bounds |
|---|---|---|---|
| `Lx` | m | 2.65 | [2.0, 5.0] |
| `Lyi` | m | 2.65 | [2.0, 5.0] |
| `Lyo` | m | 5.50 | [2.5, 9.0] |
| `T_max` | N | 12000 | [8000, 20000] |
| `arm_outer_radius` | m | 0.08 | [0.02, 0.15] |
| `arm_wall_thickness` | m | 0.005 | [0.001, 0.020] |
| `m_bat` | kg | 400 | [100, 1000] |
| `z_bat_offset` | m | 0.0 | [−0.20, +0.50] |
| `x_payload` | m | 0.0 | [−0.30, +0.30] |
| `y_payload` | m | 0.0 | [−0.40, +0.40] |

Fixed (inactive): `cT=0.03`, `d_prop=0.40 m`, `m_payload=800 kg`.

### ACS correctness invariant (never break)

```
margin(d, u_req) = h_U(d) − d' * u_req    ← positive = feasible
h_U(d)           = Σ_j max(0, d'·b_j) · f_max_j
```

- Origin `u=0`: all margins ≥ 0; thrust direction [+1,0,0,0] gives margin = 0 (boundary, not infeasible).
- Hover `u=[−mg,0,0,0]`: all margins > 0.
- Wrong sign `d'·u_req − h_U(d)` always gives ≤ 0 at origin — never use this form.

---

## MATLAB ↔ C++ alignment rules

When MATLAB and C++ results differ, MATLAB wins unless there is a documented reason.

Key alignment fixes that must not be reverted:
1. **Rotor-only clearance:** packaging check uses rotor disks only, not full element pairs.
2. **T_max bounds:** [8000, 20000] N; default 12000 N; Lx/Lyi lower bound 2.0 m; `m_payload` baseline 800 kg.
3. **cT frozen:** 0.03 (active=false). MATLAB does not optimize cT.
4. **L_ref for fault_alloc:** `max(Lx, Lyi, Lyo)`, not `arm_span / 6`.
5. **Power model disk area:** `r_eff = max_arm_length / 2`. Do not use `d_prop` for power — it is frozen for yaw-torque cT only.

---

## Code architecture

### Evaluation pipeline (Stage1Evaluator.cpp)

```
VehicleScalingModel          → PhysicalModel (mass, COM, inertia, allocation matrix)
AttainableControlSetAnalyzer → AcsResult     (trim, margins, PFWAR/FII/WCFR, fault trims)
PowertrainEvaluator          → PowertrainResult  (per-motor power, utilization)
BatteryEvaluator             → BatteryResult     (E_avail, E_req, reserve, C-rate)
  [if context.mission_profile set: MissionEvaluator → BatteryEvaluator::evaluateWithMission()]
StructuralNetworkAnalyzer    → PhysicalModel.structural.network_*  (SF, deflection, rotation)
ArchitecturePackagingEvaluator → PhysicalModel.packaging (clearance, containment, keepout)
ConstraintRegistry::evaluate → constraint_results
ObjectiveAggregator          → combined_objective
```

Call order matters: ACS runs before structural (fault trims are load cases); packaging runs after structural.

### Element hierarchy

`SpatialElement` → `BasicSpatialElement` is the base for all concrete elements. Elements opt into framework subsystems via capability interfaces in `ElementCapabilities.hpp`:

- `IPropulsionRotor` — rotor disk, enters B-matrix, rotor clearance check
- `IStructuralBeam` — hollow tube cross-section; required for StructuralNetworkAnalyzer
- `ILoadReceiver` — accepts thrust loads from the structural analyzer
- `IMotorMassContributor` — concentrated mass affecting inertia
- `IPayloadMassContributor` — payload mass reporting
- `IEnergyStorage` — exposes `batteryMass()` for BatteryEvaluator
- `IEnvelopeProvider` — exposes `LocalAABB` for packaging/containment checks

Current concrete elements: `BodyHullElement`, `BodyFrameElement`, `ArmElement`, `MotorElement`, `RotorElement`, `BatteryElement`, `PayloadElement`, `CabinEnvelopeElement`, `OccupantEnvelopeElement`, `KeepOutZoneElement`.

### Parameter and constraint registration

Parameters: `HexacopterArchitecture::registerDefaultParameters()` → `bindCanonicalParameters()` → thread through `DefaultHexacopterParameters` → element constructors.

Constraints: two registration sites:
- Architecture-level: `HexacopterArchitecture::registerDefaultConstraints()`
- Element-level: `SpatialElement::registerConstraints()`, called via `registerElementConstraints()`

**Do not capture raw `DesignParameter*` in constraint lambdas** — elements are cloned and rebound, making captured pointers stale.

### Optimization loop

`PagmoProblemAdapter` is the pagmo integration point. It clones the architecture, unpacks normalized parameters via `DesignVectorMapper`, runs Stage1Evaluator, and adds weighted penalties for violated hard constraints. Active parameters normalized to [0, 1]; physical bounds stored in `OptimizationProblem` (not normalized).

SOO: CMA-ES, default pop=24, gen=40. MOO: NSGA-II, default pop=48, gen=60. Both runners track best raw and best feasible separately.

### 3D visualization

`ArchitectureSceneBuilder` converts the assembled element list to `PrimitiveInstance` vector. Each instance carries:
- `primitive_type`, `world_transform`, `dimensions`, `padding`
- `color` (per element type)
- `wireframe` (bool) — disk primitives are wireframe by default; set this flag to make any primitive render as a wireframe

`PrimitiveMeshFactory` generates unit meshes per primitive kind. Segment primitives render as cylinders rotated −90° around Z so the cylinder axis aligns with the arm axis (X). Colors and wireframe mode are set per element type in `ArchitectureSceneBuilder::colorForElementType()` and `defaultWireframeForPrimitive()`.

### Mission and calibration subsystems

**Mission:** `MissionProfile` (JSON loader in `src/mission/MissionProfile.cpp`) → `MissionEvaluator` (per-segment energy, uses ACS faulted trims for emergency legs and `CruisePowerModel` for forward-flight legs) → `BatteryEvaluator::evaluateWithMission()`. Activated when `EvaluationContext::mission_profile` is non-null.

**Calibration:** `Calibrator` (Nelder–Mead, box-projected) minimizes mean-squared relative power residual over `FlightDataPoint` records. Hover uses actuator-disk; cruise uses `CruisePowerModel`. `applyToContext()` copies fitted params into `EvaluationContext`.

Key context fields added by these subsystems:
- `mission_profile` — `shared_ptr<const MissionProfile>`; null = legacy fixed-time hover
- `parasite_drag_area_m2` — Cd·A_ref [m²] for cruise power (default 0.6)

---

## Code style rules

- Comments only for WHY (hidden constraint, workaround, invariant). Never explain what the code does.
- No trailing summaries in responses.
- Windows `max`/`min` macro conflict: use `(std::numeric_limits<double>::max)()`. Add `NOMINMAX` to preprocessor definitions for new configurations.
- All 4 vcxproj configurations must stay in sync (Win32/x64 × Debug/Release).
- Use `$(VcpkgTriplet)` in vcxproj include paths — never hardcode `x64-windows`.

---

## Key file locations

| File | Purpose |
|---|---|
| `FrameworkCpp/docs/CODEBASE.md` | Architecture reference — read first |
| `FrameworkCpp/docs/Action.md` | Checklists for adding params/constraints/metrics/elements |
| `FrameworkCpp/FrameworkCpp.vcxproj` | VS project file — register every new .cpp/.hpp here |
| `FrameworkCpp/include/evaluation/EvaluationContext.hpp` | Objective weights, constraint thresholds, context flags |
| `FrameworkCpp/include/evaluation/EvaluationResult.hpp` | Stage1Metrics struct (all ~40 fields) |
| `FrameworkCpp/src/evaluation/Stage1Evaluator.cpp` | Full evaluation pipeline |
| `FrameworkCpp/src/core/HexacopterArchitecture.cpp` | Parameter registration and constraint registration |
| `FrameworkCpp/src/core/DefaultHexacopterBuilder.cpp` | Element construction and attachment graph |
| `FrameworkCpp/include/core/Elements.hpp` | All concrete element declarations |
| `FrameworkCpp/src/physics/ArchitecturePackagingEvaluator.cpp` | Packaging constraint computation |
| `FrameworkCpp/visualization/src/visualization/ArchitectureSceneBuilder.cpp` | Element-type → color/wireframe mapping |
| `FrameworkCpp/app/main.cpp` | CLI modes: eval/soo/moo/compare/visualize/calibrate/mission |
| `Framework/main_test.m` | MATLAB ACS verification tests (T1–T6 + comparison figures) |

---

## Memory system

Persistent notes across sessions live in:
```
C:\Users\user\.claude\projects\C--local-project-UAMOptimization\memory\
```

Read `MEMORY.md` index there for the current state. Key files:
- `project_cpp_fixes.md` — MATLAB-alignment bugs fixed + Phase 1 ACS fixes
- `project_remaining_issues.md` — open bugs/gaps
- `project_vehicle_model.md` — MATLAB vehicle_model.m calibration constants
- `project_mission_calibration.md` — mission/calibration subsystem merge notes (2026-05-29)
