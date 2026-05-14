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
│   │   │                           IMotorMassContributor, IPayloadMassContributor
│   │   ├── Elements.hpp            Concrete elements: BodyElement, ArmElement, MotorElement,
│   │   │                           PropellerElement, PayloadElement
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
│       ├── Material.hpp            Material struct + Materials::Al7075/CFRP/Steel304
│       ├── PhysicsTypes.hpp        PhysicalModel, PropulsionProxy, StructuralProxy,
│       │                           ArmStructuralResult, PackagingResult
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
    └── CODEBASE.md                 This file
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
1. `VehicleScalingModel::evaluate()` → fills `PhysicalModel` (mass, propulsion, packaging)
2. `StructuralAnalyzer::analyze()` → fills `arm_structural`, `structural.min_safety_factor`
3. Computes `Stage1Metrics` from `PhysicalModel`
4. Calls `ConstraintRegistry` evaluators → fills `constraint_results`
5. Calls `ObjectiveAggregator::aggregate()` → fills `objectives`, `combined_objective`

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
                    ├── VehicleScalingModel → PhysicalModel (mass, thrust, geometry)
                    ├── StructuralAnalyzer  → arm_structural, min_safety_factor
                    ├── ConstraintRegistry  → constraint_results
                    └── ObjectiveAggregator → combined_objective
                            └──► EvaluationResult

SooRunner / MooRunner
    ├── PagmoProblemAdapter::problem() → OptimizationProblem [PHYSICAL bounds]
    ├── pagmo runs fitness() in normalized space
    └── SooRunResult / MooRunResult
            └──► CsvExporter, ComparisonReporter
                    └── denormalizeParameter(problem, i, x) → physical values
```

---

## Current Active Design Parameters (as of last implementation)

| Stable ID suffix | Unit | Default | Bounds | Active |
|---|---|---|---|---|
| `Lx` | m | 2.0 | [1.0, 4.0] | true |
| `Lyi` | m | 4.0 | [1.0, 6.0] | true |
| `Lyo` | m | 5.5 | [2.0, 8.0] | true |
| `Tmax` | N | 8000 | [5000, 15000] | true |
| `cT` | m | 0.016 | [0.01, 0.03] | false |
| `dprop` | m | 1.2 | [0.8, 2.0] | false |
| `arm_outer_radius` | m | 0.08 | [0.02, 0.15] | false |
| `arm_wall_thickness` | m | 0.005 | [0.001, 0.020] | false |

---

## Current Stage 1 Metrics (Stage1Metrics struct)

| Field | Description |
|---|---|
| `mass` | Total vehicle mass (kg) |
| `power` | Hover power (W) |
| `fault_thrust` | Worst-case single-motor-failure thrust margin |
| `fault_alloc` | Allocation feasibility metric |
| `hover_nom` | Nominal hover margin |
| `structural` | Structural bending index proxy (legacy) |
| `packaging` | Geometry packaging penalty |
| `structural_safety` | `min_safety_factor / minimum_arm_safety_factor` (≤1 = failing) |

---

## Current Hard Constraints

| ID | Sense | Threshold | Penalty |
|---|---|---|---|
| `hover_thrust` | ≥ | 1.0 | 500 |
| `fault_thrust_margin` | ≥ | 0.0 | 500 |
| `arm_yield_failure` | ≥ | `context.minimum_arm_safety_factor` (1.5) | 2000 |

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

6. **`objective_weights` default**: `structural_safety` has weight 0.0 — it is computed and stored
   but does not affect `combined_objective` unless the user explicitly raises the weight.

7. **`arm_structural` empty guard**: If an architecture has no `IStructuralBeam` elements,
   `arm_structural` is empty and `min_safety_factor = 0`, causing `arm_yield_failure` to fire
   (conservative fail-safe).
