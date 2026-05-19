# FrameworkCpp Optimization Overview

This is the top-level optimization reference for `FrameworkCpp`.

It explains the flow of data and where each optimization concern lives in the codebase. For the current variable and hard-constraint list, use `docs/Optimization_Constraints_And_Design_Variables.md`.

## Document map

- `Optimization.md`: overall evaluation and optimizer flow
- `Optimization_Constraints_And_Design_Variables.md`: current active variables, inactive registered parameters, and hard constraints
- `Phase2_Powertrain_Battery.md`: powertrain and battery model details
- `Action.md`: implementation checklist for extending the optimization stack

## Architecture-to-objective flow

The optimization loop is:

```text
active parameters
  -> DesignVectorMapper / BoundsBuilder
  -> candidate HexacopterArchitecture
  -> VehicleScalingModel
  -> StructuralAnalyzer
  -> AttainableControlSetAnalyzer
  -> PowertrainEvaluator
  -> BatteryEvaluator
  -> hard constraint evaluation
  -> ObjectiveAggregator
  -> pagmo fitness
```

Main implementation points:

- parameter registration: [HexacopterArchitecture.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/core/HexacopterArchitecture.cpp:1)
- optimizer adapter: [PagmoProblemAdapter.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/optimization/PagmoProblemAdapter.cpp:1)
- Stage 1 evaluation: [Stage1Evaluator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/evaluation/Stage1Evaluator.cpp:1)
- objective weighting: [ObjectiveAggregator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/evaluation/ObjectiveAggregator.cpp:1)

## Design-vector handling

Only active parameters participate in optimization.

`DesignVectorMapper`:

- packs active parameters into a normalized vector
- unpacks normalized values back into the cloned architecture
- calls `architecture.updateFromParameters()` after assignment

`BoundsBuilder`:

- returns normalized lower and upper bounds for pagmo

`PagmoProblemAdapter::problem()`:

- exports physical bounds and parameter IDs for reporting and CSV/JSON outputs

## Evaluation stages

### 1. Vehicle model

`VehicleScalingModel` builds the physical model from the current architecture:

- total mass
- COM and inertia
- allocation matrix and faulted allocation matrices
- packaging clearance metrics
- structural aggregate metrics such as arm span and normalized bending index

### 2. Structural analysis

`StructuralAnalyzer` evaluates arm section properties and loads to compute:

- per-arm bending stress
- per-arm safety factor
- minimum arm safety factor across the vehicle

This is why `ArmElement` implements `IStructuralBeam` and `ILoadReceiver`.

### 3. ACS / controllability analysis

`AttainableControlSetAnalyzer` evaluates:

- nominal hover trim
- single-fault hover trims
- ACS retention and margin metrics
- the current hover-feasibility margin used by hard constraints

### 4. Powertrain analysis

`PowertrainEvaluator` computes:

- nominal total electrical hover power
- worst-fault total electrical hover power
- worst nominal thrust utilization
- worst nominal power utilization

This phase uses effective disk area derived from arm geometry, not directly from `d_prop`.

### 5. Battery analysis

`BatteryEvaluator` computes:

- available battery energy
- nominal and emergency mission energy demand
- energy reserve fraction
- peak C-rate
- battery mass fraction

These feed the Phase 2 hard constraints.

### 6. Hard constraint evaluation

The evaluator builds `ConstraintEvaluationContext` and runs all registered constraints from:

- `HexacopterArchitecture::registerDefaultConstraints()`
- each element's `registerConstraints()`

Results are stored in `EvaluationResult.constraint_results`.

### 7. Objective aggregation

`ObjectiveAggregator` selects only objectives with positive weights from `EvaluationContext.objective_weights`, then computes:

```text
combined_objective = sum(weight_i * value_i) / sum(weight_i)
```

Inactive objectives can still be computed and exported without affecting the weighted-sum SOO objective.

## Feasibility

A result is feasible only if:

1. nominal hover trim is feasible
2. all-fault hover is feasible when `require_all_fault_acs_feasible` is enabled
3. every active hard constraint is feasible

Infeasible points are still evaluated, exported, and penalized so pagmo can search with a gradient-like signal through violation magnitudes.

## Pagmo integration

`PagmoProblemAdapter` is the single integration point with pagmo.

Responsibilities:

- clone the architecture
- unpack normalized variables
- run Stage 1 evaluation
- expose bounds
- return either:
  - a single weighted-sum fitness for SOO, or
  - an objective vector for MOO
- add weighted hard-constraint penalties

Penalty behavior:

- each violated active hard constraint contributes `penalty_weight * violation`
- an additional infeasibility constant is applied when the result is not feasible

## SOO and MOO defaults

### SOO

Implementation: [SooRunner.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/optimization/SooRunner.cpp:1)

- algorithm: CMA-ES
- objective name: `combined`
- default population: 24
- default generations: 40
- default seed: 42

Outputs:

- baseline result
- best raw result
- best feasible result if one exists
- parameter and comparison exports

### MOO

Implementation: [MooRunner.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/optimization/MooRunner.cpp:1)

- algorithm: NSGA-II
- default population: 32
- default generations: 60
- default seed: 42
- config default objective set in code: `mass`, `power`, `fault_thrust`, `fault_alloc`, `hover_nom`
- CLI currently uses: `mass`, `power`, `fault_alloc`

Outputs:

- full population
- feasible sub-population
- Pareto CSV / JSON exports
- knee-point analysis through `ParetoAnalyzer`

## Export and reporting

Key export points:

- `CsvExporter::writeSooComparisonCsv`
- `CsvExporter::writeSooParametersCsv`
- `CsvExporter::writeParetoCsv`
- `CsvExporter::writeParetoParametersCsv`
- `CsvExporter::writeSooJson`
- `CsvExporter::writeMooJson`

Key reporting points:

- `ComparisonReporter::summarize(...)`
- `ComparisonReporter::summaryTable(...)`
- `ComparisonReporter::parametersTable(...)`
- `ParetoAnalyzer::summarize(...)`

## What changed relative to older docs

Older optimization notes in this folder had drift in several areas. Current code now includes:

- active structural variables: `arm_outer_radius`, `arm_wall_thickness`
- active battery variable: `m_bat`
- structural hard constraint: `arm_yield_failure`
- battery hard constraints: `battery_energy_reserve`, `battery_crate_limit`
- ACS hard constraints: `all_faults_hover_feasible`, `fault_directional_margin`
- additional objective slots: `structural_safety`, `acs_margin`

Use the detailed constraint/variable file as the canonical current list rather than older copied tables.
