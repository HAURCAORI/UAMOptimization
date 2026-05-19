# FrameworkCpp Action Guide

This guide is the short "what do I have to touch?" reference for common codebase changes.

Use it together with:

- `docs/Manual.md` for element and attachment mechanics
- `docs/Optimization.md` for the evaluation and optimizer flow
- `docs/Optimization_Constraints_And_Design_Variables.md` for the current optimization variables and hard constraints

## Rule: register new files in the Visual Studio project

Any new `.cpp` or `.hpp` file under `FrameworkCpp/` must be added to `FrameworkCpp.vcxproj`.

Add source files under the existing `ClCompile` item group:

```xml
<ClCompile Include="src\path\to\NewFile.cpp" />
```

Add headers under the existing `ClInclude` item group:

```xml
<ClInclude Include="include\path\to\NewFile.hpp" />
```

If you skip this, the project may compile partially and then fail at link time with unresolved symbols.

## Add a design parameter

1. Register it in `src/core/HexacopterArchitecture.cpp` inside `registerDefaultParameters()`.
2. Bind a member pointer in `bindCanonicalParameters()`.
3. Declare the pointer in `include/core/HexacopterArchitecture.hpp`.
4. Thread the pointer through `DefaultHexacopterParameters` in [DefaultHexacopterBuilder.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/core/DefaultHexacopterBuilder.hpp:1).
5. Pass it into the element constructor in [DefaultHexacopterBuilder.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/core/DefaultHexacopterBuilder.cpp:1).
6. Store, rebind, and consume it in the element class in `include/core/Elements.hpp` and `src/core/Elements.cpp`.

Notes:

- Set `active=true` only if the parameter should enter the pagmo design vector.
- Export wiring is mostly automatic once the parameter is active. `PagmoProblemAdapter`, `DesignVectorMapper`, `CsvExporter`, and `ComparisonReporter` already iterate active parameters.

## Add a Stage 1 metric

1. Add the field to `Stage1Metrics` in [EvaluationResult.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/evaluation/EvaluationResult.hpp:1).
2. Compute it in [Stage1Evaluator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/evaluation/Stage1Evaluator.cpp:1).
3. If it should be an objective, add a default entry in [EvaluationContext.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/evaluation/EvaluationContext.hpp:1).
4. Append it in [ObjectiveAggregator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/evaluation/ObjectiveAggregator.cpp:1).
5. Export it in `CsvExporter.cpp` if it should appear in JSON or CSV summaries.
6. Add it to `ComparisonReporter.cpp` if it belongs in the comparison table.

## Add a hard constraint

There are two common places:

- System-level constraints: `HexacopterArchitecture::registerDefaultConstraints()`
- Element-level constraints: `SpatialElement::registerConstraints()`

Checklist:

1. Decide where the constraint belongs.
2. Register it with a stable ID, sense, threshold, `active=true`, `hard=true`, and a penalty weight.
3. Read current values through `ConstraintEvaluationContext`.
4. Make sure the required physical or Stage 1 data is populated before constraint evaluation.

Notes:

- Constraints are already exported through `EvaluationResult.constraint_results`.
- `PagmoProblemAdapter` applies penalties automatically using the registered penalty weights.

## Add a new element type

1. Declare the class in a header, usually under `include/core/`.
2. Implement it in a `.cpp` file.
3. Derive from `BasicSpatialElement`.
4. Implement:
   - `clone()`
   - `registerParameters()`
   - `rebindParameters()`
   - `registerConstraints()`
   - `updateFromParameters()`
5. Add any capability interfaces needed, such as `IPropulsionRotor`, `IStructuralBeam`, `IMotorMassContributor`, `IPayloadMassContributor`, or `IEnergyStorage`.
6. Instantiate it in `DefaultHexacopterBuilder` or add it programmatically to `HexacopterArchitecture`.
7. Register the new files in `FrameworkCpp.vcxproj`.

## Add a physical-model field

1. Add the field to the relevant struct in [PhysicsTypes.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/physics/PhysicsTypes.hpp:1).
2. Populate it in:
   - `VehicleScalingModel` for geometry / mass / propulsion level data, or
   - a dedicated analyzer invoked by `Stage1Evaluator`
3. Export it in `CsvExporter.cpp` if it should appear in saved artifacts.

## Add a new output artifact

1. Add a `CsvExporter::write*()` method in `include/analysis/CsvExporter.hpp` and `src/analysis/CsvExporter.cpp`.
2. Call it from the relevant command path in [app/main.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/app/main.cpp:1).
3. Add a console summary line if the output should be advertised to the user.

## Add a new SOO or MOO objective

1. Make sure the metric exists in `Stage1Metrics`.
2. Add it to the default `objective_weights` list if weighted-sum SOO should know about it.
3. Append it in `ObjectiveAggregator.cpp`.
4. If MOO should expose it, include its name in `MooRunConfig::objective_names`.
5. If CLI defaults should use it, update `main.cpp`.

## Before you stop

Run through this quick sanity list:

1. New files are in `FrameworkCpp.vcxproj`.
2. New parameters are rebound after cloning.
3. New constraints read from current context data, not stale pointers.
4. New metrics are exported if they are intended for analysis.
5. SOO / MOO defaults still make sense after the change.
