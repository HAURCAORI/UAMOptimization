# Action Guide: Extending FrameworkCpp

This document lists **every file that must be changed** for each common extension task.
Follow it in order; skipping a step causes silent omissions in export, console, or optimization.

---

## A. Adding a New Design Parameter

A design parameter is a scalar knob the optimizer can tune (e.g., arm length, tube radius).

### Step 1 — Register in `HexacopterArchitecture`
File: `src/core/HexacopterArchitecture.cpp`
- In `registerDefaultParameters()`: call `parameters_.add({name, id_, unit, description, default, lower, upper, default, active, scale})`.
- In `bindCanonicalParameters()`: `ptr_ = parameters_.find(id_ + "::" + name)`.
- Declare the pointer in `include/core/HexacopterArchitecture.hpp` as `DesignParameter* name_ = nullptr`.

### Step 2 — Thread the pointer to the element that uses it
File: `include/core/DefaultHexacopterBuilder.hpp`
- Add `DesignParameter* new_param = nullptr` to `DefaultHexacopterParameters`.

File: `src/core/DefaultHexacopterBuilder.cpp`
- In `buildElements()` (or `buildAttachments()`): set `params.new_param = architecture.new_param_` and pass it to the element constructor.

### Step 3 — Consume the parameter in the element
File: `include/core/Elements.hpp`
- Add `DesignParameter* new_param_` member and any derived cached values.
- Add the parameter to the constructor signature.

File: `src/core/Elements.cpp`
- In the constructor: assign `new_param_ = new_param` and call `new_param_->addConsumer(id_)`.
- In `rebindParameters()`: re-assign the pointer.
- In `updateFromParameters()`: compute derived geometry from `new_param_->value`.

### Step 4 — Rebuild passes through automatically
`HexacopterArchitecture::rebuildElements()` calls `rebuildAttachments()` which calls
`rebuildAssembly()`. After parameter registration and pointer binding these are all called
through `rebuildAssembly()`. No additional wiring needed here.

### Step 5 — Verify export (nothing to change if done correctly)
- `PagmoProblemAdapter::problem()` iterates `activeParameters()` — new param is picked up automatically.
- `DesignVectorMapper` packs/unpacks all active params automatically.
- `CsvExporter::writeSooParametersCsv` / `writeParetoParametersCsv` use `problem.parameter_ids` — automatic.
- `ComparisonReporter::parametersTable` uses the same — automatic.

**Nothing to change in export code if the parameter is wired in Steps 1–3.**

---

## B. Adding a New Stage 1 Metric

A Stage 1 metric is a scalar performance indicator computed during evaluation
(e.g., structural_safety, efficiency margin).

### Step 1 — Declare the field
File: `include/evaluation/EvaluationResult.hpp`
- Add `double new_metric = 0.0` to `Stage1Metrics`.

### Step 2 — Compute and assign the value
File: `src/evaluation/Stage1Evaluator.cpp`
- After the existing metric computations, set `result.stage1.new_metric = ...`.

### Step 3 — Register as an objective (optional)
File: `include/evaluation/EvaluationContext.hpp`
- Add `{"new_metric", 0.0}` to `objective_weights` initializer (weight 0.0 = inactive by default).

File: `src/evaluation/ObjectiveAggregator.cpp`
- Add `append("new_metric", result.stage1.new_metric)`.

### Step 4 — Export
File: `src/analysis/CsvExporter.cpp`
- In `stage1ToJson()`: add `{"new_metric", m.new_metric}` to the JSON object.
- In `summaryTable()` in `ComparisonReporter.cpp`: add column name to `kColumns` and
  the value in the data loop.
- In `writeParetoCsv()`: add the column header and value if needed for MOO export.

---

## C. Adding a New Constraint

### Step 1 — Register in `HexacopterArchitecture`
File: `src/core/HexacopterArchitecture.cpp`
- In `registerDefaultConstraints()`:
  ```cpp
  constraints_.add({
      "constraint_id", id_,
      ConstraintSense::greater_equal,   // or less_equal
      threshold_value,
      /*active=*/true,
      /*hard=*/true,
      penalty_weight,
      [](const ConstraintEvaluationContext& ctx) {
          // read from ctx.physical_model or ctx.evaluation_context
          Constraint c{"constraint_id", ctx.architecture.id(), ConstraintSense::greater_equal, threshold};
          return c.evaluate(measured_value);
      }
  });
  ```

### Step 2 — Ensure the physical data is populated before constraint evaluation
Constraints run after `Stage1Evaluator::evaluate()`. Any data needed by the lambda
must be set in `PhysicalModel` by that point (see `StructuralAnalyzer` for an example).

### Step 3 — Export (automatic)
`CsvExporter` already serialises all `constraint_results` from `EvaluationResult`.
`ComparisonReporter::summaryTable` shows the worst violated constraint — no change needed.

---

## D. Adding a New Element Type

### Step 1 — Declare the element class
File: `include/core/Elements.hpp`
- Inherit from `BasicSpatialElement` and any capability interfaces needed
  (`IStructuralBeam`, `ILoadReceiver`, `IPropulsionRotor`, etc.).
- Add constructor, parameter pointers, capability method declarations.

### Step 2 — Implement the element
File: `src/core/Elements.cpp`
- Implement constructor (assign params, call `addConsumer`), `rebindParameters()`,
  `updateFromParameters()`, and all capability methods.

### Step 3 — Instantiate in the builder
File: `src/core/DefaultHexacopterBuilder.cpp`
- In `buildElements()` or `buildAttachments()`: create the element and add it to the
  architecture via `architecture.addElement(...)`.

### Step 4 — Register any new parameters
Follow **Section A** for each new parameter the element consumes.

### Step 5 — Register in vcxproj (Visual Studio)
File: `FrameworkCpp.vcxproj`
- Add `<ClCompile Include="src\path\NewElement.cpp" />` under the existing ClCompile items.
- Add `<ClInclude Include="include\path\NewElement.hpp" />` under ClInclude items.

---

## E. Adding a New Physical Model Field

A physical model field stores intermediate physics results (e.g., per-arm structural data).

### Step 1 — Declare the field
File: `include/physics/PhysicsTypes.hpp`
- Add the field to the appropriate struct (`PhysicalModel`, `PropulsionProxy`,
  `StructuralProxy`, etc.). Add a new sub-struct if needed.

### Step 2 — Populate the field
Either in `VehicleScalingModel` (for geometry/mass/propulsion) or in a dedicated
analyzer called from `Stage1Evaluator::evaluate()` (for derived physics like structural analysis).

### Step 3 — Expose in export
File: `src/analysis/CsvExporter.cpp`
- In `physicalModelToJson()`: add the new field to the returned JSON object.

---

## F. Adding a New Output File Format

File: `include/analysis/CsvExporter.hpp` + `src/analysis/CsvExporter.cpp`
- Add a static `write*()` method.
- Call it from the appropriate `runSoo()`, `runMoo()`, or `runCompare()` in `app/main.cpp`.
- Print the result in the console export summary line.
