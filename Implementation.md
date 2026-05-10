# Codex implementation instructions

## Project title
Fault-tolerant hexacopter architecture co-design using simplified spatial element modeling

## Implementation scope for the current phase
This document defines the **current C++ implementation scope only**.

The current implementation scope is:
- preserve the useful numerical logic from the existing MATLAB framework,
- refactor the project into a clean layered C++ architecture,
- implement **Stage 1 only**,
- use **pagmo2** as the optimization backend,
- postpone all Stage 2, long simulation, and mission-validation logic until a later project phase.

### Explicitly out of scope for the current implementation
Do **not** implement the following in this phase:
- Stage 2 evaluation,
- full nonlinear time-domain mission simulation,
- figure-8 mission tracking evaluation,
- hover recovery simulation,
- emergency landing trajectory optimization,
- full FEM,
- GUI or visualization-heavy tooling.

If MATLAB files contain Stage 2 logic, keep them as reference only and do not port them now.

---

## Migration objective
Reimplement the existing MATLAB framework in C++ **without discarding the useful numerical logic**.

The new project must preserve:
- the overall optimization workflow,
- the baseline hexacopter parameterization,
- the reduced-order mass, inertia, and control-allocation logic,
- the Stage 1 screening concept,
- the comparison and reporting flow.

At the same time, the new project must **not preserve the current responsibility layout**. The MATLAB code is useful as a numerical reference, but the structure is too tightly coupled for a new extensible project.

The C++ version must therefore follow a clean layered architecture.

---

## Hard architectural rule
Preserve **formulas and workflow**, but refactor **ownership and responsibilities**.

### Required layers
- `core/` for geometry, parameters, constraints, and architecture objects,
- `physics/` for mass, inertia, control allocation, structural proxies,
- `evaluation/` for Stage 1 metrics and objective aggregation,
- `optimization/` for design-vector mapping and optimizer wrappers,
- `analysis/` for comparison tables and reporting,
- `app/` for runnable examples.

This layer split is mandatory.

---

## Primary design principles

### 1. Do not rewrite the math blindly
First read the MATLAB code and keep its proven numerical logic where it is still useful.

### 2. Do not preserve MATLAB responsibility layout
Many MATLAB files mix:
- configuration,
- model definitions,
- physics computation,
- objective aggregation,
- optimizer callbacks,
- reporting.

The C++ version must separate these concerns.

### 3. Registered parameters only
The optimizer must never edit arbitrary fields directly.

Every optimizable quantity must be registered as a `DesignParameter` with:
- name,
- owner id,
- lower bound,
- upper bound,
- current value,
- default value,
- active flag,
- normalization / scale,
- optional metadata.

### 4. Constraints must be first-class
Do not hide constraints in random evaluation code.

Constraints must be explicit objects with:
- name,
- owner id,
- hard/soft mode,
- inequality/equality type,
- weight or penalty coefficient,
- active flag,
- evaluation result.

### 5. Elements own local semantics, architecture owns global assembly
Each `SpatialElement` owns:
- local geometry,
- local parameters,
- local mass/inertia logic,
- optional local constraints.

`HexacopterArchitecture` owns:
- all registered elements,
- global parameter registry view,
- global constraint registry view,
- architecture assembly,
- system-level evaluation context.

### 6. Start from reduced-order models
The first version should optimize:
- mass,
- center of gravity,
- inertia,
- control allocation,
- faulted control allocation,
- thrust reserve,
- simple structural proxies,
- packaging validity.

Do **not** begin with full FEM or long nonlinear trajectory simulation inside the optimization loop.

---

## What must be preserved from MATLAB

The old framework is still the numerical reference. Preserve these concepts and formulas.

### A. Top-level workflow
Reference:
- `@Framework/main_mdo.m line 22-245`

Preserve this logical flow:
1. build config,
2. build default design,
3. evaluate baseline,
4. run Stage 1 optimization,
5. run MOO if needed,
6. compare designs,
7. report/export results.

### B. Config-driven problem definition
Reference:
- `@Framework/config/mdo_config.m line 24-115`

Preserve the idea that one central config defines:
- active variables,
- bounds,
- objectives,
- weights,
- optimizer settings,
- fault scenario settings.

### C. Baseline design defaults
Reference:
- `@Framework/core/design_default.m line 1-49`

Preserve equivalent baseline values and units for:
- arm geometry,
- thrust parameters,
- propeller parameters,
- payload and mass.

### D. Control allocation logic
Reference:
- `@Framework/core/build_B_matrix.m line 1-37`

Preserve the initial `B` matrix formulation exactly in the first C++ version for consistency.

### E. Vehicle scaling / reduced-order mass logic
Reference:
- `@Framework/core/vehicle_model.m line 49-142`

Preserve the useful reduced-order logic such as:
- motor mass scaling with `T_max`,
- frame mass scaling with arm span,
- inertia approximation from lumped masses,
- cost-related scaling terms only if they are still useful in Stage 1.

### F. Auxiliary parameter translation
Reference:
- `@Framework/core/hexacopter_params.m line 1-91`

Preserve the idea of translating design variables into a consistent physical model object containing:
- geometry,
- mass properties,
- propulsion properties,
- allocation-related terms.

### G. Master evaluation flow
Reference:
- `@Framework/evaluation/eval_design.m line 1-192`

Preserve the high-level sequence:
1. build/update the physical model,
2. evaluate Stage 1,
3. collect metric terms,
4. aggregate objective values,
5. return a structured result.

### H. Stage 1 evaluation logic
Reference:
- `@Framework/evaluation/eval_stage1.m line 1-84`

Preserve the idea that Stage 1 is a cheap design-screening layer that combines reduced-order engineering metrics.

### I. Stage 1 metric kernels
References:
- `@Framework/metrics/compute_acs_volume.m`
- `@Framework/metrics/fault_isotropy_index.m`
- `@Framework/metrics/compute_hover_threshold.m`
- `@Framework/metrics/hover_feasibility.m`
- `@Framework/metrics/stage1_proxy_metrics.m` if present in your patched MATLAB project

Preserve or reinterpret these ideas as Stage 1 proxy metrics for:
- allocation quality,
- fault tolerance,
- hover feasibility,
- reserve margin,
- mass/power penalties.

### J. Optimizer separation
References:
- `@Framework/optimization/objective_fcn.m line 1-77`
- `@Framework/optimization/constraint_fcn.m line 1-73`
- `@Framework/optimization/run_cmaes.m line 30-377`
- `@Framework/optimization/run_soo.m line 38-284`
- `@Framework/optimization/run_moo.m line 41-268`

Preserve the separation between:
- problem definition,
- objective callback,
- constraint callback,
- optimizer runner.

### K. Comparison / reporting workflow
References:
- `@Framework/analysis/compare_designs.m line 1-228`
- `@Framework/analysis/pareto_analysis.m line 1-147`

Preserve the idea of a post-optimization analysis layer for:
- comparison tables,
- Pareto interpretation,
- sensitivity sweeps,
- reporting/export.

---

## What must be changed completely

Do **not** preserve the following MATLAB style issues.

1. Loose struct-style passing everywhere.
2. Hidden parameter-to-vector mapping.
3. Objective logic mixed with physics logic.
4. Core model functions that also know about optimizer details.
5. Scripts acting as both app layer and framework layer.
6. Hard-coded ownership of design variables inside many unrelated functions.

The C++ version must make ownership explicit.

---

## Required layer mapping from the current MATLAB project

This is the most important migration rule.

### `core/` — model objects only
Owns:
- geometry primitives,
- transforms,
- parameter objects,
- constraint objects,
- attachments / symmetry relations,
- spatial elements,
- architecture container.

Relevant MATLAB references:
- `@Framework/core/design_default.m line 1-49`
- `@Framework/config/mdo_config.m line 24-115`

Do **not** put these in `core/`:
- `build_B_matrix`,
- objective aggregation,
- evaluation metrics,
- optimizer wrappers.

`core/` defines **what the system is**, not how it is solved or optimized.

### `physics/` — physics kernels and reduced-order engineering models
Owns:
- mass property computation,
- inertia assembly,
- control allocation,
- faulted allocation,
- thrust reserve,
- hover power estimation,
- structural stress / deflection proxies.

Relevant MATLAB references:
- `@Framework/core/build_B_matrix.m line 1-37`
- `@Framework/core/vehicle_model.m line 49-142`
- `@Framework/core/hexacopter_params.m line 1-91`
- `@Framework/metrics/compute_acs_volume.m`
- `@Framework/metrics/fault_isotropy_index.m`
- `@Framework/metrics/compute_hover_threshold.m`
- `@Framework/metrics/hover_feasibility.m`
- `@Framework/metrics/stage1_proxy_metrics.m` if present in your patched MATLAB project

`physics/` must be reusable without an optimizer.

### `evaluation/` — Stage 1 metrics and objective aggregation only
Owns:
- Stage 1 metrics,
- fault-case selection for Stage 1,
- metric normalization,
- objective-term selection,
- weighted-sum aggregation,
- hard/soft constraint post-processing.

Relevant MATLAB references:
- `@Framework/evaluation/eval_design.m line 1-192`
- `@Framework/evaluation/eval_stage1.m line 1-84`
- `@Framework/evaluation/eval_acs.m`
- `@Framework/optimization/objective_fcn.m line 1-77`
- `@Framework/optimization/constraint_fcn.m line 1-73`

Important dependency rule:
- `evaluation/` may depend on `physics/` and `core/`.
- `physics/` must not depend on `evaluation/`.

### `optimization/` — design-vector mapping and solver wrappers only
Owns:
- parameter flattening,
- design-vector packing/unpacking,
- bound collection,
- objective callback,
- constraint callback,
- solver wrappers,
- SOO / MOO runners.

Relevant MATLAB references:
- `@Framework/optimization/objective_fcn.m line 1-77`
- `@Framework/optimization/constraint_fcn.m line 1-73`
- `@Framework/optimization/run_cmaes.m line 30-377`
- `@Framework/optimization/run_soo.m line 38-284`
- `@Framework/optimization/run_moo.m line 41-268`

The optimizer must only see registered parameters and callback interfaces.

### `analysis/` — reports and postprocessing only
Owns:
- comparison tables,
- Pareto analysis,
- design sweeps,
- reporting,
- export helpers.

Relevant MATLAB references:
- `@Framework/analysis/compare_designs.m line 1-228`
- `@Framework/analysis/pareto_analysis.m line 1-147`
- `@Framework/analysis/sweep_design_space.m`
- `@Framework/analysis/visualize_results.m`

### `app/` — runnable entry points only
Owns:
- small executables,
- examples,
- reproducing old workflows,
- CLI entry points.

Relevant MATLAB references:
- `@Framework/main_mdo.m line 22-245`
- any comparison driver scripts

`app/` must remain thin. It should assemble objects and call framework code, not implement framework logic itself.

---

## Recommended target C++ folder structure

```text
HexaArch/
  CMakeLists.txt
  cmake/
  app/
    main_stage1.cpp
    main_compare.cpp
    example_build_architecture.cpp

  include/
    core/
      DesignParameter.hpp
      ParameterRegistry.hpp
      Constraint.hpp
      ConstraintRegistry.hpp
      Attachment.hpp
      Transform3.hpp
      BoundingShape.hpp
      SpatialElement.hpp
      HexacopterArchitecture.hpp
      ArchitectureBuilder.hpp

    elements/
      ArmElement.hpp
      MotorElement.hpp
      RotorElement.hpp
      BatteryElement.hpp
      BodyElement.hpp
      PayloadElement.hpp

    physics/
      PhysicsTypes.hpp
      VehicleScalingModel.hpp
      MassPropertiesSolver.hpp
      AllocationMatrixBuilder.hpp
      FaultAllocationAnalyzer.hpp
      HoverPowerEstimator.hpp
      StructuralProxyEvaluator.hpp
      PackagingModel.hpp

    evaluation/
      EvaluationContext.hpp
      EvaluationResult.hpp
      Stage1Evaluator.hpp
      ArchitectureEvaluator.hpp
      ObjectiveAggregator.hpp

    optimization/
      DesignVectorMapper.hpp
      BoundsBuilder.hpp
      OptimizationProblem.hpp
      OptimizationResult.hpp
      PagmoProblemAdapter.hpp
      PagmoAlgorithmFactory.hpp
      SooRunner.hpp
      MooRunner.hpp

    analysis/
      ComparisonReporter.hpp
      ParetoAnalyzer.hpp
      SweepRunner.hpp
      CsvExporter.hpp

    utils/
      Units.hpp
      MathTypes.hpp
      JsonIO.hpp
      Id.hpp

  src/
    ... matching implementation files ...

  tests/
    test_parameter_registry.cpp
    test_mass_properties.cpp
    test_build_B_matrix.cpp
    test_fault_allocation.cpp
    test_packaging.cpp
```

Use:
- C++17 or C++20,
- Eigen,
- nlohmann/json,
- GoogleTest or Catch2,
- pagmo2,
- vcpkg manifest mode for dependency management.

The dependency path for the first implementation must assume **vcpkg** rather than ad-hoc manual linking.

---

## Optimization backend choice
Use **pagmo2** as the optimization backend, and manage external dependencies through **vcpkg manifest mode**.

### Why pagmo2
For this project, pagmo2 is a good fit because it gives:
- a stable external optimization backend,
- both single-objective and multi-objective algorithms,
- a clean problem/algorithm interface,
- easier future extension than custom-written optimizer logic.

### Current optimization policy
For the current implementation:
- use pagmo2 for **SOO** and **MOO**,
- keep evaluation serial at first,
- do not enable parallel/island complexity in the first milestone,
- use your own architecture/evaluation framework and expose it through a pagmo2-compatible problem wrapper,
- resolve pagmo2, Eigen, and other third-party dependencies through **vcpkg**, not manual include/library path editing.

### Recommended initial algorithms
Use these first unless there is a strong reason to change:
- SOO: `cmaes`
- MOO: `nsga2`

Only after the framework is stable may you add additional algorithms.

---

## vcpkg integration rules

### Dependency management policy
Use **vcpkg manifest mode** for third-party dependencies.

Required initial dependencies:
- `eigen3`
- `pagmo2`
- `nlohmann-json`
- one test framework: `gtest` or `catch2`

Optional later dependencies may be added only if clearly justified.

### CMake integration policy
The CMake project should assume package discovery through the vcpkg toolchain.

Preferred pattern:
- `find_package(Eigen3 CONFIG REQUIRED)`
- `find_package(pagmo CONFIG REQUIRED)` or the correct package config exposed by the installed port
- `find_package(nlohmann_json CONFIG REQUIRED)`
- `find_package(GTest CONFIG REQUIRED)` or equivalent

Do not add manual include directories for these dependencies unless the user explicitly requests a fallback path.

### Repository bootstrap artifacts
Phase 0 should create at least:
- `vcpkg.json`
- `CMakeLists.txt`
- optional `CMakePresets.json`
- minimal build instructions in `README.md`

### Build-system stopping rule
If package discovery fails, stop and let the user fix the local vcpkg/toolchain configuration before continuing with later phases.

---

## Required core abstractions

### 1. `DesignParameter`
Represents one optimizable scalar.

Required fields:
- `std::string name`
- `std::string ownerId`
- `double value`
- `double lowerBound`
- `double upperBound`
- `double defaultValue`
- `bool active`
- `double scale`
- optional unit / description / tags

Required methods:
- `double normalized() const`
- `void setFromNormalized(double)`
- `bool isWithinBounds() const`
- `void clamp()`

The optimizer must interact with the architecture only through registered parameters.

### 2. `Constraint`
Represents one evaluable condition.

Required fields:
- name,
- owner id,
- constraint type,
- hard/soft mode,
- penalty weight,
- active flag.

Recommended evaluation output:

```cpp
struct ConstraintEvaluation {
    double value;
    double violation;
    bool feasible;
};
```

### 3. `SpatialElement`
Abstract base class for all architecture components.

Recommended interface:

```cpp
class SpatialElement {
public:
    virtual ~SpatialElement() = default;

    virtual std::string id() const = 0;
    virtual std::string type() const = 0;

    virtual void registerParameters(ParameterRegistry& registry) = 0;
    virtual void registerConstraints(ConstraintRegistry& registry) const = 0;

    virtual double mass() const = 0;
    virtual Eigen::Vector3d localCOM() const = 0;
    virtual Eigen::Matrix3d localInertiaAtLocalCOM() const = 0;

    virtual BoundingShape boundingShape() const = 0;
    virtual Eigen::Isometry3d localPose() const = 0;

    virtual void updateFromParameters() = 0;
};
```

Important rule:
- keep the base class small,
- use derived classes or helper evaluators for specialized behavior.

### 4. `HexacopterArchitecture`
System-level container.

Responsibilities:
- own all elements,
- maintain registries,
- update dependent geometry,
- expose architecture-level queries,
- provide flat design-variable access through a mapper,
- support fault scenario evaluation inputs.

It must **not** directly implement optimizer logic.

---

## Required Stage 1 objective structure
The current implementation is **Stage 1 only**.

Use a modular weighted-sum objective built from reduced-order metrics.

### Recommended default Stage 1 objective terms
At minimum support these terms:
- `mass`
- `power`
- `fault_thrust`
- `fault_alloc`
- `hover_nom`
- optional `structural`
- optional `packaging`

### Recommended meaning of the terms
- `mass`: normalized total mass
- `power`: nominal hover power proxy
- `fault_thrust`: penalty for insufficient remaining thrust under a single-rotor fault
- `fault_alloc`: penalty from the faulted control allocation matrix quality
- `hover_nom`: nominal hover utilization or hover-quality proxy
- `structural`: penalty from reduced-order arm stress/deflection or stiffness margin
- `packaging`: penalty from overlap, insufficient clearance, or envelope violation

### Recommended weighted form
Use a configurable weighted sum:

```text
J_stage1 = sum_i w_i * J_i
```

The active objective list and weights must come from configuration, not from hard-coded logic.

---

## Required Stage 1 constraints
The first implementation should support only reduced-order hard constraints.

### Minimum hard constraints
- design variable bounds,
- rotor non-overlap,
- element overlap or invalid packaging,
- minimum arm length / rotor clearance,
- failed-hover thrust feasibility,
- minimum acceptable faulted allocation quality,
- optional CG region constraint,
- optional structural stress/deflection bound.

### Soft constraints
Soft constraints must be represented through penalties in the objective aggregation, not hidden elsewhere.

---

## Parameter registration and optimizer mapping
This is a critical requirement.

### Rule
The optimizer must only see a flat design vector built from registered active parameters.

### Required classes
- `ParameterRegistry`
- `DesignVectorMapper`
- `BoundsBuilder`

### Required behavior
- deterministic ordering of active parameters,
- pack architecture -> vector,
- unpack vector -> architecture,
- bound export,
- normalized or scaled representation,
- stable parameter identity for reporting.

### Important rule
Do not let pagmo2 access architecture members directly.
Use a dedicated wrapper that updates the architecture through the mapper.

---

## pagmo2 integration rules

### 1. Use an explicit problem adapter
Implement a class such as `PagmoProblemAdapter` that owns or references:
- the architecture,
- the evaluator,
- the vector mapper,
- the selected objective list,
- the constraint policy.

### 2. Separate SOO and MOO clearly
- `SooRunner` should construct a pagmo2 problem configured for one scalar objective.
- `MooRunner` should construct a pagmo2 problem configured for a vector objective.

### 3. Constraint handling policy
If pagmo2 algorithms used in the first phase do not directly support your preferred constraint style, use a deterministic penalty method in the evaluation layer.
Do not scatter penalty logic across multiple places.

### 4. Reproducibility
Expose:
- algorithm name,
- seed,
- population size,
- generation count,
- stopping criteria,
- objective list,
- parameter bounds.

Write these to output files for every optimization run.

---

## Phase 0 — project bootstrap and vcpkg integration only
This phase exists because Codex may generate correct source code but still fail at dependency wiring or CMake configuration.

### Objective of Phase 0
Create a buildable C++ project skeleton that is explicitly prepared for **vcpkg manifest mode**.

Do not proceed to the real framework implementation until the user confirms that:
1. vcpkg is available locally,
2. the manifest is recognized,
3. CMake configures successfully,
4. the project builds successfully.

### Phase 0 deliverables
Implement the following only:

1. base folder structure,
2. top-level `CMakeLists.txt`,
3. `vcpkg.json` manifest,
4. optional `CMakePresets.json` or clearly documented CMake configure instructions for vcpkg toolchain use,
5. minimal library targets for:
   - `core`,
   - `physics`,
   - `evaluation`,
   - `optimization`,
   - `analysis`,
6. one runnable application entry:
   - `app/main_stage1.cpp`,
7. one minimal smoke example:
   - build a default architecture object,
   - create a trivial evaluator call,
   - print a placeholder result,
8. placeholder source files and headers for all required modules,
9. pagmo2-aware but still minimal optimization target wiring,
10. a short `README.md` section describing how to configure and build with vcpkg.

### Phase 0 dependency policy
Use **vcpkg manifest mode**.

Create a `vcpkg.json` containing the dependencies needed for the current code skeleton. At minimum, prepare for:
- `eigen3`,
- `pagmo2`,
- `nlohmann-json`,
- one test library such as `gtest` or `catch2`.

Do not hard-code absolute include paths or library paths.
Do not write custom Find scripts for pagmo2 or Eigen unless strictly necessary.
Prefer normal CMake package discovery through the vcpkg toolchain.

### Phase 0 CMake policy
The project must be written so that the intended build path is:
- vcpkg toolchain file provided at configure time,
- dependencies resolved through `vcpkg.json`,
- package discovery performed through `find_package(...)`.

Codex should prepare CMake for this style, but must not assume the user’s local vcpkg path.

Use placeholders or documented variables for the toolchain path rather than machine-specific paths.

### Phase 0 success condition
After Codex finishes Phase 0, stop and wait.

Do not continue until the user:
1. installs or confirms vcpkg,
2. runs CMake with the vcpkg toolchain,
3. confirms the project builds successfully,
4. asks to continue.

### Reason for stopping after Phase 0
This project depends on external packages. Even with vcpkg, local toolchain and generator issues can block progress. It is better to validate the build system first before generating the full framework.

---

## Phase 1 — implement core architecture and Stage 1 evaluation
Proceed only after the user confirms Phase 0 builds successfully with vcpkg.

### Phase 1 goals
Implement:
- `DesignParameter`,
- `Constraint`,
- `ParameterRegistry`,
- `ConstraintRegistry`,
- `SpatialElement`,
- initial derived elements,
- `HexacopterArchitecture`,
- Stage 1 evaluation without running optimization yet.

### Minimum derived elements for Phase 1
- `ArmElement`
- `MotorElement`
- `RotorElement`
- `BatteryElement`
- `BodyElement`
- `PayloadElement`

### Physics kernels for Phase 1
Implement:
- mass aggregation,
- COM aggregation,
- inertia tensor aggregation,
- nominal allocation matrix,
- single-rotor-fault allocation matrix,
- hover power proxy,
- structural proxy for arms,
- packaging overlap checks.

### Phase 1 acceptance criteria
A default architecture can be built and evaluated for Stage 1 metrics in a deterministic way.

---

## Phase 2 — enable pagmo2-backed optimization
Proceed only after Phase 1 works correctly and the vcpkg-based build remains stable.

### Phase 2 goals
Implement:
- `DesignVectorMapper`
- `BoundsBuilder`
- `PagmoProblemAdapter`
- `SooRunner`
- `MooRunner`
- config-driven objective selection

### Required first algorithms
- SOO: `cmaes`
- MOO: `nsga2`

### Phase 2 acceptance criteria
The project can:
- run a Stage 1 SOO optimization,
- run a Stage 1 MOO optimization,
- export baseline and optimized results,
- compare designs through the analysis layer.

---

## Phase 3 — reporting and analysis
Proceed only after Phase 2 works correctly.

### Phase 3 goals
Implement:
- comparison table generation,
- Pareto front export,
- summary CSV/JSON export,
- reproducible run metadata export,
- small runnable comparison example in `app/main_compare.cpp`.

### Important note
This phase is still Stage 1 only.
Do not add any Stage 2 or long simulation logic here.

---

## Recommended implementation order inside Codex
When generating code, follow this exact order.

1. create the project skeleton and CMake,
2. implement utility math/types,
3. implement parameter and constraint registries,
4. implement `SpatialElement` and derived elements,
5. implement `HexacopterArchitecture`,
6. implement physics kernels,
7. implement Stage 1 evaluator,
8. implement analysis helpers,
9. implement pagmo2 integration only after Phase 0 build confirmation,
10. implement SOO/MOO runners,
11. implement comparison applications.

Do not reverse this order.

---

## Detailed migration guidance by MATLAB file

### `@Framework/core/build_B_matrix.m line 1-37`
Port this logic into:
- `physics/AllocationMatrixBuilder.*`

Do not leave it in `core/`.

### `@Framework/core/vehicle_model.m line 49-142`
Port the mass / scaling / inertia-related reduced-order formulas into:
- `physics/VehicleScalingModel.*`
- `physics/MassPropertiesSolver.*`

### `@Framework/core/hexacopter_params.m line 1-91`
Use this as reference for translating registered design variables into a physically consistent internal model.
The equivalent logic should live across:
- `core/HexacopterArchitecture.*`
- `physics/PhysicsTypes.*`
- `physics/VehicleScalingModel.*`

### `@Framework/evaluation/eval_design.m line 1-192`
Use this only as reference for the evaluation flow.
Port the Stage 1 parts into:
- `evaluation/ArchitectureEvaluator.*`
- `evaluation/Stage1Evaluator.*`
- `evaluation/ObjectiveAggregator.*`

Do not port Stage 2 branches.

### `@Framework/evaluation/eval_stage1.m line 1-84`
This is the main reference for the first evaluator implementation.
Port and generalize it into:
- `evaluation/Stage1Evaluator.*`

### `@Framework/optimization/objective_fcn.m line 1-77`
Use as reference for the optimization objective callback structure only.
In C++, objective assembly belongs in:
- `evaluation/ObjectiveAggregator.*`

The pagmo2 adapter should only call that evaluator.

### `@Framework/optimization/constraint_fcn.m line 1-73`
Use as reference for constraint callback structure only.
In C++, constraint evaluation belongs in:
- `evaluation/ArchitectureEvaluator.*`
- `optimization/PagmoProblemAdapter.*`

### `@Framework/optimization/run_cmaes.m line 30-377`
Do not port this runner directly.
Use it only as reference for:
- optimization configuration,
- stopping/reporting style,
- initialization style.

The actual backend should be pagmo2 through the vcpkg-managed dependency path.

### `@Framework/optimization/run_soo.m line 38-284`
Use as reference for the top-level SOO flow only.
Port that flow into:
- `optimization/SooRunner.*`

### `@Framework/optimization/run_moo.m line 41-268`
Use as reference for the top-level MOO flow only.
Port that flow into:
- `optimization/MooRunner.*`

### `@Framework/analysis/compare_designs.m line 1-228`
Use as reference for result-table structure and metric reporting.
Port into:
- `analysis/ComparisonReporter.*`

### `@Framework/analysis/pareto_analysis.m line 1-147`
Use as reference for Pareto post-processing and knee-point style reporting.
Port into:
- `analysis/ParetoAnalyzer.*`

---

## Coding rules for Codex

### Rule 1
Do not implement deep inheritance hierarchies unless they clearly simplify the design.
Use composition where possible.

### Rule 2
Do not place optimization callbacks inside `SpatialElement` classes.

### Rule 3
Do not place Stage 1 objective formulas inside `core/` or `physics/`.
They belong to `evaluation/`.

### Rule 4
Do not call pagmo2 directly from `app/`.
Always use `optimization/` wrappers.

### Rule 5
Do not silently create optimizer-visible parameters.
All optimizer-visible parameters must be explicitly registered.

### Rule 6
Keep units explicit and consistent.
Prefer SI units.

### Rule 7
Every major module must be independently testable.

---

## Minimum test plan

### Unit tests
Implement tests for:
- parameter registration and stable ordering,
- design-vector pack/unpack,
- mass/COM/inertia aggregation,
- `B` matrix generation,
- faulted allocation metric,
- packaging overlap detection,
- Stage 1 objective aggregation.

### Integration tests
Implement at least:
- build default architecture and evaluate Stage 1,
- run one small SOO case after pagmo2 is enabled,
- run one small MOO case after pagmo2 is enabled,
- generate one comparison report.

---

## Acceptance criteria for the current project phase
The current work is acceptable only if all of the following are true:

1. the project builds in Phase 0 through vcpkg-based dependency resolution,
2. the architecture and evaluation logic are separated cleanly,
3. Stage 1 metrics run without simulation,
4. the optimizer sees only registered parameters,
5. pagmo2 integration is isolated behind wrappers,
6. SOO and MOO use pagmo2 after Phase 2,
7. all Stage 2 logic is absent from the current implementation.

---

## Final instruction to Codex
Rebuild the project as a **layered Stage 1 C++ framework** that preserves the MATLAB numerical logic where useful, but reorganizes the code into:
- `core/`,
- `physics/`,
- `evaluation/`,
- `optimization/`,
- `analysis/`,
- `app/`.

Use **pagmo2** as the optimization backend and manage it through **vcpkg manifest mode**.

In Phase 0, prepare the repository for vcpkg-based configuration and stop after the user confirms that the project configures and builds successfully.

Do not implement Stage 2.
Do not implement full simulation.
Do not implement full FEM.
Do not continue past the vcpkg/bootstrap boundary until the user confirms the dependency setup and build are successful.

## Hard requirement: cpp-lsp first

Codex must use **cpp-lsp first** before any implementation or refactoring.
If cpp-lsp is unavailable or not functioning correctly, **do not proceed**.

Workflow:
1. Verify cpp-lsp availability.
2. Inspect workspace symbols and diagnostics through cpp-lsp.
3. Use symbol-aware navigation to locate the relevant classes, functions, and build files.
4. Read only the minimum necessary local code afterward.

Do not proceed by manual file scanning as a fallback.q