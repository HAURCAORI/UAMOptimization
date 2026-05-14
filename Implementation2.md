# Codex implementation instructions

## Project title
Fault-tolerant hexacopter architecture co-design using simplified spatial element modeling

## Purpose of this instruction revision
This instruction file supersedes the previous Stage 1 refactoring instructions for the **next refactor only**.

The current C++ project already has:
- a layered Stage 1 structure,
- pagmo2-backed SOO/MOO,
- shared `DesignParameter` objects,
- a CLI with `eval`, `soo`, `moo`, and `compare` modes.

However, the current implementation is **not yet an extensible spatial-element framework**.
The main missing capability is:
- extensible `SpatialElement` ownership,
- explicit geometric relationships / attachment graph,
- geometry-aware constraints derived from those relationships,
- physics that is evaluated from the element graph rather than from hard-coded architecture scalars.

This refactor must fix those issues while preserving the useful reduced-order Stage 1 logic.

---

## Implementation scope for the current phase
This document defines the **current C++ implementation scope only**.

The implementation scope is:
- preserve the useful Stage 1 numerical logic from the MATLAB framework,
- preserve pagmo2 as the optimization backend,
- preserve the layered C++ layout,
- refactor the framework so that `SpatialElement` becomes genuinely extensible,
- add geometric relationship modeling and related constraints,
- keep the implementation **Stage 1 only**.

### Explicitly out of scope
Do **not** implement the following in this phase:
- Stage 2 evaluation,
- mission simulation,
- hover recovery simulation,
- emergency landing trajectory optimization,
- full FEM,
- GUI / viewer,
- topology optimization that changes rotor count,
- mesh geometry or CAD kernels.

---

## Hard architectural rule
Preserve **formulas and workflow**, but refactor **ownership and responsibilities**.

### Required layers
- `core/` for geometry, parameters, constraints, attachments, and architecture objects,
- `physics/` for mass, inertia, control allocation, structural proxies,
- `evaluation/` for Stage 1 metrics and objective aggregation,
- `optimization/` for design-vector mapping and pagmo2 wrappers,
- `analysis/` for comparison tables and reporting,
- `app/` for runnable examples.

This layer split is mandatory.

---

## Current implementation audit (must be treated as input to the refactor)
The implementation notes say the project already has all six layers, shared canonical parameters, Stage 1 evaluators, pagmo-backed SOO/MOO, and CLI/export support. Those statements are the correct starting point for this refactor, not something to delete. @ImplementationNotes.md line 5-20

The notes also say the current physics is intentionally reduced-order and that the current core layer contains `DesignParameter`, `ParameterRegistry`, `Constraint`, `ConstraintRegistry`, `SpatialElement`, and `HexacopterArchitecture`. @ImplementationNotes.md line 22-63

However, from static inspection of the current C++ code, the following architectural problems remain and must be corrected:

### A. `SpatialElement` is not truly extensible
Current problem:
- concrete element classes are defined as local classes inside `HexacopterArchitecture.cpp`,
- external extension is impossible without editing architecture internals,
- `SpatialElement::registerParameters()` and `registerConstraints()` exist, but actual parameter/constraint ownership is still centralized.

Relevant files:
- `@FrameworkCpp/include/core/SpatialElement.hpp line 23-41`
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 26-236`
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 337-386`

### B. No explicit geometry relationship / attachment model
Current problem:
- elements only expose a local pose,
- there is no parent-child attachment graph,
- there is no relative transform constraint system,
- there is no symmetry / mirroring / attachment rule abstraction.

Relevant files:
- `@FrameworkCpp/include/core/SpatialElement.hpp line 37-40`
- `@FrameworkCpp/include/core/HexacopterArchitecture.hpp line 13-61`
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 232-255`

### C. Geometry primitives are too weak
Current problem:
- only spherical bounding shapes are supported,
- no box / cylinder / segment primitives,
- no primitive-pair constraint infrastructure.

Relevant files:
- `@FrameworkCpp/include/core/SpatialElement.hpp line 14-21`

### D. Physics still bypasses the element graph
Current problem:
- `VehicleScalingModel` computes rotor positions, mass properties, packaging, and structural proxies directly from scalar architecture getters,
- the element graph is not the source of truth.

Relevant files:
- `@FrameworkCpp/src/physics/VehicleScalingModel.cpp line 13-160`
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 337-386`

### E. Constraint extensibility is still name-dispatch based
Current problem:
- the constraint registry stores metadata,
- actual evaluation is hard-coded by name in `Stage1Evaluator`.

Relevant files:
- `@FrameworkCpp/include/core/Constraint.hpp line 19-30`
- `@FrameworkCpp/include/core/ConstraintRegistry.hpp line 10-20`
- `@FrameworkCpp/src/evaluation/Stage1Evaluator.cpp line 282-303`

### F. Geometry logic is duplicated
Current problem:
- motor / rotor placement logic is implemented in multiple places,
- this creates divergence risk.

Relevant files:
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 24-25`
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 153-157`
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 187-188`
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 223-225`
- `@FrameworkCpp/src/physics/VehicleScalingModel.cpp line 13-34`

### G. At least one design variable is effectively dead
Current problem:
- `cT` is exposed as a design parameter,
- but `VehicleScalingModel` overrides it whenever `propellerDiameter() > 0`.

Relevant files:
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 345-351`
- `@FrameworkCpp/src/physics/VehicleScalingModel.cpp line 66-71`

This must be corrected either by:
- making `cT` active and authoritative, or
- removing it as an active design parameter in the current phase.

---

## Primary design objective of this refactor
Make the architecture **element-driven, attachment-aware, and constraint-extensible** while keeping the current Stage 1 optimization flow.

The target rule is:

> The optimizer manipulates registered parameters.
> Parameters update elements.
> Elements are assembled through an attachment graph.
> Physics is computed from the assembled element graph.
> Constraints are evaluated from registered evaluators, not from hard-coded names.

---

## New mandatory design principles

### 1. Public concrete element classes
Concrete element types must be public types in `core/`, not anonymous local classes in `HexacopterArchitecture.cpp`.

At minimum, create public classes for:
- `ArmElement`
- `MotorElement`
- `RotorElement`
- `BatteryElement`
- `BodyElement`
- `PayloadElement`

These classes must each live in their own header/source pair or in a small, obvious grouped module.

### 2. Attachment graph is mandatory
Add an explicit attachment / relationship system.

Required concepts:
- `Attachment`
- parent element id
- child element id
- local relative transform
- optional parameterized transform components
- optional symmetry / mirroring group id
- optional attachment constraints

The architecture must assemble world poses by traversing the attachment graph.

### 3. Geometry primitives must be extensible
Replace the current sphere-only bounding shape with a primitive system that can support at minimum:
- sphere
- box
- cylinder
- disk
- segment / beam proxy

Keep the collision / distance logic reduced-order and simple.
Do **not** add a full mesh engine.

### 4. Elements must own local parameter registration
Each concrete element must register its own local parameters and local constraints.

The architecture may add:
- shared parameters,
- global constraints,
- cross-element constraints,
- convenience builders.

But it must not remain the sole owner of every parameter definition.

### 5. Constraints must be evaluable objects or callbacks
Do not continue the pattern of hard-coded string-based dispatch in `Stage1Evaluator`.

Required direction:
- each constraint has metadata **and** an evaluator binding,
- evaluator binding may be a callable, function object, or dedicated constraint class,
- hard/soft handling remains part of the constraint object.

### 6. Physics must consume the assembled architecture graph
`VehicleScalingModel` and related physics utilities must compute from:
- elements,
- assembled world poses,
- element geometry,
- element mass/inertia contributions,
- thrust element definitions,
- attachment-derived positions.

Do not keep scalar duplication as the primary source of truth.

### 7. Preserve reduced-order MATLAB logic where useful
Preserve the reduced-order logic from the MATLAB framework for:
- allocation matrix construction,
- vehicle scaling / mass proxy,
- hover power proxy,
- faulted hover feasibility,
- structural proxy,
- simple packaging checks.

But port that logic into the new ownership structure.

Relevant MATLAB references:
- `@Framework/core/build_B_matrix.m line 1-37`
- `@Framework/core/vehicle_model.m line 49-141`
- `@Framework/metrics/hover_feasibility.m line 1-57`
- `@Framework/config/mdo_config.m line 24-53`
- `@Framework/optimization/objective_fcn.m line 1-76`

---

## Required target architecture

### `core/`
Must own:
- `DesignParameter`
- `ParameterRegistry`
- `Constraint`
- `ConstraintRegistry`
- `GeometryPrimitive`
- `Attachment`
- `TransformRule`
- `SpatialElement`
- public concrete element classes
- `HexacopterArchitecture`
- architecture builder helpers

### `physics/`
Must own:
- mass property assembly
- inertia aggregation
- allocation matrix construction
- failed-rotor allocation variants
- hover power proxy
- reduced-order structural proxy
- primitive distance / clearance utilities

### `evaluation/`
Must own:
- Stage 1 metric evaluation
- objective aggregation
- hard/soft constraint aggregation
- feasibility decision

### `optimization/`
Must own:
- design vector mapper
- bounds export
- pagmo2 problem adapter
- SOO runner
- MOO runner

### `analysis/`
Must own:
- comparison tables
- CSV / JSON export
- Pareto analysis

### `app/`
Must own:
- CLI entry
- simple example architectures
- reproducible evaluation / optimization modes

---

## Required new core abstractions

### `GeometryPrimitive`
A value type representing a simple geometry primitive.

Required fields or equivalent:
- primitive kind
- local transform
- dimensions
- optional padding / safety radius

Supported kinds for this phase:
- sphere
- box
- cylinder
- disk
- segment

### `Attachment`
Represents a geometric relationship between two elements.

Required data:
- parent id
- child id
- local transform from parent frame to child frame
- optional parameter bindings for translation / rotation terms
- enabled flag
- optional symmetry tag

### `SpatialElement`
Base class for extensible elements.

Must support:
- stable id
- type name
- local geometry primitive(s)
- local mass / COM / inertia contribution
- local parameter registration
- local constraint registration
- attachment endpoints or attachable frames
- `updateFromParameters()`

Do not require every derived class to implement excessive interfaces.
Keep the virtual API lean.

### `ConstraintBinding`
Add a mechanism that binds a constraint to an evaluator.

Examples:
- callable returning signed metric value
- class implementing `evaluate(context)`

Constraint metadata and constraint evaluation must no longer be separate hard-coded worlds.

### `AssemblyState`
Add an internal assembled-geometry state containing:
- world pose of each element
- world geometry primitive cache
- mass property cache
- rotor/thrust element pose cache

This can live in `core/` or `physics/`, but it must be an explicit concept.

---

## Required optimization model behavior
Use pagmo2 as the optimization backend.

### SOO backend
Default:
- `pagmo::cmaes`

### MOO backend
Default:
- `pagmo::nsga2`

### Required validation
- validate NSGA-II population size properly,
- expose a deterministic seed path,
- continue selecting best feasible result explicitly,
- keep infeasibility penalties deterministic,
- separate raw best from feasible best.

Do not rewrite custom optimizers.

---

## Required implementation phases

## Phase 0 — build/bootstrap only
Goal:
- keep the project buildable with CMake + vcpkg,
- verify pagmo2 and Eigen integration,
- keep `app/main.cpp` compiling.

Tasks:
1. Keep or fix root `CMakeLists.txt`.
2. Keep or fix `vcpkg.json`.
3. Ensure imported targets are used for Eigen and pagmo2.
4. Ensure the library target and CLI target both build.
5. Do **not** begin deeper refactoring until the user confirms the build works.

Stop condition:
- after Phase 0, stop and wait for user confirmation that dependency installation and build succeeded.

---

## Phase 1 — geometry / attachment core
Goal:
- introduce extensible geometry primitives and attachment graph without breaking the Stage 1 CLI structure.

Tasks:
1. Add `GeometryPrimitive`.
2. Add `Attachment`.
3. Add basic primitive distance / clearance utilities for the supported primitives.
4. Extend `SpatialElement` to expose local primitives and attachable frames.
5. Add architecture assembly logic that computes world transforms from attachments.
6. Keep existing reduced-order evaluation behavior compiling during the transition.

Acceptance criteria:
- an element can be attached to another element with a relative transform,
- world poses are assembled by traversal rather than by scattered ad hoc formulas,
- rotor positions can be obtained from attached rotor elements.

---

## Phase 2 — public concrete elements
Goal:
- make element extensibility real.

Tasks:
1. Move all concrete element types out of `HexacopterArchitecture.cpp`.
2. Create public element classes in `core/`.
3. Ensure each element registers its own local parameters and local constraints.
4. Add small builder helpers for the default hexacopter architecture.
5. Remove anonymous local element-class definitions from the architecture implementation.

Acceptance criteria:
- a new element type can be added without editing `HexacopterArchitecture.cpp` internals,
- element registration works through public APIs.

---

## Phase 3 — architecture-driven physics
Goal:
- make physics consume the assembled element graph.

Tasks:
1. Refactor `VehicleScalingModel` so it reads from assembled elements.
2. Compute mass / COM / inertia by aggregating element contributions.
3. Derive rotor positions from attached thrust / rotor elements.
4. Derive packaging from geometry primitives, not only scalar arm lengths.
5. Preserve the reduced-order Stage 1 formulas where still valid.
6. Eliminate duplicated rotor-position formulas.

Acceptance criteria:
- changing an element pose changes the evaluated mass properties and geometry checks,
- rotor clearance is evaluated from element geometry,
- duplicated geometry logic is removed.

---

## Phase 4 — extensible constraints and Stage 1 evaluation
Goal:
- replace name-dispatch constraints with real evaluable constraints.

Tasks:
1. Add evaluator bindings to constraints.
2. Remove hard-coded `if (constraint.name == ...)` dispatch from `Stage1Evaluator`.
3. Distinguish:
   - element-local constraints,
   - global architecture constraints,
   - cross-element geometric constraints.
4. Keep Stage 1 metrics limited to the current reduced-order set.

Stage 1 metrics for this phase:
- mass
- hover power proxy
- fault thrust penalty
- fault allocation penalty/proxy
- nominal hover proxy
- structural proxy
- packaging / clearance proxy

Acceptance criteria:
- new constraints can be added without editing the evaluator name-dispatch block,
- geometry-related constraints are evaluated through the new primitive/attachment system.

---

## Phase 5 — optimization mapping cleanup
Goal:
- keep pagmo2 optimization but make the parameter model architecture-driven.

Tasks:
1. Ensure the design vector maps only to registered active parameters.
2. Ensure shared parameters and element-local parameters are both supported.
3. Remove any dead parameter behavior.
4. Keep SOO and MOO wrappers stable.
5. Validate NSGA-II population requirements.

Acceptance criteria:
- optimizer-visible parameters all affect the evaluated model,
- parameter activation / deactivation is deterministic,
- SOO and MOO both run against the new architecture model.

---

## Phase 6 — analysis and reporting stabilization
Goal:
- keep the current CLI/reporting structure working after the architecture refactor.

Tasks:
1. Update exported parameter ids and metric names if needed.
2. Keep `eval`, `soo`, `moo`, and `compare` modes functional.
3. Ensure comparison tables can report geometry-related constraints and feasibility.
4. Add regression tests for:
   - parameter mapping,
   - attachment assembly,
   - primitive clearance,
   - faulted allocation metric,
   - SOO/MOO result export.

---

## Specific refactor rules

### Rule 1
Do not delete the current layered project layout.
Refactor **within** the existing layered structure.

### Rule 2
Do not keep concrete element classes hidden in `HexacopterArchitecture.cpp`.

### Rule 3
Do not continue using sphere-only bounding geometry.

### Rule 4
Do not keep constraint evaluation as string-name dispatch.

### Rule 5
Do not let `VehicleScalingModel` remain independent from the element graph.

### Rule 6
Do not introduce Stage 2 or high-fidelity simulation.

### Rule 7
Do not replace pagmo2.

---

## Compatibility rules with the existing MATLAB logic
Preserve these concepts from the MATLAB framework:
- control allocation matrix definition,
- parameterized vehicle scaling model,
- hover feasibility screening,
- Stage 1 weighted objective pattern,
- optimization driven by active design variables.

Reference files:
- `@Framework/core/build_B_matrix.m line 1-37`
- `@Framework/core/vehicle_model.m line 49-141`
- `@Framework/metrics/hover_feasibility.m line 1-57`
- `@Framework/config/mdo_config.m line 24-53`
- `@Framework/optimization/objective_fcn.m line 1-76`

Do **not** preserve these structural weaknesses from MATLAB:
- loose struct-style ownership,
- mixed objective/constraint/physics responsibilities,
- hidden vector mapping,
- scripts acting as framework internals.

---

## First concrete files that should change
Start by auditing and refactoring these current C++ files first:
- `@FrameworkCpp/include/core/SpatialElement.hpp line 14-41`
- `@FrameworkCpp/include/core/HexacopterArchitecture.hpp line 13-61`
- `@FrameworkCpp/src/core/HexacopterArchitecture.cpp line 26-386`
- `@FrameworkCpp/src/physics/VehicleScalingModel.cpp line 13-160`
- `@FrameworkCpp/src/evaluation/Stage1Evaluator.cpp line 175-316`
- `@FrameworkCpp/include/core/Constraint.hpp line 19-30`
- `@FrameworkCpp/include/core/ConstraintRegistry.hpp line 10-20`

Then update these optimization-facing files only after the core/physics/evaluation refactor stabilizes:
- `@FrameworkCpp/src/optimization/DesignVectorMapper.cpp`
- `@FrameworkCpp/src/optimization/PagmoProblemAdapter.cpp`
- `@FrameworkCpp/src/optimization/SooRunner.cpp`
- `@FrameworkCpp/src/optimization/MooRunner.cpp`

---

## Required acceptance criteria for this refactor
The refactor is acceptable only if all of the following become true:

1. `SpatialElement` is externally extensible.
2. Concrete element types are public and not hidden in architecture internals.
3. The architecture supports explicit geometric relationships / attachments.
4. Geometry-related constraints can be defined from those relationships.
5. Stage 1 physics consumes the assembled element graph.
6. Constraint evaluation no longer relies on hard-coded string dispatch.
7. No optimizer-visible parameter is dead.
8. pagmo2 SOO/MOO still works after the refactor.
9. Stage 1-only scope is preserved.
10. The project remains buildable with CMake + vcpkg.

---

## Final instruction to Codex
Do **not** treat this as a greenfield rewrite.

This is a **surgical but architecture-correct refactor** of the current C++ project.
Preserve:
- the current Stage 1 reduced-order numerical intent,
- pagmo2 integration,
- the six-layer layout,
- the CLI modes.

But correct the framework so that:
- `SpatialElement` is actually extensible,
- attachments and geometric relationships exist,
- geometry constraints are real first-class objects,
- and physics is driven by the assembled architecture rather than by duplicated scalar formulas.
