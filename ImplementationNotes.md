# Implementation Notes

## Scope implemented

The current C++ project in `FrameworkCpp/` implements:
- the original Phase 0/1/2/3 milestones from `Implementation.md`
- the architecture refactor series from `Implementation2.md`

Implemented layers:
- `core/`
- `physics/`
- `evaluation/`
- `optimization/`
- `analysis/`
- `app/`

Implemented milestones:
- Phase 0: bootstrap, project layout, LSP setup
- Phase 1: shared-parameter Stage 1 model
- Phase 2: pagmo2-backed SOO/MOO
- Phase 3: CLI/report/export support
- `Implementation2.md` refactor: attachment-aware, geometry-aware, element-driven Stage 1 architecture

Still out of scope:
- Stage 2 logic
- long mission simulation
- nonlinear recovery simulation
- FEM
- mesh/CAD geometry
- GUI/viewer tooling

## Current architecture

### 1. Canonical shared parameters

`DesignParameter` is the canonical shared parameter object.

Current behavior:
- `ParameterRegistry` owns canonical parameter objects
- elements bind to shared parameter pointers
- one parameter can have multiple consumers through `consumer_ids`
- the optimizer maps only active registered parameters
- `updateFromParameters()` updates bound elements and then re-assembles the architecture

Default active design parameters:
- `Lx`
- `Lyi`
- `Lyo`
- `T_max`
- `cT`

Registered but inactive by default:
- `d_prop`
- `m_payload`

`cT` is active and authoritative. It is not overwritten inside the vehicle model.

### 2. Public composition API

`HexacopterArchitecture` is no longer only a hidden default-builder container.

Current public composition surface:
- `addElement(...)`
- `removeElement(...)`
- `clearElements()`
- `addAttachment(...)`
- `removeAttachment(...)`
- `clearAttachments()`
- `addConstraint(...)`
- `removeConstraint(...)`
- `rebuildAssembly()`

This allows external code to assemble a custom architecture graph without editing the built-in default builder path.

Important behavior:
- added elements are rebound to the architecture’s canonical parameter registry
- added elements register their own parameter consumers and local constraints
- rebuilding the assembly re-updates element state and re-walks the attachment graph

### 3. Copy/assignment preserve the actual graph

Copy/assignment no longer reconstruct through the default hex template.

Current copy behavior:
- parameters are copied
- attachments are copied
- constraints are copied
- each element is cloned through `SpatialElement::clone()`
- cloned elements are rebound to the copied parameter registry through `rebindParameters(...)`
- the copied graph is re-assembled

This means custom element/attachment graphs survive copying instead of collapsing back to the default architecture.

### 4. Public element model

Concrete elements are public `core/` types rather than hidden classes inside `HexacopterArchitecture.cpp`.

Current element set:
- `BodyElement`
- `BatteryElement`
- `PayloadElement`
- `ArmElement`
- `MotorElement`
- `RotorElement`

Implemented in:
- `FrameworkCpp/include/core/Elements.hpp`
- `FrameworkCpp/src/core/Elements.cpp`

Each element now participates through public APIs:
- `clone()`
- `registerParameters()`
- `rebindParameters()`
- `registerConstraints()`
- `updateFromParameters()`
- local geometry primitive definition
- local mass / COM / inertia contribution

### 5. Attachment-driven assembly

The architecture is assembled from an explicit attachment graph.

Core concepts:
- `GeometryPrimitive`
- `Attachment`
- `AttachmentRelationship`
- `AssemblyState`

Current attachment flow:
- elements are created or added
- parent/child attachments are created or added
- `assemble()` traverses the graph
- each assembled element receives a world pose
- physics reads the assembled graph instead of rebuilding geometry from scattered formulas

Current default graph:
- root -> body
- body -> battery
- body -> payload
- body -> each arm
- arm_i -> motor_i
- motor_i -> rotor_i

Important note:
- `AssemblyState::find()` is implemented in `src/core/AssemblyState.cpp`, not inline in the header, so `SpatialElement` does not need to be a complete type in `AssemblyState.hpp`

### 6. First-class attachment relationships

Attachments are no longer just anchor names plus ad hoc transforms.

Current relationship model in `Attachment.hpp`:
- `rigid_mount`
- `local_offset`
- `mirrored_local_offset`

Each `Attachment` now contains:
- parent id
- child id
- parent anchor
- child anchor
- `AttachmentRelationship`
- contact policy
- optional custom `relative_transform` callback override

This gives the framework a reusable built-in relationship layer while still allowing callback-based custom transforms when needed.

### 7. Attachment contact policy and packaging exemptions

Attachments also carry a contact policy:
- `enforce_clearance`
- `allow_touch`
- `bonded_overlap`

Current behavior:
- the architecture precomputes packaging exemption pairs during assembly
- bonded ancestry is tracked through the attachment graph
- physics asks the architecture whether a pair is exempt through `isPackagingPairExempt(...)`

This replaced the earlier direct parent/child-only filtering and removed repeated parent-map rebuilding from the physics layer.

### 8. Capability-style physics interfaces

The physics layer no longer depends only on concrete element classes for the main contribution paths.

Current capability interfaces:
- `IPropulsionRotor`
- `IStructuralMember`
- `IMotorMassContributor`
- `IPayloadMassContributor`

Implemented in:
- `FrameworkCpp/include/core/ElementCapabilities.hpp`

Current use:
- `VehicleScalingModel` uses rotor capability for rotor index and yaw sign
- structural-member capability for span and frame-mass contribution
- motor-mass contributor marker for motor mass
- payload-mass contributor marker for payload mass

This does not eliminate all reduced-order assumptions, but it decouples the physics layer from several exact concrete element types.

## Implemented modules

### Core

Implemented:
- `DesignParameter`
- `ParameterRegistry`
- `Constraint`
- `ConstraintRegistry`
- `GeometryPrimitive`
- `Attachment`
- `AttachmentRelationship`
- `AssemblyState`
- `SpatialElement`
- `ElementCapabilities`
- public concrete element classes
- `HexacopterArchitecture`
- `DefaultHexacopterBuilder`

Current geometry primitive support:
- sphere
- box
- cylinder
- disk
- segment

### Physics

Implemented:
- `AllocationMatrixBuilder`
- `PrimitiveDistance`
- `VehicleScalingModel`
- `PhysicsTypes`

Current physics behavior:
- mass / COM / inertia are aggregated from assembled elements
- rotor positions are read from assembled rotor elements
- rotor yaw signs are element-defined, not inferred from index-only hardcoding inside the physics layer
- packaging / clearance uses primitive-pair distance checks
- packaging exemptions come from attachment contact policy
- structural proxy remains reduced-order
- allocation matrices remain reduced-order Stage 1 / six-rotor oriented

### Evaluation

Implemented:
- `ArchitectureEvaluator`
- `Stage1Evaluator`
- `ObjectiveAggregator`
- `EvaluationContext`
- `EvaluationResult`

Current Stage 1 metrics include:
- mass
- power
- fault thrust penalty
- fault allocation penalty/proxy
- nominal hover proxy
- structural proxy
- packaging proxy

Current constraint model:
- each `Constraint` carries metadata and an evaluator callback
- architecture-global constraints are registered by `HexacopterArchitecture`
- element-local constraints are registered by concrete elements
- evaluated constraint results preserve:
  - `stable_id`
  - `owner_id`
  - `name`

### Optimization

Implemented:
- `DesignVectorMapper`
- `BoundsBuilder`
- `OptimizationProblem`
- `PagmoProblemAdapter`
- `SooRunner`
- `MooRunner`

Current backend policy:
- SOO: pagmo `cmaes`
- MOO: pagmo `nsga2`

Current optimization behavior:
- design vectors map only active registered parameters
- infeasible solutions receive deterministic large penalties
- SOO keeps both raw best and best feasible
- MOO keeps both raw population and feasible population
- penalty lookup uses stable constraint ids
- `nsga2` population size is explicitly validated:
  - minimum size `>= 5`
  - multiple of `4`

### Analysis

Implemented:
- `ComparisonReporter`
- `ParetoAnalyzer`
- `CsvExporter`

### App

Main VS entry point:
- `FrameworkCpp/app/main.cpp`

Supported CLI modes:
- `eval`
- `soo`
- `moo`
- `compare`

## Output/export behavior

### `soo`

Current `soo` mode exports:
- `comparison.csv`
- `soo_parameters.csv`
- `soo_run.json`

Contents include:
- baseline result
- raw best result
- best feasible result if found
- decoded parameter values

### `moo`

Current `moo` mode exports:
- `pareto_front.csv`
- `pareto_parameters.csv`
- `moo_run.json`

Contents include:
- final population
- feasibility flags
- decoded parameter values
- objective vectors

### `compare`

Current `compare` mode exports both the SOO and MOO output sets, plus compare-level summary artifacts.

## Verification done

Most verification in this environment was done with:
- `clangd --check`
- source inspection
- added smoke/regression tests

Coverage added during the refactor includes:
- attachment graph assembly
- bonded contact-policy defaults
- primitive clearance
- shared parameter consumers
- active parameter mapping
- element-local constraint registration
- `cT` affecting yaw allocation terms
- custom composition API with copy-preserving graph semantics
- pagmo problem metadata
- NSGA-II invalid population-size rejection

Important note:
- some `clangd --check` runs still report internal `ExtractFunction` tweak noise on larger files or test files
- those are not compile diagnostics

## Known limitations / remaining gaps

### 1. MATLAB parity is structural and logical, not exact

The project preserves the Stage 1 workflow and reduced-order formulas, but not exact optimizer trajectories.

Reasons:
- C++ uses pagmo `cmaes` / `nsga2`
- MATLAB reference uses MATLAB GA/gamultiobj and custom CMA-ES logic

### 2. Relationship semantics are still intentionally small

The framework now has explicit attachment relationships, but the built-in relationship vocabulary is still compact:
- rigid mount
- local offset
- mirrored local offset

Not yet implemented as first-class relationship objects:
- coaxial constraints
- concentric constraints
- face-to-face mount rules
- clearance relationships

### 3. Constraint categories are still generic callbacks

Constraints are flexible and extensible, but they are not yet separated into strong typed families such as:
- element-local geometric constraints
- assembly relationship constraints
- architecture-global constraints
- optimization-only penalty constraints

### 4. Allocation/hover logic is still Stage-1-specific

The front-end architecture is now more open, but the control-allocation and hover-feasibility math is still six-rotor Stage 1 logic.

That means the framework is more composable structurally than it is aeromechanically.

### 5. Geometry is still reduced-order

The current model is geometry-aware, but still reduced-order:
- no CAD
- no mesh collision
- no flexible-structure model

This is intentional for the Stage 1 scope.

### 6. Runtime build/executable validation is still environment-limited

I verified structure and compile health with `clangd`, but I did not run a full MSBuild or CMake build in this environment after every refactor step.

### 7. CLI controls are still limited

Currently configurable:
- mode
- output directory
- SOO population
- SOO generations
- MOO population
- MOO generations

Not yet exposed:
- objective selection from CLI
- objective weights
- seed
- active-variable toggles

## Recommended next steps

1. Rebuild the Visual Studio project and run `eval`, `soo`, `moo`, and `compare` against the current composition-capable architecture.
2. Add one or two small custom-graph examples outside the default hex builder path to exercise the new public API.
3. Expand first-class relationship objects if new topologies need richer geometric semantics.
4. Continue moving Stage 1 physics toward capability-driven logic where useful.
5. If broader vehicle classes are needed, generalize the allocation and hover solver away from the fixed six-rotor model.

## Example commands

```powershell
.\FrameworkCpp.exe eval
.\FrameworkCpp.exe soo --soo-pop 64 --soo-gen 100 --output-dir C:\local\project\UAMOptimization\FrameworkCpp\output
.\FrameworkCpp.exe moo --moo-pop 64 --moo-gen 100 --output-dir C:\local\project\UAMOptimization\FrameworkCpp\output
.\FrameworkCpp.exe compare --soo-gen 80 --moo-gen 100 --output-dir C:\local\project\UAMOptimization\FrameworkCpp\output
```
