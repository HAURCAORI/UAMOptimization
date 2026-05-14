# AUDIT_AGENT_INSTRUCTIONS.md

## Purpose

This document defines the audit instructions for a **dedicated review agent** assigned to inspect the current C++ project for:

- architectural correctness,
- compliance with the intended framework requirements,
- extensibility of the `SpatialElement`-based design,
- consistency of parameter / constraint / assembly behavior,
- and practical readiness for Stage 1 optimization work.

The current project is **Stage 1 only**.  
Do **not** audit it against Stage 2, long-horizon mission simulation, FEM, CAD/mesh geometry, or GUI requirements.

---

## Audit Scope

The audit target is the current C++ framework version (v4-style architecture baseline), which is intended to support:

- layered C++ project structure,
- `SpatialElement`-based architecture composition,
- attachment-aware geometry assembly,
- reduced-order Stage 1 physics,
- pagmo2-backed SOO/MOO optimization,
- analysis/report/export flow.

The audit must check both:

1. **Project requirements**: what the framework is supposed to support.
2. **Current implementation status**: what is already implemented and whether it satisfies the requirements.

The agent must clearly distinguish:

- **implemented and correct**
- **implemented but limited**
- **partially implemented**
- **missing**
- **implemented but architecturally risky**

---

## Hard Audit Rules

1. **Do not redesign the whole project immediately.**
   - Prefer identifying whether the current implementation can be improved incrementally.

2. **Preserve the Stage 1 project boundary.**
   - Do not mark the absence of Stage 2 as a defect.

3. **Audit the framework structure, not just formulas.**
   - A numerically plausible formula is not enough if ownership and extensibility are wrong.

4. **Prioritize extensibility and responsibility separation.**
   - The core question is whether the framework can support future element and topology extension without rewriting core files.

5. **Be strict about fake extensibility.**
   - Public interfaces do not count as extensible if the real behavior still depends on hard-coded default builder logic.

6. **Prefer static architecture correctness over speculative performance criticism.**
   - Only flag performance problems when they are structurally meaningful.

---

## Project Requirements

## 1. Project Goal

The framework must support:

**Fault-tolerant hexacopter architecture co-design using simplified spatial element modeling**

The project must provide a reusable C++ foundation for optimizing a reduced-order hexacopter architecture under Stage 1 objectives, with emphasis on:

- sizing,
- placement,
- packaging,
- fault tolerance,
- structural proxies,
- and optimizer-safe parameter handling.

---

## 2. Required Layered Structure

The project should be organized into these layers:

### `core/`
Responsible for:
- geometry primitives,
- parameters,
- constraints,
- attachments,
- assembly state,
- spatial elements,
- architecture object,
- default architecture builder.

### `physics/`
Responsible for:
- mass / COM / inertia aggregation,
- control allocation,
- faulted allocation,
- packaging / clearance evaluation,
- structural proxies,
- reduced-order Stage 1 vehicle property estimation.

### `evaluation/`
Responsible for:
- Stage 1 metrics,
- objective aggregation,
- constraint evaluation,
- architecture evaluation context/results.

### `optimization/`
Responsible for:
- design-vector mapping,
- bounds extraction,
- optimization problem wrappers,
- pagmo2 integration,
- SOO / MOO runners.

### `analysis/`
Responsible for:
- comparison reporting,
- Pareto processing,
- CSV/JSON export.

### `app/`
Responsible for:
- executable entry,
- CLI modes,
- runnable example workflows.

### Audit requirement
The agent must verify that responsibilities are actually separated, not just folder-renamed.

---

## 3. Core Model Requirements

### 3.1 Canonical shared design parameters
The framework must have a canonical parameter system where:

- the registry owns parameters,
- elements bind to shared parameter instances,
- one parameter may have multiple consumers,
- only active registered parameters are exposed to the optimizer,
- architecture updates propagate consistently to elements and assembly.

### 3.2 SpatialElement extensibility
`SpatialElement` must support extension through public derived classes.

At minimum, the agent should verify whether the framework can support:
- body,
- battery,
- payload,
- arm,
- motor,
- rotor

as public element classes.

Elements should support:
- cloning,
- parameter registration or parameter rebinding,
- local constraint registration,
- local geometry definition,
- local mass / COM / inertia contribution,
- parameter-driven state update.

### 3.3 Public composition API
The architecture object must expose public graph-editing APIs such as:
- add/remove element,
- add/remove attachment,
- add/remove constraint,
- rebuild assembly.

The audit must verify that external code can create a custom architecture graph without modifying the default builder source.

### 3.4 Copy/assignment semantics
Copy and assignment must preserve:
- parameters,
- elements,
- attachments,
- constraints,
- and the actual custom graph structure.

The audit must flag any implementation that silently reconstructs the default architecture during copy.

---

## 4. Geometry and Assembly Requirements

### 4.1 Geometry primitives
The framework should support simple reduced-order geometry primitives such as:
- sphere,
- box,
- cylinder,
- disk,
- segment.

### 4.2 Attachment-aware assembly
Assembly must be explicit and graph-based.

The framework should support:
- parent/child attachments,
- named anchors or anchor frames,
- assembled world poses,
- graph validation.

The audit should check for:
- missing parent/child detection,
- multiple-parent detection,
- cycle detection,
- orphan detection.

### 4.3 Relationship model
Attachments should be more than raw transforms.

The framework should support first-class relationship semantics such as:
- rigid mount,
- local offset,
- mirrored local offset.

Additional relationship types may be absent, but the agent must check whether the design can be extended without rewriting architecture core.

### 4.4 Contact policy and packaging exemptions
Attachments should be able to define contact/clearance policy such as:
- enforce clearance,
- allow touch,
- bonded overlap.

The audit must verify whether packaging checks can exempt bonded or allowed-contact pairs using data from the attachment graph rather than ad hoc hard-coded logic.

---

## 5. Constraint System Requirements

### 5.1 Constraint registration
Constraints must be first-class objects with:
- stable identity,
- owner id,
- name,
- callback/evaluator behavior,
- hard/soft usage compatibility.

### 5.2 Constraint sources
The project should support both:
- architecture-global constraints,
- element-local constraints.

### 5.3 Audit requirement
The agent must verify that constraint evaluation is not still implemented as fragile string-based dispatch in the Stage 1 evaluator.

---

## 6. Physics Requirements

The framework is **Stage 1 reduced-order**, not high fidelity.

The physics layer must support:

### 6.1 Mass properties
- total mass
- center of mass
- inertia aggregation from assembled elements

### 6.2 Rotor / control allocation
- rotor positions from assembled elements
- control allocation matrix
- faulted allocation matrix
- rotor yaw sign handling
- thrust reserve proxy
- fault allocation quality proxy

### 6.3 Structural proxy
- reduced-order structural metric
- not FEM

### 6.4 Packaging / clearance
- primitive-pair clearance checks
- packaging proxy
- use of graph/contact exemptions when applicable

### 6.5 Capability-driven direction
The physics layer should increasingly depend on **capabilities** rather than exact concrete element classes.

Examples:
- propulsion rotor capability
- structural member capability
- payload mass capability
- motor mass capability

The audit should judge whether the project is moving in this direction or still tightly hard-wired to exact classes.

---

## 7. Evaluation Requirements

The project is **Stage 1 only**.

The Stage 1 evaluator should support:
- mass objective term
- power objective term
- fault thrust penalty/proxy
- fault allocation penalty/proxy
- nominal hover proxy
- structural proxy
- packaging proxy

The evaluation layer should:
- aggregate objective terms,
- evaluate registered constraints,
- preserve constraint identity in results,
- keep Stage 1 logic separated from `core/` and `physics/`.

---

## 8. Optimization Requirements

### 8.1 Backend
The project should use **pagmo2** as the optimization backend.

### 8.2 Supported optimization modes
- SOO via pagmo `cmaes`
- MOO via pagmo `nsga2`

### 8.3 Mapping requirements
The optimization layer must:
- map only active registered parameters,
- expose deterministic bounds,
- decode vectors back into architecture state.

### 8.4 Feasibility handling
Constraint infeasibility may currently be handled via deterministic penalties, but the audit should verify:
- stable constraint lookup,
- correct feasible/raw solution separation,
- valid NSGA-II population guards.

For NSGA-II, population size should be checked for:
- minimum size,
- multiple-of-4 requirement.

---

## 9. Analysis / App Requirements

### 9.1 Analysis
The framework should provide:
- comparison reporting,
- Pareto analysis,
- CSV/JSON export.

### 9.2 App layer
The framework should provide a runnable entry with CLI modes such as:
- `eval`
- `soo`
- `moo`
- `compare`

The audit should note whether source files for app/build are present in the reviewed archive or only described in notes.

---

## 10. Out-of-Scope Features

The following are intentionally out of scope and must not be treated as defects:

- Stage 2 mission simulation
- nonlinear recovery simulation
- FEM
- CAD / mesh geometry
- GUI / viewer
- exact MATLAB optimizer trajectory reproduction

---

## What Is Reported As Currently Implemented

The following items are expected to be present in the current implementation baseline:

### Core
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

### Public composition
- add/remove element
- add/remove attachment
- add/remove constraint
- assembly rebuild

### Elements
- `BodyElement`
- `BatteryElement`
- `PayloadElement`
- `ArmElement`
- `MotorElement`
- `RotorElement`

### Geometry / assembly
- anchor-based attachments
- relationship kinds:
  - rigid mount
  - local offset
  - mirrored local offset
- contact policies:
  - enforce clearance
  - allow touch
  - bonded overlap

### Physics
- reduced-order mass/COM/inertia aggregation
- rotor extraction from assembled graph
- reduced-order control allocation / fault allocation
- structural proxy
- primitive clearance
- capability use for rotor / structure / payload / motor contributions

### Evaluation
- Stage 1 metric evaluation and objective aggregation
- callback-based constraints
- stable constraint ids in results

### Optimization
- `DesignVectorMapper`
- `BoundsBuilder`
- `OptimizationProblem`
- `PagmoProblemAdapter`
- `SooRunner`
- `MooRunner`
- SOO = CMA-ES
- MOO = NSGA-II
- feasible/raw result separation
- NSGA-II population validation

### Analysis / app
- comparison reporting
- Pareto analysis
- CSV/JSON export
- CLI entry with `eval`, `soo`, `moo`, `compare`

The audit must verify these claims against the actual codebase and report any mismatch.

---

## Expected Remaining Limitations

The agent should not over-report the following as critical defects if they are intentionally still present:

1. The relationship vocabulary is still small.
2. Constraints are flexible callbacks, not strongly typed categories.
3. Physics is still Stage-1 / six-rotor oriented.
4. Geometry is still reduced-order, not CAD-based.
5. CLI controls may still be limited.
6. Runtime build verification may be unavailable in the current review environment.

These are acceptable limitations unless implemented in a way that breaks extensibility or correctness.

---

## Required Audit Output Format

The dedicated audit agent must produce output in this structure:

### 1. Executive summary
- overall status
- whether the framework is acceptable as the current Stage 1 base
- whether another rewrite is necessary

### 2. Requirement compliance table
For each major requirement group:
- status: `pass` / `partial` / `fail`
- short rationale

Groups:
- layered structure
- parameter system
- public element extensibility
- public composition API
- copy/assignment semantics
- geometry and assembly
- attachment relationships
- contact policy / packaging exemptions
- constraint system
- physics modularity
- Stage 1 evaluation
- pagmo2 optimization integration
- analysis/app support

### 3. Verified strengths
List concrete strengths that are genuinely implemented.

### 4. Remaining architectural gaps
List the most important gaps only.

### 5. Risk assessment
Classify:
- low-risk issues,
- medium-risk issues,
- high-risk issues.

### 6. Recommended next actions
Only recommend **incremental** next actions unless the framework is fundamentally unsalvageable.

---

## Required Audit Style

The dedicated agent must be:

- technically strict,
- concise,
- explicit about uncertainty,
- honest about what was statically inspected versus what was actually built/run.

The agent must clearly say when:
- something was inferred from source structure,
- something was confirmed in code,
- something could not be runtime-verified.

Do not exaggerate missing features that are intentionally out of scope.
Do not praise superficial OOP patterns unless they actually improve extensibility.
Do not recommend another rewrite unless the current version is fundamentally broken.
