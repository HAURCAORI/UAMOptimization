# Agent Project Brief

## Project Title
**Systematic design of the hexacopter (UAM) under fault situation**

## Overview
This project is not mainly about implementing a fixed controller architecture or reproducing one exact simulation workflow.  
The main purpose is to build a MATLAB-based framework that can answer a higher-level design question:

> **How should a hexacopter UAM be designed so that it maintains acceptable performance under rotor fault conditions?**

The optimization target is therefore **system design**, not only control input generation.
This is for MDO course project and we need to execute the multi design optimization.
Therefore the objective function should be a combination of the performance metrics and the design variables.
The optimization purpose is to find the best design variables that can feasible control authority under fault.
=> Safety but cost effective design study

The existing MATLAB files already provide useful starting points:
- a baseline closed-loop simulation with low-level control and fault injection
- a mission-style path-following simulation with measurable tracking performance
- and an attainable-control-set / feasible-control-region analysis under actuator faults

The current project direction has also shifted away from a pure trajectory-control topic toward a **design-oriented fault study**, while still allowing obstacle- or mission-related performance to appear as one evaluation scenario when useful.

---

## Core Intent
Agent should treat this project as the creation of a **design exploration and optimization environment**.

The important question is not:

- “What is the exact implementation sequence?”
- “Which controller structure must be used without deviation?”

The important question is:

- **“What quantities do we want to obtain from optimization?”**
- **“How can MATLAB evaluate those quantities consistently for a candidate design?”**

So the implementation should remain flexible as long as it supports meaningful optimization and preserves consistency with the existing project direction.

---

## Main Goal of Optimization
The framework should allow us to compare candidate hexacopter designs under rotor-fault situations and identify designs that are better according to selected performance measures.

A design may be evaluated from one or both of the following viewpoints.

### 1. External performance viewpoint
A candidate design is judged by how well it performs in a representative simulation scenario.

Examples:
- following a prescribed path,
- recovering after a rotor fault,
- maintaining altitude or attitude,
- executing an emergency maneuver,
- or flying a task in an obstacle-aware environment.

### 2. Internal capability viewpoint
A candidate design is judged by the size or quality of its feasible control authority under fault.

Examples:
- attainable control set size,
- hover-feasible wrench region,
- roll/pitch moment authority after fault,
- controllability-related geometric margins.

These two viewpoints may be used separately or combined.

---

## What We Want to Get from Optimization
For a given set of design variables, the framework should produce **interpretable scalar metrics** that help answer system-design questions.

Typical outputs of interest include:

### A. Mission / simulation outputs
- whether the vehicle can complete the scenario,
- tracking error or trajectory deviation,
- recovery quality after fault,
- actuator saturation usage,
- attitude excursion,
- control effort,
- stability or divergence indicators,
- touchdown or terminal-state quality if landing is considered.

### B. Feasible-control-region outputs
- projected attainable-control volume,
- cross-sectional hover capability,
- faulted control-region shrinkage,
- comparison between nominal and faulted configurations,
- margins related to safe controllability.

### C. Design trade-off outputs
- which parameters improve fault tolerance,
- which parameters enlarge feasible control authority,
- which parameters reduce mission failure,
- whether one design improves internal controllability but worsens external performance.

The optimizer should ultimately help us identify:
- **good designs,**
- **bad designs,**
- and **the trade-offs between robustness, controllability, and mission performance.**

---

## Recommended Project Interpretation
Agent should interpret the project as a **system-level design study** with MATLAB as the evaluation engine.

This means:
- the simulation is a tool,
- the ACS / feasible-region analysis is a tool,
- the controller is a tool,
- and the final interest is the **design parameter effect under fault**.

The implementation should therefore emphasize:
- reusable evaluation,
- configurable scenarios,
- clear metrics,
- and optimization readiness.

---

## Acceptable Evaluation Modes
Agent may structure the project in any reasonable way, but it should support at least these styles of evaluation.

### Mode A — Scenario-based evaluation
Use a simulation scenario to evaluate a design.

Examples:
- nominal hover,
- commanded attitude sequence,
- figure-8 or other prescribed path,
- fault occurrence during flight,
- simplified obstacle-aware mission,
- emergency recovery case.

The scenario does not have to be fixed forever.  
The framework should allow scenarios to evolve.

### Mode B — Capability-based evaluation
Use feasible-control-region or attainable-control-set analysis to evaluate a design.

Examples:
- overall virtual-control-region size,
- faulted hover-feasible set,
- roll/pitch authority after a failed rotor,
- relative degradation ratio under fault.

### Mode C — Combined evaluation
Use both simulation performance and internal control capability together.

This is especially useful if one design flies well in one scenario but has poor intrinsic fault margin, or vice versa.

---

## Design Variables
Agent should keep the design-variable set flexible.

Possible examples:
- arm lengths,
- motor thrust capability,
- moment-to-thrust ratio,
- total mass or simplified mass scaling,
- inertia-related parameters,
- actuator configuration,
- controller gain scales,
- fault allocation parameters,
- or other simplified system parameters that influence faulted performance.

The exact initial design vector does **not** need to be fixed in advance.  
Agent may start with a compact and practical subset, then expand if needed.
If possible implement the design variables as a struct or class, then the design variables can be easily modified and evaluated.

The essential requirement is that:
1. the design variables can be mapped into the MATLAB model, and  
2. each candidate design can be evaluated consistently.

---

## Relationship to Existing Code
The prior scripts should be treated as **reference assets**, not as rigid constraints.

### Existing assets provide:
- vehicle and propulsion parameter conventions, fileciteturn1file0turn1file1
- fault injection and command allocation ideas, fileciteturn1file0
- path-following evaluation ideas, fileciteturn1file1
- attainable control set analysis and actuator configuration comparison. fileciteturn1file2turn1file3

### Agent should preserve:
- useful conventions,
- physically meaningful sign conventions,
- consistency of actuator configuration interpretation,
- and comparability with prior results.

### Agent does **not** need to preserve:
- one-script structure,
- one exact controller form,
- one exact optimization driver,
- or one exact file organization.

Refactoring and redesign are allowed if they improve clarity and optimization usability.

---

## Desired Characteristics of the New Framework
The new MATLAB project should ideally have these properties:

### 1. Modular
Simulation, control, fault handling, region analysis, and optimization should be separable.

### 2. Configurable
A user should be able to change:
- scenario,
- fault case,
- design variables,
- evaluation weights,
- and optimization settings
without rewriting the whole project.

### 3. Optimization-oriented
The code should make it easy to pass a candidate design into an evaluation function and obtain a scalar objective plus supporting metrics.

### 4. Interpretable
Results should not be just “best value found.”  
They should also explain why the design is better:
- smaller tracking error,
- larger feasible region,
- lower saturation,
- better recovery after fault, etc.

### 5. Extendable
Later additions such as obstacle terms, linear MPC, or additional scenarios should be possible without major restructuring.

---

## Role of Obstacle / Mission Content
Obstacle-related or trajectory-related content may be included as **one evaluation scenario**, but it should not dominate the project definition.

If obstacle-aware simulation is useful, it should serve this broader purpose:

- evaluate whether a design still performs acceptably in a constrained mission after fault.

So obstacle handling is optional as a scenario layer, not the only identity of the project.  
This is consistent with the shift toward a system-design topic rather than only trajectory optimization. fileciteturn2file1

---

## Optimization Philosophy
Agent should focus on producing a framework where optimization can answer questions such as:

- Which motor/arm configuration gives better fault tolerance?
- Which design retains more feasible control authority after a rotor fault?
- Which parameter changes improve external mission performance most effectively?
- Is there a trade-off between internal controllability and external task performance?
- Which design variables are most sensitive?

The optimizer itself is not the main contribution.  
The contribution is the **design insight obtained through structured evaluation**.

---

## Suggested Deliverables
Agent should choose a practical implementation strategy, but the final framework should ideally provide:

### Minimum
- a reusable method to evaluate one candidate design,
- at least one simulation-based metric,
- at least one internal capability metric,
- and one optimization entry point.

### Better
- separate evaluation modes,
- support for multiple fault cases,
- comparison plots,
- and automated reporting of key metrics.

### Best
- combined objective support,
- sensitivity or parameter sweep tools,
- and a clean structure that can later incorporate more advanced control or mission scenarios.

---

## What Agent Should Optimize For
When making implementation choices, prefer:
1. **clarity of evaluation objective,**
2. **flexibility of design-variable handling,**
3. **reusability of simulation and analysis,**
4. **ease of comparing candidate designs,**
5. **reasonable runtime.**

Avoid overfitting the code to one early scenario.

---

## What Not to Assume Too Early
Agent should avoid assuming, unless clearly needed:
- one final controller architecture,
- one exact path planner,
- one exact state definition for all future versions,
- one final objective function,
- or one permanently fixed file structure.

Those details may evolve as the project matures.

---

## Final Instruction to Agent
Please treat this project as the development of a **MATLAB-based optimization framework for fault-aware hexacopter system design**.

Refer to the existing scripts as guidance, preserve meaningful existing assumptions, and build whatever modular structure is most suitable for:

- evaluating candidate designs,
- extracting interpretable performance metrics,
- and enabling optimization-driven design comparison under rotor-fault conditions.

The framework should help us understand **which designs are better and why**, rather than merely reproducing a single simulation case.

Do not modify the existing scripts, and create new directories for the new framework.
Hybrid design is acceptable, if the performance is crucial use external tools such as GPOPS, CMA-ES, and C++ codes for the optimization.
Optimization should be done in MATLAB, and the optimization results should be visualized and reported.

**IMPORTANT:** This whole project is for MDO course project, and the originality and creativity of the project will be evaluated.
Therefore if possible search the literatures and find out the best practice and apply them to the project. Also the code should be well documented and well-organized. Divide the project into smaller modules and make the project reusable and extendable.

## Future Improvement
- If possible, add the feature about 3D drone model generation which parametically defined the aircraft geometry
  This can be used for the check for feasiblity and performance evaluation.
  Since this optimization is for MDO objective, structure + control + safety should be considered simultaneously.
  This 3D model generation can be used for the design optimization and evaluate the physical parameters such as mass, inertia, and center of gravity.
- If optimization is clear and well-defined, add the feature about the optimization-based control design.