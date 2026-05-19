# UAMOptimization — Claude Code Instructions

This repository is a Stage 1 reduced-order co-design framework for a fault-tolerant hexacopter UAM.
It has two sub-projects:

- **`Framework/`** — MATLAB reference implementation (optimization, evaluation, plotting)
- **`FrameworkCpp/`** — C++ production implementation (pagmo2 optimizer, ACS analysis, SVG plotter)

---

## Mandatory: Read CODEBASE.md before touching FrameworkCpp

Before modifying **any** file under `FrameworkCpp/`, read:

```
FrameworkCpp/docs/CODEBASE.md
```

It contains the authoritative directory layout, class responsibilities, data flow, design
parameters (with correct defaults and bounds), constraint IDs and penalties, ACS invariants,
and known gotchas. Do not rely on memory — the file is ground truth. After every implementation
session that changes architecture, update CODEBASE.md to reflect what changed.

## Mandatory: Consult Action.md for any feature addition

When adding a new **parameter, constraint, metric, element type, evaluator, or output format**,
follow the step-by-step checklist in:

```
FrameworkCpp/docs/Action.md
```

**Critical rule enforced there:** every new `.cpp` or `.hpp` file must be registered in
`FrameworkCpp/FrameworkCpp.vcxproj` (both `<ClCompile>` and `<ClInclude>` item groups).
Skipping this causes `LNK2019` linker errors with no indication of which file is missing.
See Section G of Action.md for the exact procedure.

---

## Project context

### Vehicle
Fixed-topology hexacopter (6 rotors, positions fixed). Design variables: arm geometry (Lx, Lyi,
Lyo), max thrust per motor (T_max), arm tube cross-section. NED convention: z-down, upward thrust
= negative Fz. B-matrix row 0 = [−1, −1, −1, −1, −1, −1].

### Implementation phases
- **Phase 1 (done):** ACS analysis — B matrix, trim LP, directional margins, volume metrics,
  per-axis reserves, soft margin objective, named fault constraint.
- **Phase 2 (done):** Powertrain limits (actuator-disk power model) and battery sizing
  (`m_bat` design variable, `battery_energy_reserve` + `battery_crate_limit` constraints).
- **Phase 3 (pending):** Multi-member structural network and load propagation.
- **Phase 4 (pending):** Stiffness / deflection constraints.
- **Phase 5 (pending):** MetricRole enum and hard/soft metric labeling.

### ACS correctness invariant (never break)
```
margin(d, u_req) = h_U(d) - d' * u_req      ← correct (positive = feasible)
h_U(d)          = Σ_j max(0, d'·b_j)·f_max_j
```
- Origin `u=0`: all margins ≥ 0; thrust direction [+1,0,0,0] gives margin = 0 (boundary, NOT infeasible).
- Hover `u=[-mg,0,0,0]`: all margins > 0.
- Wrong sign (`d'·u − h_U`) always gives ≤ 0 at origin — do not use.

---

## MATLAB ↔ C++ alignment rules

When the MATLAB result and C++ result differ, the MATLAB reference wins unless there is a
documented reason (see `FrameworkCpp/docs/CODEBASE.md` invariants and memory file
`project_cpp_fixes.md`).

Key alignment fixes already made (do not revert):
1. Packaging clearance: rotor-only clearance check (not full element-pair).
2. T_max bounds: [8000, 20000] N; default 12000 N (starts optimizer inside ACS-feasible region, hover_margin≈+0.12); Lx/Lyi lower bound raised to 2.0 m; m_payload baseline 800 kg.
3. cT: frozen at 0.03 (active=false); MATLAB does not optimize cT.
4. L_ref for fault_alloc: `max(Lx, Lyi, Lyo)`, not `arm_span/6`.
5. Power model disk area: `r_eff = max_arm_length / 2` — do NOT use `d_prop` for power (it is frozen for yaw-torque cT only).

---

## Code style rules

- No comments explaining what the code does — only WHY (hidden constraint, workaround, invariant).
- No trailing summaries in responses — the user reads the diff.
- Windows `max`/`min` macro conflict: use `(std::numeric_limits<double>::max)()` (parens prevent
  macro expansion). Add `NOMINMAX` to preprocessor definitions for new configurations.
- All 4 vcxproj configurations must stay in sync (Win32/x64 × Debug/Release).
- Use `$(VcpkgTriplet)` in vcxproj include paths — never hardcode `x64-windows`.

---

## Key file locations

| File | Purpose |
|---|---|
| `FrameworkCpp/docs/CODEBASE.md` | Architecture reference — read first |
| `FrameworkCpp/docs/Action.md` | Checklists for adding params/constraints/metrics/elements; Section G: vcxproj registration |
| `FrameworkCpp/FrameworkCpp.vcxproj` | VS project file — every new `.cpp`/`.hpp` must be listed here |
| `FrameworkCpp/include/evaluation/EvaluationContext.hpp` | Objective weights, constraint thresholds |
| `FrameworkCpp/include/evaluation/EvaluationResult.hpp` | Stage1Metrics struct definition |
| `FrameworkCpp/src/evaluation/Stage1Evaluator.cpp` | Full evaluation pipeline |
| `FrameworkCpp/src/core/HexacopterArchitecture.cpp` | Design parameters and constraint registrations |
| `FrameworkCpp/include/physics/AttainableControlSetAnalyzer.hpp` | ACS structs (AcsCaseResult, AcsResult) |
| `Framework/main_test.m` | MATLAB ACS verification tests (T1–T6 + comparison figures) |

---

## Memory system

Persistent notes across sessions live in:
```
C:\Users\user\.claude\projects\C--local-project-UAMOptimization\memory\
```

Read `MEMORY.md` index there for the current state. Key files:
- `project_cpp_fixes.md` — MATLAB-alignment bugs fixed + Phase 1 ACS fixes
- `project_remaining_issues.md` — open bugs/gaps (B1–B3, D2–D3, C1, F1–F4, G1–G2)
- `project_vehicle_model.md` — MATLAB vehicle_model.m calibration constants
