---
name: project_progress
description: Current build status and what has been completed vs remaining for the UAM MDO framework
type: project
---

## Status: Framework Complete, Optimization Not Yet Run

### Completed This Session
- [x] All 20 framework .m files written and tested
- [x] MANUAL.md written (full user documentation)
- [x] Core evaluation pipeline verified in MATLAB:
      - compute_acs_volume: nominal vol = 7.49e16, motor 1 retention = 0.404
      - hover_feasibility: motors 3,5 infeasible at baseline T_max
      - eval_acs: PFWAR=0.302, FII=0.311, WCFR=0.200, hover_ok=4/6
      - eval_simulation: motor 1 fault → stable (alt_rmse=0.013m), motor 3 → diverges
      - eval_design: J_combined=0.648 for baseline
- [x] Design-space insights discovered via MATLAB sweeps:
      - Lyi sweep: FII min at Lyi≈4.0m
      - T_max sweep: hover_rate jumps at 11000N threshold
      - 2D grid: min J=0.431 at Lyi=1.5m, T_max=11455N (cost-optimal)
- [x] compute_hover_threshold rewritten: single LP (max-λ) replacing 40-iter binary search (~40× speedup)
- [x] eval_acs: include_double=false by default (3× speedup for optimization)
- [x] run_soo: live 3-panel convergence figure (conv curve + var bars + metrics)
- [x] run_moo: live Pareto scatter (f1-f2 + f1-f3 panels, updates per generation)
- [x] constraint_fcn: fixed Inf return bug for GA compatibility
- [x] main_mdo.m: full 6-section pipeline with section timers and banner output

### Not Yet Run (to be done on desktop)
- [ ] SOO optimization (GA + fmincon polish) — expected ~3–6 min
- [ ] MOO optimization (gamultiobj Pareto) — expected ~10–20 min
- [ ] Full simulation validation of optimal designs
- [ ] Final compare_designs table + radar chart
- [ ] ACS geometry figure (Section 6)

### Estimated Runtimes
- eval_design (ACS mode): ~15–30 ms/call
- SOO (50 pop × 80 gen = 4000 evals): ~3–6 min
- MOO (100 pop × 150 gen = 15000 evals): ~10–20 min

### Known Verified Numbers (baseline)
- FII = 0.311, WCFR = 0.200, PFWAR = 0.302
- hover_ok = 4/6 (motors 3,5 infeasible)
- hover_margin = −0.333 (T_max 33% below 3× threshold)
- T_hover_worst = 10,991 N (= 3.0 × m·g/6)
- J_combined = 0.648 at baseline
