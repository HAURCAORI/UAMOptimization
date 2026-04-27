---
name: project_overview
description: Full project context for UAM hexacopter fault-tolerant MDO framework — AE50001 Team 5 2026 Spring
type: project
---

## Project Identity
- Course: AE50001 MDO (Multi-Disciplinary Optimization)
- Team: Team 5, 2026 Spring
- Topic: "Systematic Design of Hexacopter UAM under Rotor Fault Situations"
- Goal: MDO framework that finds hexacopter designs with best fault tolerance at minimum structural/motor cost

## Working Directory
`C:\Project\UAMOptimization\`

## Repository Structure
```
Src/                          ← ORIGINAL scripts (DO NOT MODIFY)
  sim_control.m               Attitude/altitude control sim with LOE fault injection
  sim_path_following.m        Figure-8 path following simulation
  acs_analysis.m              Attainable Control Set analysis (convex hull)

Framework/                    ← NEW MDO framework (built this session)
  main_mdo.m                  Entry point — run this
  MANUAL.md                   Full user manual
  core/
    design_default.m          Baseline design vector struct
    hexacopter_params.m       Design → UAM/Prop/Env structs + parametric inertia
    build_B_matrix.m          4×6 control effectiveness matrix
    eom_hex.m                 6-DOF rigid body equations of motion
  evaluation/
    eval_acs.m                ACS-based fault tolerance metrics
    eval_simulation.m         Closed-loop cascade PI sim with fault injection
    eval_design.m             Master evaluator (ACS + sim, returns scalar objectives)
  metrics/
    compute_acs_volume.m      4D convex hull volume via 64 vertices
    hover_feasibility.m       LP feasibility: can vehicle hover after fault?
    compute_hover_threshold.m Min T_max per fault via single LP (max-λ formulation)
    fault_isotropy_index.m    FII = std(r_i)/mean(r_i) across 6 single faults
  optimization/
    objective_fcn.m           Scalar objective wrapper for MATLAB optimizers
    constraint_fcn.m          Physical feasibility nonlinear constraints
    run_soo.m                 GA + fmincon polish, live convergence figure
    run_moo.m                 gamultiobj Pareto, live scatter update
  analysis/
    sweep_design_space.m      1D/2D parameter sweeps
    pareto_analysis.m         Domination filter + knee point (max dist to ideal line)
    compare_designs.m         Side-by-side table + radar chart
    visualize_results.m       ACS 3D plots, sim histories, sweep curves, Pareto scatter
```

## Vehicle Configuration
- Tandem hexacopter (3 rows × 2 motors), NED convention
- Motor layout: (±Lx, ±Lyi) front/rear + (0, ±Lyo) mid
- Spin: PPNNPN → yaw row = [-cT, -cT, +cT, +cT, -cT, +cT]
- Baseline: m=2240.73 kg, Lx=Lyi=2.65 m, Lyo=5.5 m, T_max=7327 N, cT=0.03

## Key Design Findings Discovered This Session
1. Mean single-fault ACS retention = 1/3 exactly (zonotope identity, geometry-invariant)
2. cT has zero effect on retention or FII
3. Motors 3 and 5 always need T_max ≥ 3.0×(m·g/6) for hover after failure (geometry-invariant)
4. Baseline T_max = 2.0× → motors 3,5 cannot hover after fault (confirmed by both LP and simulation)
5. FII minimized at Lyi ≈ 4.0 m (from 1D sweep)
6. Optimal T_max sweet spot ≈ 11,100 N (just above 3.0× threshold)
7. Pareto trade-off: FII improvement (larger Lyi) vs structural cost (smaller Lyi)
