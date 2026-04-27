---
name: todo_list
description: Outstanding tasks and future improvements for the UAM MDO framework
type: project
---

## Immediate TODO (next session — run on desktop)

### 1. Run main_mdo.m Section 3: SOO
```matlab
cd C:\Project\UAMOptimization\Framework
main_mdo  % run sections one at a time
```
- Expected result: d_opt.T_max ≈ 11,100N, d_opt.Lyi ≈ 3.5–4.0m
- Expected J_combined ≈ 0.42–0.45 (vs baseline 0.648)
- All 6 motors hover-feasible after any single fault

### 2. Run main_mdo.m Section 4: MOO (gamultiobj)
- Expected: ~30–50 Pareto solutions
- Key trade-off to see: FII decreases as Lyi increases, but J_cost increases
- Knee should land near Lyi ≈ 3–3.5m, T_max ≈ 11,200N

### 3. Validate with full simulation (mode='full')
- Run motor 3 fault scenario on optimized design (previously diverging on baseline)
- Check: alt_rmse, max_att_excurs, recovery_time, diverged=false

### 4. Run compare_designs for final report
- Baseline vs SOO-optimal vs Pareto-knee
- Motor 3 fault scenario (worst case)

## If Results Are Unexpected

### If SOO J_combined doesn't improve below ~0.50:
- Check constraint_fcn: c(4) must cap Inf at 10 (not return Inf)
- Try reducing max_iter to 40 and pop_size to 30 for a quick test first
- Try running fmincon directly from a manually set x0 near [2.65, 3.5, 5.5, 11100]

### If MOO Pareto front is degenerate (all on one edge):
- Increase ParetoFraction to 0.5 in gamultiobj options
- Check that all 3 objectives are returning varied values (not all 0 or all 1)

### If simulation diverges on optimized design with motor 3 fault:
- This means hover_margin > 0 but the controller still fails
- Check if dt=0.005 is used (not 0.01) for sim validation
- The PI controller may need gain retuning for the modified inertia

## Future Improvements (nice-to-have)

### High value
- [ ] Adaptive controller gain scaling with inertia changes
      (current controller uses fixed gains tuned for baseline inertia)
- [ ] Add Lx to optimization (currently often hits bounds; need wider bounds)
- [ ] Test with double motor fault (include_double=true for final analysis)
- [ ] Sensitivity analysis: Sobol indices or Morris screening

### Medium value
- [ ] 3D parametric drone geometry visualization (from Instruction.md future work)
      Plot motor positions, arm lengths as 3D stick model
- [ ] Path-following simulation scenario (figure-8) as mission metric
      Use sim_path_following.m as reference for controller architecture
- [ ] Multiple fault scenario simulation (run all 6 single faults, average metrics)

### Low priority
- [ ] CMA-ES integration (external, faster for high-dim problems)
- [ ] Export results to CSV/Excel for report generation
- [ ] MATLAB live script version of main_mdo.m for better presentation

## Design Recommendations for Paper

Based on completed analysis:

1. **Minimum viable design**: T_max = 10,991N (3× hover share), any Lyi
   → All single faults hover-feasible; baseline cannot guarantee this

2. **Isotropy-optimal**: T_max = 11,100N, Lyi ≈ 4.0m
   → FII minimized (0.284 vs 0.311); most balanced fault distribution

3. **Cost-optimal feasible**: T_max = 11,100N, Lyi = 1.0–1.5m
   → Lower structural cost; FII slightly higher (0.36) but acceptable

4. **Key message**: The baseline 2× T_max margin is insufficient for full
   fault-tolerant hover. A 3× margin is the minimum safety requirement,
   independent of arm geometry. Within this constraint, isotropy can be
   improved by choosing Lyi ≈ 4.0m.
