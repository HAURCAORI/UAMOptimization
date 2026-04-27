---
name: codebase_details
description: Critical implementation details, function signatures, objective formulation, and design decisions for the UAM MDO framework
type: project
---

## Objective Function Design

J_combined = w1*J_FII + w2*J_hover + w3*J_mission + w4*J_cost
Default weights: [0.35, 0.40, 0.00, 0.25]

### J_FII = FII = std(r_i)/mean(r_i)
- r_i = ACS volume retention for motor i single fault
- Minimized at Lyi ≈ 4.0 m
- Baseline = 0.311

### J_hover
- = 0 when T_max ≥ T_hover_worst (hover margin ≥ 0)
- = 1 - exp(5 * hover_margin) when margin < 0 (exponential penalty)
- hover_margin = T_max / T_hover_worst − 1
- T_hover_worst = 10,991 N for baseline vehicle (always 3.0 × m·g/6)

### J_cost = J_struct * J_motor
- J_struct = arm_span / arm_span_baseline
- J_motor = (T_max / T_max_baseline)^1.5

## Critical Performance Optimization
compute_hover_threshold uses SINGLE LP per fault (not binary search):
  max λ  s.t.  [B_fault, -v_hover]*[T; λ] = 0,  0≤T≤T_max·(1-loe),  0≤λ≤1
  T_thresh = T_ref / λ_opt

This replaced 40-iteration binary search → ~40× speedup.

## Constraint Function (constraint_fcn.m)
c(1) = 1.10*m*g - 6*T_max           (hover margin ≥ 10%)
c(2) = -(Lyo - Lyi - 0.1)           (outer arm wider than inner)
c(3) = 0.05 - WCFR                  (ACS retains ≥5% after worst fault)
c(4) = max_util - 1.0               (hover utilization ≤1, capped at 10 if Inf)

## Inertia Model (hexacopter_params.m)
Calibrated to baseline (Lx=Lyi=2.65, Lyo=5.5, Ix=12000, Iy=9400):
  mm_Ix = 12000 / (4*2.65² + 2*5.5²) = 135.5 kg  (effective motor mass, roll)
  mm_Iy = 9400  / (4*2.65²)          = 335.0 kg  (effective motor mass, pitch)
Scaled design:
  Ix = mm_Ix * (4*Lyi² + 2*Lyo²)
  Iy = mm_Iy * (4*Lx²)
  Iz = Ix + Iy

## Default Variable Bounds for SOO/MOO
Lx:    [1.0, 5.0] m
Lyi:   [1.0, 5.0] m   (must stay < Lyo - 0.1)
Lyo:   [2.5, 9.0] m
T_max: [8000, 16000] N

## Key State Vector Layout (eom_hex.m)
X(1:3)   = [u,v,w]         body velocity [m/s]
X(4:6)   = [p,q,r]         angular rate [rad/s]
X(7:9)   = [phi,theta,psi] Euler angles [rad]
X(10:12) = [x,y,z]         NED position [m]  (z positive down)
X(13:15) = [vx,vy,vz]      NED inertial velocity (updated externally each step)

## ACS Vertex Generation (compute_acs_volume.m)
- 2^6 = 64 extreme thrust combinations (T_i ∈ {0, T_max_i})
- V_all = B * T_vertices  → 4D point cloud
- convhulln(pts_4d, {'Qt','Qx'}) → volume

## Simulation Controller (eval_simulation.m)
Cascade PI:
  Altitude:   outer P (tau_z=5s) → inner PI (Kp=2000, Ki=600)
  Attitude:   outer P (tau_phi=1s, tau_theta=1s, tau_psi=5s)
  Rate:       inner PI (Kp_p=60000, Ki_p=30000, etc.)
  Allocation: pinv(B_eff) * v_cmd
  Motor lag:  first-order, tau=0.04s
Default dt=0.01s (10ms) for optimization speed; use 0.005s for accuracy.

## eval_acs Default Settings
- include_double = false (single faults only for optimization speed)
- p_motor = 0.05 (5% per-motor failure probability)
- Hover threshold computed via compute_hover_threshold (single LP)

## Live Plot Architecture (run_soo.m)
3-panel figure:
  Panel 1: all J_all dots + cummin(J_all) blue line (convergence)
  Panel 2: normalized design variables bar chart (x-lb)/(ub-lb)
  Panel 3: FII, hover_deficit, J_cost bars at current best

## Pareto Knee Identification (pareto_analysis.m)
Method: max perpendicular distance from ideal (0,0,0) to antiideal (1,1,1) line
  F_norm = (F - f_min) / (f_max - f_min)
  line_dir = [1,1,1]/sqrt(3)
  knee_score(k) = ||F_norm(k) - dot(F_norm(k), line_dir)*line_dir||
  knee_idx = argmax(knee_score)
