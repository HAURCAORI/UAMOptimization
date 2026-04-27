# UAM Hexacopter Fault-Tolerant MDO Framework
## User Manual

**AE50001 MDO Course Project — Team 5 — 2026 Spring**
*Systematic Design of Hexacopter UAM under Rotor Fault Situations*

---

## Table of Contents

1. [Terminology & Symbols](#1-terminology--symbols)
2. [Quick Start](#2-quick-start)
3. [Framework Overview](#3-framework-overview)
4. [Theoretical Background](#4-theoretical-background)
5. [Design Variables](#5-design-variables)
   - 5.1 Active design variables
   - 5.2 Fixed / derived fields
   - 5.3 Physics-based mass model (vehicle_model.m)
   - 5.4 Constraints enforced
6. [Configuration Reference](#6-configuration-reference)
   - 6.4 Vehicle Model
   - 6.5 Two-Stage Evaluation
   - 6.6 Evaluation Mode
7. [Module Reference](#7-module-reference)
8. [Optimization Workflow](#8-optimization-workflow)
9. [Key Metrics Explained](#9-key-metrics-explained)
10. [Performance Notes](#10-performance-notes)
11. [Design Findings Summary](#11-design-findings-summary)

---

## 1. Terminology & Symbols

### 1.1 Vehicle Geometry

| Symbol | Unit | Description |
|--------|------|-------------|
| Lx | m | Fore/aft arm length — distance from the body center to the front (or rear) motor row along the x-axis |
| Lyi | m | Inner lateral arm — distance from the center-line to each front/rear motor (rows 1 and 3) along the y-axis |
| Lyo | m | Outer lateral arm — distance from the center-line to each mid motor (row 2, motors M3/M4) along the y-axis; must satisfy Lyo > Lyi + 0.1 m |
| arm_span | m | Total arm span: 2·Lx + 2·Lyi + 2·Lyo (baseline = 21.6 m) |
| m | kg | Total vehicle mass — **derived** from design variables via vehicle_model.m (not fixed) |
| m_payload | kg | Fixed UAM payload budget: 1500 kg (set in design_default.m to activate physics-based model) |
| m_motor | kg | Per-motor mass — scales with T_max via Delbecq 2020 scaling law |
| m_frame | kg | Structural frame mass — scales linearly with arm_span |
| cT | — | Moment-to-thrust ratio: yaw moment per unit thrust, N = cT × T |
| d_prop | m | Propeller diameter (optional design variable; when set, cT = 0.03 × d_prop / 0.40) |

```
Top-view schematic (NED, z into page):

           M1         M2
          (−Lyi)    (+Lyi)
       ┌─────┬───────────┬─────┐
       │     │     x     │     │
       │     │     ↑     │     │   Lx
       │     │     │     │     │
  M3   ├─────┤     +─→y  ├─────┤   M4
(−Lyo) │     │           │     │ (+Lyo)
       │     │           │     │
       │     │           │     │   Lx
       └─────┴───────────┴─────┘
          M5         M6
         (−Lyi)    (+Lyi)

  Row 1 (front): M1, M2   at x = +Lx
  Row 2 (mid):   M3, M4   at x =   0
  Row 3 (rear):  M5, M6   at x = −Lx
```

### 1.2 Motor & Thrust

| Symbol | Unit | Description |
|--------|------|-------------|
| T_i | N | Thrust produced by motor i (i = 1…6) |
| T_max | N | Maximum thrust per motor (same for all motors; design variable) |
| T | N | Thrust vector [T₁; T₂; T₃; T₄; T₅; T₆], 6×1 |
| LOE_i | — | Loss-of-effectiveness of motor i: 0 = healthy, 1 = fully failed. Effective T_max_i = (1−LOE_i)·T_max |
| loe_vec | — | 6×1 vector of LOE values, one per motor |

**Spin directions (PPNNPN):**

| Motor | Position | Spin | Sign in B |
|-------|----------|------|-----------|
| M1 | front-left | CCW | −cT |
| M2 | front-right | CCW | −cT |
| M3 | mid-left | CW | +cT |
| M4 | mid-right | CW | +cT |
| M5 | rear-left | CCW | −cT |
| M6 | rear-right | CW | +cT |

### 1.3 Control Effectiveness

| Symbol | Dim | Description |
|--------|-----|-------------|
| B | 4×6 | Control effectiveness matrix. Maps thrust vector T to virtual control v |
| v = B·T | 4×1 | Virtual control vector [Fz; L; M; N] |
| Fz | N | Net vertical force (NED: positive = downward; negative = upward lift) |
| L | Nm | Roll moment (about x-axis) |
| M | Nm | Pitch moment (about y-axis) |
| N | Nm | Yaw moment (about z-axis) |

```
         ⎡ −1   −1   −1   −1   −1   −1 ⎤   Fz   ← sum of all thrusts (lift)
B  =     ⎢−Lyi  Lyi −Lyo  Lyo −Lyi  Lyi⎥   L    ← lateral moment arm × thrust
         ⎢  Lx   Lx    0    0  −Lx  −Lx⎥   M    ← fore/aft moment arm × thrust
         ⎣−cT  −cT  +cT  +cT  −cT  +cT⎦   N    ← yaw via spin direction
```

### 1.4 Attainable Control Set (ACS)

| Symbol | Description |
|--------|-------------|
| ACS | Attainable Control Set — the set of all virtual controls achievable by the vehicle: ACS = { B·T : 0 ≤ T_i ≤ T_max_i } |
| V_nom | Nominal ACS volume (4D, units N·Nm³). Computed with all motors healthy |
| V_k | ACS volume after motor k fails (LOE_k = 1) |
| r_k | Single-fault volume retention after motor k fails: r_k = V_k / V_nom |
| WCFR | Worst-Case Fault Retention: min(r_k) over k = 1…6 |
| PFWAR | Probabilistic Fault-Weighted ACS Retention: Σ P(fault k) · r_k |

### 1.5 Fault Tolerance Metrics

| Symbol | Formula | Description |
|--------|---------|-------------|
| FII | std(r₁…r₆) / mean(r₁…r₆) | **Fault Isotropy Index** — measures how unevenly fault impacts are distributed. FII = 0 means all faults are equally impactful (ideal). Higher FII means a "weakest-link" design |
| hover_margin | T_max / T_hover_worst − 1 | Normalized margin above the hover-feasibility threshold. ≥ 0 → vehicle can hover after any single fault; < 0 → some faults cause unrecoverable descent |
| T_hover_worst | N | Minimum T_max that makes single-fault hover feasible for every motor. For this vehicle: T_hover_worst = 3.0 × m·g / 6 ≈ 10,991 N |
| hover_util | — | Motor utilization at hover after fault: max(T_i / T_max_i). ≤ 1 means feasible |
| p_motor | — | Independent per-motor failure probability (default 0.05). Used to compute PFWAR |

### 1.6 Objective Function

The combined scalar objective minimized by the optimizer:

```
J = (w_FII · J_FII  +  w_hover · J_hover  +  w_mission · J_mission  +  w_cost · J_cost)
    ─────────────────────────────────────────────────────────────────────────────────────
                        w_FII + w_hover + w_mission + w_cost

  J_FII     = FII
  J_hover   = 0                          if hover_margin ≥ 0
            = 1 − exp(5 · hover_margin)  if hover_margin < 0   (∈ (0,1))
  J_mission = alt_RMSE / alt_cmd         (simulation mode only; else excluded)

  J_cost (physics-based, vehicle_model path):
            = (6·m_motor + m_frame) / cost_ref
              where cost_ref = 6 × 74.07 + 296.31 = 740.73 kg  (baseline)

  J_cost (legacy fixed-mass path, no m_payload):
            = (arm_span / arm_span_base) · (T_max / T_max_base)^1.5
```

NaN terms (e.g. J_mission in `mode='acs'`) are automatically excluded and their weights renormalized.

> **Physics-based J_cost** captures the self-consistent mass-propulsion coupling: a higher T_max increases motor mass (∝ T_max^(3/3.5)), which raises total vehicle mass, which raises the hover threshold, which demands even more T_max. The optimizer must find the equilibrium that satisfies all constraints at minimum cost.

| Weight | Default | Governs |
|--------|---------|---------|
| w_FII | 0.35 | Balance of fault impacts across motors |
| w_hover | 0.40 | Hover survivability after single fault |
| w_mission | 0.00 | Altitude tracking performance (disabled by default) |
| w_cost | 0.25 | Motor + structural cost (penalizes large arms and high T_max) |

### 1.7 Optimizer Symbols (CMA-ES)

| Symbol | Description |
|--------|-------------|
| x_norm | Design variable vector normalized to [0,1]: x_norm = (x_phys − lb) / (ub − lb) |
| x_phys | Physical design variable vector in engineering units |
| n | Number of active design variables |
| λ (lambda) | Offspring count per generation. Default: 4 + floor(3·log(n)) |
| μ (mu) | Number of parents (best offspring kept): μ = floor(λ/2) |
| σ (sigma) | Step size — controls search radius in normalized space |
| σ₀ | Initial step size (default 0.25 → searches within ±25% of each range) |
| C | n×n covariance matrix — encodes the shape and orientation of the search distribution |
| p_c, p_s | Evolution paths for covariance and step-size adaptation |
| μ_eff | Variance-effective selection mass: 1 / Σ w_i² |
| c_s, d_s | Step-size adaptation rate and damping |
| c_c, c₁, c_μ | Covariance update rates (rank-one and rank-μ) |
| χ_N | Expected length of a random unit normal vector in n dimensions: √n·(1 − 1/(4n) + 1/(21n²)) |

---

## 2. Quick Start

Run the full MDO pipeline from `main_mdo.m`:

```matlab
cd C:\Project\UAMOptimization\Framework
main_mdo
```

To run optimization with the default config:
```matlab
addpath('config','core','evaluation','metrics','optimization','analysis');
cfg    = mdo_config();          % load all settings
d_base = design_default();
[d_opt, J_opt, history] = run_soo(d_base, cfg);
```

To change anything (optimizer, weights, bounds, active variables) — edit `config/mdo_config.m` only. No other file needs to change.

To run only baseline evaluation:
```matlab
d = design_default();
r = eval_design(d, struct('mode','acs','verbose',true));
```

---

## 3. Framework Overview

```
Framework/
├── main_mdo.m                  ← Entry point (run this)
│
├── config/                     ← Master configuration (edit here only)
│   └── mdo_config.m            Hyperparameters, bounds, vehicle model, stage settings
│
├── core/                       ← Vehicle model
│   ├── design_default.m        Design vector (baseline; sets m_payload to activate physics model)
│   ├── vehicle_model.m         ★ DATCOM-style parameterized model (Delbecq 2020 scaling laws)
│   ├── hexacopter_params.m     Design → UAM/Prop/Env structs (dispatches to vehicle_model)
│   ├── build_B_matrix.m        Control effectiveness matrix B
│   └── eom_hex.m               6-DOF equations of motion
│
├── evaluation/                 ← Two-stage performance evaluation
│   ├── eval_stage1.m           ★ Stage 1: ACS feasibility + hover screening (~15 ms/eval)
│   ├── eval_stage2.m           ★ Stage 2: Mission simulation + recovery metrics
│   ├── eval_acs.m              ACS-based fault tolerance metrics
│   ├── eval_simulation.m       Closed-loop simulation metrics
│   └── eval_design.m           Master evaluator (orchestrates both stages)
│
├── metrics/                    ← Individual metric functions
│   ├── compute_acs_volume.m    4D convex hull volume of ACS
│   ├── hover_feasibility.m     LP check: can vehicle hover after fault?
│   ├── compute_hover_threshold.m  Min T_max required per fault (single LP)
│   └── fault_isotropy_index.m  FII: balance of fault impacts
│
├── optimization/               ← Optimizer drivers
│   ├── objective_fcn.m         Scalar objective wrapper for optimizers
│   ├── constraint_fcn.m        Nonlinear constraints (uses physics-based UAM.m)
│   ├── run_soo.m               Single-objective: CMA-ES / GA / fmincon
│   ├── run_cmaes.m             CMA-ES implementation (self-contained)
│   └── run_moo.m               Multi-objective: gamultiobj (Pareto)
│
└── analysis/                   ← Post-processing and visualization
    ├── sweep_design_space.m    1D / 2D parameter sweeps
    ├── pareto_analysis.m       Pareto filtering + knee identification
    ├── compare_designs.m       Side-by-side design comparison
    └── visualize_results.m     Unified plotting (ACS, sim, sweep, Pareto)
```

★ = added in current version

### Data Flow

```
mdo_config()  ─────────────────────────────────────────────────────┐
                                                                    ↓
design_default()                                              eval_design()
  (sets m_payload)                                          ┌──────┴───────┐
       ↓                                               Stage 1         Stage 2
  hexacopter_params()                               eval_stage1()   eval_stage2()
  ├─ vehicle_model()  ← DATCOM-style scaling            │              │
  │   (physics-based mass, inertia, cost)           eval_acs()  eval_simulation()
  └─ legacy fixed-mass path (no m_payload)
       ↓
  run_soo(d, cfg)  →  run_cmaes(d, cfg)  ←───────── eval_design()
       ↓
  pareto_analysis()  →  compare_designs()  →  visualize_results()
```

---

## 4. Theoretical Background

### 3.1 Vehicle Configuration

The vehicle is a **tandem hexacopter** (3 rows × 2 motors) using NED convention (z-down):

```
Motor layout (top view):
        M1 (+Lx, -Lyi)    M2 (+Lx, +Lyi)   ← front row
        M3 (  0, -Lyo)    M4 (  0, +Lyo)   ← mid row
        M5 (-Lx, -Lyi)    M6 (-Lx, +Lyi)   ← rear row

Spin direction (PPNNPN): M1,M2=CCW(-cT)  M3,M4=CW(+cT)  M5=CCW  M6=CW
```

### 3.2 Control Effectiveness Matrix

```
B = [  -1    -1    -1    -1    -1    -1  ]   Fz  [N]
    [-Lyi   Lyi  -Lyo   Lyo  -Lyi   Lyi]   L   [Nm]
    [ Lx    Lx    0     0    -Lx   -Lx  ]   M   [Nm]
    [-cT   -cT   +cT   +cT   -cT   +cT  ]   N   [Nm]
```

The virtual control vector **v = B · T** maps motor thrusts **T** (6×1) to
[Fz; L; M; N] — net force and moments in the body frame.

### 3.3 Attainable Control Set (ACS)

```
ACS = { B · T  :  0 ≤ T_i ≤ T_max_i }
```

- The ACS is a **zonotope** (polytope generated by line segments).
- With 6 motors and T_min=0, T_max fixed, the ACS has **2^6 = 64 vertices**.
- Volume is computed via `convhulln` in 4D [Fz, L, M, N] space.
- Under fault (LOE_i = 1): T_max_i → 0, collapsing that column of B.

### 3.4 Zonotope Retention Identity (novel finding)

> For any arm geometry, **mean(r_i) = 1/3** where r_i = V_fault_i / V_nominal
> is the ACS volume retention after motor i fails (i = 1…6).

**Consequence:** PFWAR (probabilistic fault-weighted retention) is approximately geometry-invariant. The meaningful geometric metric is therefore **FII** (the distribution of retentions), not their mean.

---

## 5. Design Variables

### 5.1 Active design variables (optimized)

| Field | Symbol | Baseline | Unit | Bounds | Description |
|-------|--------|----------|------|--------|-------------|
| `d.Lx` | Lx | 2.65 | m | [1.0, 5.0] | Fore/aft arm length (motors 1,2,5,6) |
| `d.Lyi` | Lyi | 2.65 | m | [1.0, 5.0] | Inner lateral arm (front/rear row) |
| `d.Lyo` | Lyo | 5.50 | m | [2.5, 9.0] | Outer lateral arm (mid row, motors 3,4) |
| `d.T_max` | T_max | 7327 | N | [8000, 16000] | Max thrust per motor |
| `d.d_prop` | d_prop | 0.40 | m | [0.30, 0.60] | Propeller diameter (**frozen** by default; set `active(5)=true` to optimize) |

### 5.2 Fixed / derived fields

| Field | Value | Description |
|-------|-------|-------------|
| `d.m_payload` | 1500.0 kg | Fixed UAM payload — activates physics-based vehicle model when set |
| `d.cT` | 0.03 | Moment-to-thrust ratio (overridden by d_prop if that variable is active) |
| `d.m` | *derived* | Total mass — computed by vehicle_model.m; kept in struct for backward compat |

### 5.3 Physics-based mass model (vehicle_model.m)

When `d.m_payload` is set (which is the default in `design_default.m`), all mass and inertia properties are derived from geometry and propulsion via Delbecq 2020 scaling laws [R4]:

```
m_motor = 74.07 × (T_max / 7327)^(3/3.5)          [kg per motor]
m_frame = 13.72 × arm_span                          [kg]
m_total = 1500 + 6·m_motor + m_frame               [kg]

Ixx = m_motor × (4·Lyi² + 2·Lyo²) + 5436          [kg·m²]
Iyy = m_motor × (4·Lx²)           + 7319          [kg·m²]
Izz = Ixx + Iyy                                    [kg·m²]
```

Calibration: at baseline (Lx=Lyi=2.65m, Lyo=5.5m, T_max=7327N) this yields m=2240.73 kg, Ixx=12000, Iyy=9400 kg·m² — matching the reference vehicle exactly.

> **Novel mass-propulsion coupling**: T_max → m_motor → m_total → hover threshold → required T_max. This feedback loop is resolved analytically at each evaluation, making the cost function physically consistent.

### 5.4 Constraints enforced (constraint_fcn.m)

- `6 × T_max ≥ 1.1 × m_total · g`  — hover margin using *computed* mass
- `Lyo > Lyi + 0.1`                 — outer arm wider than inner
- `WCFR ≥ 0.05`                     — ACS retains at least 5% after worst fault
- `hover_util ≤ 1`                  — T_max covers worst-case hover threshold

---

## 6. Configuration Reference

All hyperparameters live in **`config/mdo_config.m`**. Edit only this file to change any setting.

```matlab
cfg = mdo_config();
```

### 6.1 Objective Weights

```matlab
cfg.weights.FII     = 0.35;   % Fault Isotropy Index
cfg.weights.hover   = 0.40;   % Hover safety penalty
cfg.weights.mission = 0.00;   % Mission tracking RMSE (needs eval.mode='sim'/'full')
cfg.weights.cost    = 0.25;   % Motor + structural cost index
```

### 6.2 Design Variables

```matlab
cfg.vars.names  = {'Lx',  'Lyi', 'Lyo', 'T_max', 'd_prop'};
cfg.vars.lb     = [ 1.0,   1.0,   2.5,   8000,    0.30   ];
cfg.vars.ub     = [ 5.0,   5.0,   9.0,  16000,    0.60   ];
cfg.vars.x0     = [ 2.65,  2.65,  5.50,  7327,    0.40   ];
cfg.vars.units  = {'m',   'm',   'm',   'N',     'm'    };
cfg.vars.active = [true,  true,  true,  true,    false  ];  % d_prop frozen
```

**To activate propeller diameter as a design variable:**
```matlab
cfg.vars.active(5) = true;   % d_prop optimized; cT = 0.03 × d_prop / 0.40
```

**To add a new variable** (e.g. `cT`):
```matlab
cfg.vars.names{end+1}  = 'cT';
cfg.vars.lb(end+1)     = 0.005;
cfg.vars.ub(end+1)     = 0.10;
cfg.vars.x0(end+1)     = 0.03;
cfg.vars.units{end+1}  = '-';
cfg.vars.active(end+1) = true;
```

**To freeze a variable** (e.g. fix Lx at baseline):
```matlab
cfg.vars.active(1) = false;   % Lx frozen at x0=2.65
```

### 6.3 Optimizer Settings

```matlab
cfg.opt.method     = 'cmaes';  % 'cmaes' | 'ga' | 'fmincon' | 'patternsearch'
cfg.opt.max_evals  = 1500;     % CMA-ES total evaluation budget
cfg.opt.sigma0     = 0.25;     % CMA-ES initial step size in [0,1] space
cfg.opt.pop_size   = 0;        % CMA-ES lambda; 0 = auto (4 + floor(3·log(n)))
cfg.opt.max_iter   = 80;       % GA max generations
cfg.opt.ga_pop     = 50;       % GA population size
cfg.opt.tol_fun    = 1e-6;     % convergence tolerance
cfg.opt.use_polish = true;     % fmincon SQP polish after global search
cfg.opt.verbose    = true;
cfg.opt.plot_live  = true;
```

### 6.4 Vehicle Model

```matlab
cfg.model.use_vehicle_model = true;   % true: physics-based mass (default)
                                      % false: legacy fixed d.m
```

When `true` (and `d.m_payload` is set in the design struct), `hexacopter_params.m` calls `vehicle_model.m` to compute mass, inertia, and cost from design variables. This is the DATCOM-style novel contribution. Set to `false` only for legacy testing.

### 6.5 Two-Stage Evaluation

```matlab
cfg.stage.run_stage2_in_opt = false;  % include mission simulation in optimizer?
                                      % false (default): Stage 1 only (~15 ms/eval)
                                      % true: Stage 1 + Stage 2 (~30× slower)
```

| Stage | Function | Cost | Purpose |
|-------|----------|------|---------|
| Stage 1 | `eval_stage1` | ~15 ms | ACS feasibility, hover screening — use in optimization |
| Stage 2 | `eval_stage2` | ~300–500 ms | Mission tracking, recovery time — use for validation |

### 6.6 Evaluation Mode

```matlab
cfg.eval.mode = 'acs';   % 'acs' | 'sim' | 'full'
```

| Mode | Calls | Cost | Use for |
|------|-------|------|---------|
| `'acs'` | Stage 1 only | ~15–30 ms/call | optimization (default) |
| `'sim'` | Stage 2 only | ~300–1000 ms/call | mission performance |
| `'full'` | Stage 1 + Stage 2 | sum of above | final validation only |

### 6.7 Fault & Simulation Configuration

```matlab
cfg.fault.include_double = false;       % include 2-motor fault scenarios
cfg.fault.p_motor        = 0.05;        % per-motor failure probability

cfg.sim.loe_vec    = [1;0;0;0;0;0];    % motor 1 fails fully
cfg.sim.T_end      = 30;               % simulation duration [s]
cfg.sim.fault_time = 5;                % fault injection time [s]
cfg.sim.dt         = 0.01;             % timestep [s]
cfg.sim.alt_cmd    = 10;               % commanded altitude [m]
```

---

## 7. Module Reference

### 6.1 `design_default()` → `d`
Returns the baseline design struct. Sets `d.m_payload = 1500` to activate the physics-based vehicle model path in `hexacopter_params.m`. The `d.m` field is retained for backward compatibility but is overridden by `vehicle_model.m` at evaluation time.

### 6.2 `vehicle_model(d)` → `model`
**DATCOM-style parameterized vehicle model.** Maps design variables to all physical properties via Delbecq 2020 scaling laws. Called automatically by `hexacopter_params` when `d.m_payload` is present.

Key outputs:

| Field | Description |
|-------|-------------|
| `model.m` | Total mass = m_payload + 6·m_motor + m_frame [kg] |
| `model.m_motor` | Per-motor mass = 74.07 × (T_max/7327)^(3/3.5) [kg] |
| `model.m_frame` | Frame mass = 13.72 × arm_span [kg] |
| `model.Ixx/Iyy/Izz` | Inertia from point-mass motor model [kg·m²] |
| `model.B` | 4×6 control effectiveness matrix |
| `model.cost_total` | = 6·m_motor + m_frame (for J_cost) [kg] |
| `model.cost_ref` | Reference cost at baseline = 740.73 kg |

### 6.3 `hexacopter_params(d)` → `[UAM, Prop, Env]`
Dispatches to `vehicle_model(d)` when `d.m_payload` is present (physics-based path), or uses the legacy calibrated fixed-mass path otherwise. Attaches the full model struct as `UAM.model` for downstream cost computation.

### 6.4 `build_B_matrix(Lx, Lyi, Lyo, cT)` → `B`
Returns the 4×6 control effectiveness matrix for the PPNNPN spin configuration.

### 6.5 `eom_hex(X, T_vec, UAM, Env)` → `X_dot`
6-DOF rigid body equations of motion. State vector X is 15×1:
`[u,v,w, p,q,r, phi,theta,psi, x,y,z, vx,vy,vz]`

### 6.6 `compute_acs_volume(B, T_max, loe_vec)` → `vol`
Computes 4D convex hull volume of the ACS.
Handles degenerate cases (too many faults → low-rank ACS).

### 6.7 `hover_feasibility(B, T_max, m, g, loe_vec)` → `[feasible, utilization]`
LP feasibility check: can the vehicle hover after fault?
`utilization` = max(T_i / T_max_i) at the optimal hover solution.

### 6.8 `compute_hover_threshold(B, m, g, loe_list)` → `[T_thresh, T_ratios]`
Minimum T_max required for hover per fault scenario.
Uses a **single LP** per scenario (max hover fraction λ formulation).
**40× faster than the binary search alternative.**

### 6.9 `fault_isotropy_index(B, T_max, vol_nominal)` → `[FII, retention_vec]`
Computes FII = std(r_i)/mean(r_i) for all 6 single-motor faults.

### 6.10 `eval_acs(d, fault_config)` → `metrics`
Evaluates all ACS-based fault tolerance metrics.

Key outputs:

| Field | Description |
|-------|-------------|
| `vol_nominal` | Nominal ACS volume [N·Nm³] |
| `PFWAR` | Probabilistic Fault-Weighted ACS Retention ∈ [0,1] |
| `FII` | Fault Isotropy Index (lower = more balanced) |
| `WCFR` | Worst-Case single-fault Retention ∈ [0,1] |
| `hover_ok_single` | [6×1] logical: can hover after motor k fails? |
| `hover_margin` | T_max / T_hover_worst − 1 (≥0 means all faults ok) |
| `T_hover_worst` | Min T_max to guarantee hover after any single fault |
| `T_hover_ratios` | T_thresh_k / (m·g/6) per motor |

**fault_config options:**

| Field | Default | Description |
|-------|---------|-------------|
| `include_double` | `false` | Include 2-motor fault scenarios |
| `p_motor` | `0.05` | Per-motor failure probability for PFWAR |

### 6.11 `eval_simulation(d, sim_config)` → `metrics`
Runs closed-loop simulation with fault injection.

Key outputs:

| Field | Description |
|-------|-------------|
| `alt_rmse` | Altitude RMSE post-fault [m] |
| `att_rmse_phi/theta` | Roll/pitch RMSE post-fault [deg] |
| `max_att_excurs` | Max attitude excursion post-fault [deg] |
| `recovery_time` | Time to re-enter ±5° attitude window [s] |
| `ctrl_effort` | Normalized RMS thrust effort |
| `diverged` | `true` if vehicle crashed during simulation |

**sim_config options:**

| Field | Default | Description |
|-------|---------|-------------|
| `loe_vec` | `[1;0;0;0;0;0]` | Motor 1 fault |
| `t_fault` | 10 | Fault injection time [s] |
| `t_end` | 40 | Simulation end time [s] |
| `dt` | 0.01 | Time step [s] (use 0.005 for accuracy) |
| `alt_cmd` | 10 | Altitude command [m] |

### 6.12 `eval_stage1(d, cfg_or_fault)` → `result`
**Stage 1 evaluator** — fast ACS feasibility and hover screening (~15 ms). Used inside the optimization loop.

Accepts either an `mdo_config` struct or a `fault_config` struct.

| Output field | Description |
|-------------|-------------|
| `feasible` | true if WCFR ≥ 5% and geometry is non-degenerate |
| `J_FII` | Fault Isotropy Index |
| `J_hover` | Hover margin penalty ∈ [0,1] |
| `WCFR` | Worst-case fault retention |
| `PFWAR` | Probabilistic fault-weighted retention |
| `hover_margin` | T_max / T_hover_worst − 1 |
| `acs` | Full ACS metrics struct (from eval_acs) |

### 6.13 `eval_stage2(d, cfg_or_sim)` → `result`
**Stage 2 evaluator** — closed-loop mission simulation (~300–500 ms). Called only for designs that passed Stage 1, used for validation.

Accepts either an `mdo_config` struct or a raw `sim_config` struct. Handles both `fault_time` and `t_fault` field naming conventions automatically.

| Output field | Description |
|-------------|-------------|
| `feasible` | true if simulation did not diverge |
| `J_mission` | Normalized altitude RMSE = alt_rmse / alt_cmd |
| `alt_rmse` | Altitude tracking error post-fault [m] |
| `max_att_excurs` | Max roll+pitch excursion [deg] |
| `recovery_time` | Time to re-enter ±5° band after fault [s] (Inf if never) |
| `diverged` | true if attitude exceeded 45° |
| `sim` | Full simulation output struct (t_vec, X_hist, T_hist) |

### 6.14 `eval_design(d, options)` → `result`
Master evaluator. Calls `eval_acs` (Stage 1) and/or `eval_simulation` (Stage 2) depending on `options.mode`. Uses `UAM.model` from `hexacopter_params` for physics-based J_cost when available.

**options.mode:**
- `'acs'`  — Stage 1 only (~15–30 ms/call). Use for optimization.
- `'sim'`  — Stage 2 simulation only (~300–1000 ms/call).
- `'full'` — Stage 1 + Stage 2 (for final validation).

**options.weights:** `[w_FII, w_hover, w_mission, w_cost]` or a struct with fields `.FII/.hover/.mission/.cost`
Default: `[0.35, 0.40, 0.10, 0.15]`

**Scalar objectives returned:**

| Field | Description |
|-------|-------------|
| `J_FII` | = FII (geometry-sensitive) |
| `J_hover` | = 0 if T_max covers worst fault; exponentially penalized otherwise |
| `J_mission` | Normalized simulation altitude RMSE (NaN in 'acs' mode) |
| `J_cost` | = (6·m_motor + m_frame) / 740.73 when vehicle model active; legacy formula otherwise |
| `J_combined` | Weighted combination (NaN terms dropped with weight renormalization) |

### 6.15 `run_soo(d_init, cfg_or_options)` → `[d_opt, J_opt, history]`
Single-objective optimization. Accepts either an `mdo_config` struct (recommended) or a legacy options struct (backward compatible).

**Recommended (config-driven):**
```matlab
cfg = mdo_config();
[d_opt, J_opt, hist] = run_soo(design_default(), cfg);
```

**Legacy options struct** (still supported):
```matlab
opts.method    = 'ga';
opts.var_names = {'Lyi','T_max'};
opts.lb        = [1.0, 8000];
opts.ub        = [5.0, 16000];
[d_opt, J_opt, hist] = run_soo(design_default(), opts);
```

When `opts.method = 'cmaes'`, the legacy path automatically builds a cfg struct and delegates to `run_cmaes`.

**`history` struct fields:**

| Field | Description |
|-------|-------------|
| `J_all` | J at every function evaluation |
| `J_best` | Best J found up to each improvement |
| `x_best` | Design variable vector at best J |
| `n_evals` | Total function evaluations used |

### 6.16 `run_cmaes(d_init, cfg)` → `[d_opt, J_opt, history]`
Self-contained CMA-ES optimizer. Called automatically by `run_soo` when `cfg.opt.method = 'cmaes'`.

Implements the **(μ/μ_w, λ)-CMA-ES** (Hansen 2016) entirely in normalized [0,1] variable space, making it scale-invariant across variables with different physical units (arm lengths in m vs T_max in N).

Key design choices:
- **Normalized space**: all variables mapped to [0,1] before sampling
- **Reflection bounds**: out-of-bounds candidates reflected back into [0,1] (preserves search distribution better than clamping)
- **Amortized eigendecomposition**: `eig(C)` computed every `floor(λ/(c1+cmu)/n/10)` generations
- **Stagnation detection**: stops if no improvement for 30% of the evaluation budget
- **Optional SQP polish**: `cfg.opt.use_polish = true` runs `fmincon` after CMA-ES

### 6.17 `run_moo(d_init, options)` → `[pareto_designs, pareto_J, ...]`
Multi-objective Pareto optimization with live updating Pareto scatter.
Uses `gamultiobj` (MATLAB Global Optimization Toolbox required).

### 6.18 `sweep_design_space(d_base, var, range, options)` → `results`
1D or 2D parameter sweep. Example:
```matlab
% 1D
sw = sweep_design_space(d_base, 'Lyi', linspace(1,5.5,30));

% 2D
sw = sweep_design_space(d_base, {'Lyi','T_max'}, ...
     {linspace(1,5.5,20), linspace(7000,16000,20)});
```

### 6.19 `pareto_analysis(designs, F, options)` → `[F_nd, d_nd, knee_idx]`
Filters dominated solutions and identifies the knee point.
Knee = maximum perpendicular distance from ideal–antiideal line.

### 6.20 `compare_designs(designs, labels, options)`
Prints a side-by-side comparison table and radar chart.

### 6.21 `visualize_results(data, type, options)`
Unified visualization:

| `type` | Input | Description |
|--------|-------|-------------|
| `'acs'` | `eval_design` result | ACS retention bar, 3D projection, hover polar |
| `'sim'` | `eval_design` result | Altitude, attitude, thrust time histories |
| `'sweep'` | `sweep_design_space` result | 1D curves or 2D surface plots |
| `'pareto'` | `[N×3]` objective matrix | 3D scatter + 2D projections |

---

## 8. Optimization Workflow

### 8.1 Why two optimization paths?

This framework provides two distinct optimization paths that answer different questions:

| Path | Function | What it answers |
|------|----------|-----------------|
| **SOO** (Section 3 of `main_mdo`) | `run_soo` / `run_cmaes` | "What is the single best design given my current priorities?" |
| **MOO / Pareto** (Section 4 of `main_mdo`) | `run_moo` + `pareto_analysis` | "What are all the designs where I cannot improve one objective without hurting another?" |

Neither replaces the other. SOO gives one concrete design fast (~20 s). MOO gives the complete trade-off picture (~10–20 min).

---

### 8.2 SOO-Optimal design

**What it is:** The single design that minimizes the combined scalar objective `J_combined = Σ wᵢ·Jᵢ / Σ wᵢ` subject to physical constraints.

**When to use it:** When you have already decided on priorities — you know you care more about hover safety than cost, or more about FII than T_max — and you want a single answer to bring to a meeting or report.

**What to expect:**
- The result is sensitive to the weight choices. Changing `cfg.weights` will give a different SOO-optimal design.
- With default weights `[FII=0.35, hover=0.40, cost=0.25]`, the SOO optimizer drives toward the minimum arm span that just satisfies the hover constraint. All three geometry variables hit their lower bounds because reducing arm span reduces J_cost without meaningfully hurting J_FII at these weight values.
- The SOO-optimal design is therefore a *hover-safe, minimum-cost* design, not a *fault-isotropic* design.
- If fault balance matters more, increase `cfg.weights.FII` (e.g. to 0.60) before running SOO.

**Limitation:** SOO conflates multiple objectives into one number. The "best" design depends entirely on the chosen weights, which encode engineering judgement. If the weights are wrong, the result is wrong.

---

### 8.3 Pareto Front and the Knee Design

**What the Pareto front is:** The set of designs where no objective can be improved without making at least one other objective worse. Every design on the Pareto front is optimal for *some* weighting of objectives.

This framework computes a 3-objective Pareto front:

| Objective | Symbol | Meaning |
|-----------|--------|---------|
| f₁ | J_FII | Fault balance — wants large, symmetric arm span |
| f₂ | J_hover | Hover safety — wants high T_max |
| f₃ | J_cost | Motor + structural cost — wants small arm span and low T_max |

These objectives conflict:
- f₁ vs f₃: FII improves with larger Lyi (longer lateral arms), but cost increases with arm span
- f₂ vs f₃: Hover safety requires high T_max, but heavier motors raise cost
- f₁ and f₂ are partially aligned (both want larger arms and more thrust), creating a clear cost–safety frontier

**What the Pareto knee is:** The point on the Pareto front where the *rate of return* switches — going further toward one objective yields diminishing improvement at rapidly increasing cost to the others. Mathematically, `pareto_analysis` finds the knee as the point with maximum perpendicular distance from the line connecting the ideal (best each objective separately) and anti-ideal (worst each objective on the front) points.

```
                f₃ (cost)
                ▲
                │   Pareto front
                │  ╲
                │   ╲     ← knee point here
                │    ╲___________
                └──────────────────→ f₁ (FII)

  Below the knee: small cost improvement requires large FII sacrifice
  Above the knee: small FII improvement requires large cost increase
```

**Why use the knee design?** It is the design that offers the best overall balance without pre-committing to a specific weight vector. It is the recommended design for reports and comparisons when you do not have a strong engineering reason to favor one objective over others.

**What to expect from the knee design:**
- T_max will be higher than the SOO-optimal (hover safety is a hard-to-avoid cost in the MOO)
- Arm span will be moderate — not minimum (as SOO drives) and not maximum
- FII will be lower than baseline (more balanced fault impacts)
- J_cost will be higher than SOO-optimal but lower than a pure-FII-optimal design

**Interpretation summary:**

| Design | Arm span | T_max | FII | Cost | Best for |
|--------|----------|-------|-----|------|----------|
| Baseline | reference | low | 0.311 | 1.00 | — starting point |
| SOO-Optimal | minimum | just-sufficient | ~0.32 | ~0.77 | Reports requiring one number; weight-dependent |
| Pareto Knee | moderate | moderate-high | ~0.20 | ~0.85 | Balanced recommendation; weight-independent |

---

### 8.4 Recommended sequence

```
1. Edit config/mdo_config.m   → set optimizer, weights, variable bounds
2. main_mdo Section 1         → evaluate baseline, understand weaknesses
3. main_mdo Section 2         → 1D/2D sweeps to identify design levers
4. main_mdo Section 3         → SOO with CMA-ES (one line: run_soo(d_base, cfg))
                                 → gives best design under your chosen weights
5. main_mdo Section 4         → MOO Pareto front (gamultiobj)
                                 → reveals trade-off structure, weight-independent
6. pareto_analysis            → identify knee design
7. eval_design mode='full'    → validate both SOO-optimal and knee with simulation
8. compare_designs            → side-by-side report: Baseline / SOO / Knee
```

Run steps 3 and 4 with `cfg.eval.mode = 'acs'` (default). Only use `mode='full'` in step 7.

### 8.5 Objective weights guidance

| Priority | `weights` | Expected SOO outcome |
|----------|-----------|----------------------|
| Safety first | `[0.20, 0.60, 0.00, 0.20]` | Highest T_max; geometry at lower bounds |
| Balanced (default) | `[0.35, 0.40, 0.00, 0.25]` | Minimum arm + hover-just-satisfied |
| Fault isotropy focus | `[0.60, 0.20, 0.00, 0.20]` | Larger Lyi; more even fault impacts |
| Cost minimization | `[0.25, 0.35, 0.00, 0.40]` | Smallest feasible arm + lowest T_max |
| With mission | `[0.25, 0.30, 0.25, 0.20]` | Needs `eval.mode='sim'`; much slower |

### 8.6 Common customizations (all in `mdo_config.m`)

**Run GA instead of CMA-ES:**
```matlab
cfg.opt.method  = 'ga';
cfg.opt.max_iter = 80;
cfg.opt.ga_pop   = 50;
```

**Optimize only T_max (freeze geometry):**
```matlab
cfg.vars.active = [false, false, false, true];
```

**Activate propeller diameter as a 5th design variable:**
```matlab
cfg.vars.active(5) = true;   % d_prop ∈ [0.30, 0.60] m; cT scales linearly
```

**Add a completely new variable** (e.g. cT as an independent variable):
```matlab
cfg.vars.names{end+1}  = 'cT';
cfg.vars.lb(end+1)     = 0.005;  cfg.vars.ub(end+1)  = 0.10;
cfg.vars.x0(end+1)     = 0.03;   cfg.vars.units{end+1} = '-';
cfg.vars.active(end+1) = true;
```

---

## 9. Key Metrics Explained

### Fault Isotropy Index (FII) — *novel*

```
FII = std(r₁, r₂, ..., r₆) / mean(r₁, ..., r₆)
```

where rᵢ = ACS volume after motor i fails / nominal ACS volume.

- **FII = 0**: All motor failures degrade the ACS by exactly the same amount. Ideal balanced design.
- **FII > 0**: Some motors are more critical than others. High FII means the design has a "weakest link."
- Baseline: FII = 0.311.

### Probabilistic Fault-Weighted ACS Retention (PFWAR)

```
PFWAR = Σₖ P(fault k) × V_fault_k / V_nominal
```

Weights each fault scenario by its probability using independent motor failure model (p = 0.05 per motor by default).

**Note:** For single-motor faults, mean(rᵢ) = 1/3 exactly (zonotope identity). PFWAR ≈ 0.30 is geometry-invariant for this configuration. Use FII and hover_margin as the design levers.

### Hover Margin

```
hover_margin = T_max / T_hover_worst − 1
```

- **≥ 0**: T_max is sufficient — all 6 single-motor faults allow hover.
- **< 0**: T_max is insufficient — magnitude shows how far below threshold.
- Baseline: hover_margin = −0.333 (T_max is 33% below the 3× threshold).
- Threshold: T_hover_worst = 3.0 × m·g/6 = **10,991 N** for this vehicle.

### Worst-Case Fault Retention (WCFR)

Minimum ACS volume retention across all single-motor faults.
Baseline: WCFR = 0.20 (motors 3 and 5 reduce ACS to 20% of nominal).

---

## 10. Performance Notes

### Evaluation speed (per `eval_design` call, `mode='acs'`):

| Operation | Calls per eval | Cost |
|-----------|---------------|------|
| `compute_acs_volume` | 7 (nominal + 6 single faults) | ~10 ms total |
| `compute_hover_threshold` | 6 (one LP each) | ~5 ms total |
| `eval_acs` total | — | **~15–30 ms** |

### Optimization runtimes (measured):

| Optimizer | Settings | Time | Evals |
|-----------|----------|------|-------|
| **CMA-ES** | budget=1500, pop=auto(8) | **~20 s** | ~1150 (converges early) |
| GA (SOO) | 50 pop × 80 gen | ~3–6 min | ~4000 |
| fmincon (polish) | 100 iter | < 1 min | ~200 |
| gamultiobj (MOO) | 100 pop × 150 gen | ~10–20 min | ~15000 |

CMA-ES is the recommended optimizer: it adapts its search distribution to the problem geometry and typically converges in 1/4 the evaluations of GA.

### Speed tips
- Use `mode='acs'` (not `'full'`) inside optimization loops.
- Set `fault_config.include_double = false` (default) during optimization; use `true` only for final analysis.
- For fast sweeps, use `sw_opts.verbose = false`.
- If using parallel toolbox, set `'UseParallel', true` in GA options (not used by CMA-ES).

---

## 11. Design Findings Summary

| Finding | Details |
|---------|---------|
| **Zonotope identity** | Mean single-fault ACS retention = 1/3 exactly, regardless of arm geometry |
| **PFWAR invariance** | PFWAR ≈ 0.30 is geometry-invariant; not a useful design lever for arm shape |
| **Hover threshold** | T_max ≥ 3.0× (m·g/6) required for all-fault hover feasibility |
| **Mass-propulsion coupling** | With physics-based model: raising T_max increases motor mass → raises hover threshold. Equilibrium T_max ≈ 11,400 N (slightly above the fixed-mass estimate of 10,991 N) |
| **T_max sweet spot** | Just above the self-consistent hover threshold: full hover + minimum motor cost |
| **Cost drives geometry** | Cost objective favors minimum arm span; hover constraint drives T_max |
| **cT irrelevance** | Moment-to-thrust ratio has no effect on ACS retention or FII |
| **Pareto trade-off** | FII (wants large Lyi) vs cost (wants small Lyi + small T_max) |

### Three-way design comparison

The framework produces three named designs for comparison (see `compare_designs` in Section 5 of `main_mdo.m`):

**Baseline** — The reference vehicle from the original simulation scripts. No optimization applied. Exists to quantify what the optimizer is improving over.

**SOO-Optimal** — The design minimizing `J_combined` with default weights `[FII=0.35, hover=0.40, cost=0.25]`. This is a *weight-dependent* answer to "what is the best single design?" Under default weights, the optimizer drives to minimum arm span (lowest cost) while just satisfying the hover constraint. The geometry bounds are all active at their lower limits because the FII weight is insufficient to push arms outward against the cost penalty.

> If you increase `cfg.weights.FII` (e.g. to 0.60) the SOO-optimal design shifts toward larger Lyi and more balanced fault tolerance. The SOO-optimal design is only as good as the weight choice.

**Pareto Knee** — Extracted from the MOO Pareto front by `pareto_analysis`. This design offers the best balance across all three objectives without pre-committing to a weight vector. It typically has a larger arm span than the SOO-optimal (improving FII) and a moderate T_max (hover-feasible but not over-sized). It is the *weight-independent recommendation*.

> The knee design is useful when you cannot justify a specific weight vector — e.g. early in design when priorities are not yet fixed, or when presenting to stakeholders who want to see the trade-off rather than a single answer.

### Measured results (fixed-mass model, default weights)

| Metric | Baseline | SOO-Optimal | Pareto Knee | Notes |
|--------|----------|-------------|-------------|-------|
| J_combined | 0.648 | **0.304** | ~0.38 | SOO minimizes this directly |
| FII | 0.311 | ~0.32 | **~0.18** | Knee better: larger, balanced arms |
| hover_margin | −0.333 | **0.000** | **≥ 0** | Both optimal designs satisfy hover |
| hover_ok | 4/6 | **6/6** | **6/6** | — |
| J_cost | 1.000 | **0.766** | ~0.90 | SOO better: minimum arm span |
| T_max [N] | 7,327 | ~11,000 | ~11,500 | Knee slightly higher (mass coupling) |
| Lx [m] | 2.65 | 1.00 (lb) | ~2.0–3.0 | Knee not at lower bound |
| Lyi [m] | 2.65 | 1.00 (lb) | **~3.5–4.5** | Knee at FII-optimal geometry |
| Lyo [m] | 5.50 | 2.50 (lb) | ~4.0–5.5 | — |
| CMA-ES evals | — | ~1,150 | — | SOO wall time ~20 s |
| MOO evals | — | — | ~15,000 | Pareto wall time ~10–20 min |

> **Pareto Knee values are approximate** — exact values depend on MOO run (stochastic). Re-run `pareto_analysis` to get current knee from your last MOO result.

**Key take-away:** SOO-Optimal wins on cost. Pareto Knee wins on fault balance (FII). Both satisfy hover. Use SOO for a quick answer under a fixed priority set; use the Pareto Knee when presenting a design recommendation that does not depend on a particular weight choice.

---

*Framework developed for AE50001 MDO Course / Team 5 / 2026*
