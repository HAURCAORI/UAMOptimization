# Optimization Specification and Implementation Guide

## 1. Problem Statement

Find the hexacopter design vector **d** that minimizes a weighted sum of
Stage 1 objectives while satisfying geometric and hover-feasibility
constraints. Stage 1 is simulation-free (~15 ms/eval), enabling large
search budgets. Stage 2 (closed-loop simulation) is used only for
post-optimization validation.

**Design variables** (active set defined in `mdo_config.m`):

| Variable | Symbol | Default lb | Default ub | Unit | Default active |
|----------|--------|-----------|-----------|------|---------------|
| `Lx`     | Fore/aft arm length           | 1.0 | 5.0  | m | yes |
| `Lyi`    | Inner lateral arm (front/rear)| 1.0 | 5.0  | m | yes |
| `Lyo`    | Outer lateral arm (mid rotors)| 2.5 | 9.0  | m | yes |
| `T_max`  | Max thrust per motor          | 8000 | 16000 | N | yes |
| `d_prop` | Propeller diameter            | 0.30 | 0.60 | m | no (frozen) |

**Hard constraints** (enforced as infinite penalty):

- `Lyo > Lyi + 0.1` — geometric separation between rotor rings
- `WCFR >= 0.05` — worst-case fault control retention
- Single-motor hover feasibility for all six motors
- Nominal hover feasibility
- Hover origin containment at `Fz = -W, N = 0`: the point `(L,M)=(0,0)` must be feasible
  under nominal and single-fault conditions; optional positive reserve is set by
  `cfg.fault.hover_origin_margin_req`

---

## 2. Stage 1 Objective Function

The combined scalar objective is:

```
J = (w1·f1 + w2·f2 + w3·f3 + w4·f4 + w5·f5) / (w1+w2+w3+w4+w5)
```

### Objective Terms

#### f1 — Mass

```
f1 = m_tot / m_ref
```

Normalizes total vehicle mass against the reference (baseline) design.
Penalizes heavier designs. `m_ref` comes from the vehicle model or
`design_default()`.

#### f2 — Hover Power

```
f2 = P_hover / P_ref

P_hover = sum_i( T_i^1.5 ) / sqrt(A_disk)
```

Ideal induced-power proxy from momentum theory. Minimizing it reduces
hover energy consumption. Normalized against the reference design's
power at the same flight condition.

#### f3 — Fault Thrust Margin (penalty)

```
f3 = max(0, gamma_req - gamma_worst)^2

gamma_worst = min_k [ sum(T_ub,k) / (m·g) ]
T_ub,k      = T_max · (1 - loe_k)           for single-motor fault k
```

`gamma_worst` is the minimum post-fault thrust-to-weight ratio over all
six single-motor failures. `f3 = 0` when the worst fault case has
sufficient thrust margin (`gamma_worst >= gamma_req`). Positive
otherwise, giving a smooth penalty gradient near the threshold.

**Parameter**: `cfg.fault.gamma_T_req` (default 1.5). Increasing this
value demands a larger thrust reserve and provides optimization pressure
even when all fault cases are hover-feasible.

#### f4 — Fault Control Effectiveness

```
f4 = sigma_ref / sigma_worst

sigma_worst = min_k [ sigma_min( S · B · diag(T_ub,k) ) ]
```

`sigma_min` is the minimum singular value of the dimensionlessly scaled
control-effectiveness matrix under fault. `S` is a diagonal normalization
matrix with rows `[1/(mg), 1/(mg·L), 1/(mg·L), 1/yaw_max]`.

`sigma_ref` is the same metric evaluated on the reference design and used
as a normalization constant. Lower `sigma_worst` → higher `f4` → worse
controllability under fault.

#### f5 — Nominal Hover Load Fraction

```
f5 = (T_bar_hover / T_max)^2

T_bar_hover = mean_i(T_hover,i)   (nominal hover, no fault)
```

The mean motor thrust fraction in nominal hover, squared. A lower value
means more thrust headroom: either a lighter vehicle or a more powerful
motor. For a symmetric hexacopter, `T_bar_hover = m·g / 6`.

### Default Weights

```matlab
cfg.objectives.stage1.names   = {'mass','power','fault_thrust','fault_alloc','hover_nom'};
cfg.objectives.stage1.weights = [0.20,   0.20,   0.25,          0.25,         0.10];
```

Fault-related terms (`fault_thrust` + `fault_alloc`) carry 50% of the
total weight, reflecting the fault-tolerance focus of this project.

---

## 3. Multi-Objective Pareto Front

For `run_moo`, the framework uses a three-objective Pareto front:

```
F = [ J_mass,  J_power,  J_fault ]

J_fault = 0.5 · (J_fault_thrust + J_fault_alloc)
```

Configured via `cfg.objectives.stage1.moo_names = {'mass','power','fault'}`.

---

## 4. Implementation Map

| Equation term  | Where computed                  | Result field                         |
|----------------|---------------------------------|--------------------------------------|
| `f1` (mass)    | `eval_stage1.m` §J_mass         | `result.objectives.mass`             |
| `f2` (power)   | `eval_stage1.m` §J_power        | `result.objectives.power`            |
| `f3` (thrust)  | `eval_stage1.m` §J_fault_thrust | `result.objectives.fault_thrust`     |
| `f4` (alloc)   | `eval_stage1.m` §J_fault_alloc  | `result.objectives.fault_alloc`      |
| `f5` (hover)   | `eval_stage1.m` §J_hover_nom    | `result.objectives.hover_nom`        |
| `J_combined`   | `eval_design.m` `combine_objectives` | `result.J_combined`             |
| Weights, names | `mdo_config.m`                  | `cfg.objectives.stage1.*`            |
| `gamma_T_req`  | `mdo_config.m`                  | `cfg.fault.gamma_T_req`              |
| Design var bounds | `mdo_config.m`               | `cfg.vars.*`                         |

---

## 5. Extending the Framework

### 5.1 Adding a New Objective

**Step 1** — Register in `mdo_config.m`:
```matlab
cfg.objectives.stage1.names   = {'mass','power','fault_thrust','fault_alloc','hover_nom','my_obj'};
cfg.objectives.stage1.weights = [0.20,   0.20,   0.20,          0.20,         0.10,       0.10];
```

**Step 2** — Compute in `eval_stage1.m`, inside the main function body:
```matlab
%% J_my_obj
J_my_obj = ...;   % your formula using UAM, Prop, Env, acs, etc.
```

**Step 3** — Add to `result.objectives`:
```matlab
result.objectives = struct( ...
    ...
    'my_obj', J_my_obj, ...   % <-- append here
    ...);
```

**That is all.** `eval_design.m` calls `combine_objectives` which looks up
`objective_values.(name)` dynamically, so no further changes are needed.

### 5.2 Adding a New Design Variable

**Step 1** — Add a row in `mdo_config.m`:
```matlab
cfg.vars.names  = {'Lx', 'Lyi', 'Lyo', 'T_max', 'd_prop', 'new_var'};
cfg.vars.lb     = [ 1.0,  1.0,   2.5,   8000,    0.30,     <lb>     ];
cfg.vars.ub     = [ 5.0,  5.0,   9.0,  16000,    0.60,     <ub>     ];
cfg.vars.x0     = [ 2.65, 2.65,  5.50,  8000,    0.40,     <x0>     ];
cfg.vars.units  = {'m',  'm',   'm',   'N',     'm',      '<unit>' };
cfg.vars.active = [true, true,  true,  true,    false,    true     ];
```

**Step 2** — Use `d.new_var` in `hexacopter_params.m` (or your physics
model) to propagate the variable to the physical properties that affect
your objectives.

**Step 3** — Nothing else. `run_soo` and `run_cmaes` iterate over
`cfg.vars.active` and automatically include the new variable.

To temporarily freeze a variable without removing it, set `active = false`.

### 5.3 Changing the fault_thrust Formula

The `gamma_T_req` threshold is in `mdo_config.m`:
```matlab
cfg.fault.gamma_T_req = 1.5;   % default
```

- **1.0** — pure feasibility penalty (zero for any hover-feasible design)
- **1.5** — penalizes designs with thin but feasible fault margins
- **2.0** — requires a 2× T/W reserve after fault; closer to the baseline margin

### 5.4 Switching the Optimizer

Change `cfg.opt.method` in `mdo_config.m`:
```matlab
cfg.opt.method = 'cmaes';         % default; fast, ~500–1500 evals
cfg.opt.method = 'ga';            % global, slower; ~ga_pop × max_iter evals
cfg.opt.method = 'fmincon';       % local gradient-free; fast but may miss global
cfg.opt.method = 'patternsearch'; % local, derivative-free
```

---

## 6. CMA-ES Normalization

All design variables are normalized to `[0, 1]` inside `run_cmaes.m`
before passing to CMA-ES:

```
x_norm = (x_phys - lb) / (ub - lb)
```

This makes the optimizer scale-invariant (arm lengths in meters and
T_max in kN are treated equally). The initial step size `cfg.opt.sigma0 =
0.25` means the initial search radius is 25% of each variable's range.
After CMA-ES, an optional `fmincon` polish refines the solution in physical
space (`cfg.opt.use_polish = true`).

---

## 7. Feasibility and Penalty

Infeasible designs (constraint violations) are assigned `J = 1e6` and
excluded from the Pareto front. The CMA-ES also uses a soft geometric
penalty before calling `eval_design`:

```matlab
if Lyo - Lyi < 0.1
    J = 1e6 + 1e4 * (0.1 - gap)   % smooth approach penalty
end
```

This guides the optimizer toward the feasible region faster than a hard
wall at `J = 1e6`.
