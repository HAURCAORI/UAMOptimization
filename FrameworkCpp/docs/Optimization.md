# FrameworkCpp Optimization Overview

This is the top-level optimization reference for `FrameworkCpp`.

It explains the flow of data and where each optimization concern lives in the codebase. For the current variable and hard-constraint list, use `docs/Optimization_Constraints_And_Design_Variables.md`.

## Document map

- `Optimization.md`: overall evaluation and optimizer flow
- `Optimization_Constraints_And_Design_Variables.md`: current active variables, inactive registered parameters, and hard constraints
- `Phase2_Powertrain_Battery.md`: powertrain and battery model details
- `Action.md`: implementation checklist for extending the optimization stack

## Architecture-to-objective flow

The optimization loop is:

```text
active parameters
  -> DesignVectorMapper / BoundsBuilder
  -> candidate HexacopterArchitecture
  -> VehicleScalingModel
  -> StructuralAnalyzer
  -> AttainableControlSetAnalyzer
  -> PowertrainEvaluator
  -> BatteryEvaluator
  -> hard constraint evaluation
  -> ObjectiveAggregator
  -> pagmo fitness
```

Main implementation points:

- parameter registration: [HexacopterArchitecture.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/core/HexacopterArchitecture.cpp:1)
- optimizer adapter: [PagmoProblemAdapter.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/optimization/PagmoProblemAdapter.cpp:1)
- Stage 1 evaluation: [Stage1Evaluator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/evaluation/Stage1Evaluator.cpp:1)
- objective weighting: [ObjectiveAggregator.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/evaluation/ObjectiveAggregator.cpp:1)

## Design-vector handling

| Name               | Symbol   | Unit | Default | Bounds         | Active |
|--------------------|----------|------|---------|----------------|--------|
| Lx                 | Lx       | m    | 2.65    | [2.0, 5.0]     | yes    |
| Lyi                | Lyi      | m    | 2.65    | [2.0, 5.0]     | yes    |
| Lyo                | Lyo      | m    | 5.50    | [2.5, 9.0]     | yes    |
| T_max              | Tmax     | N    | 12000   | [8000, 20000]  | yes    |
| cT                 | cT       | -    | 0.03    | [0.01, 0.08]   | no     |
| d_prop             | d_prop   | m    | 0.40    | [0.20, 1.20]   | no     |
| m_payload          | m_pay    | kg   | 800     | [400, 2000]    | no     |
| arm_outer_radius   | r_o      | m    | 0.08    | [0.02, 0.15]   | yes    |
| arm_wall_thickness | t_w      | m    | 0.005   | [0.001, 0.020] | yes    |
| m_bat              | m_bat    | kg   | 400     | [100, 1000]    | yes    |

Active parameters (7): Lx, Lyi, Lyo, T_max, arm_outer_radius, arm_wall_thickness, m_bat.
Inactive parameters are fixed at their defaults during optimization.

- packs active parameters into a normalized vector
- unpacks normalized values back into the cloned architecture
- calls `architecture.updateFromParameters()` after assignment

    x_norm = (x - x_lower) / (x_upper - x_lower) * scale    (scale = 1.0 for all)

Pagmo sees bounds [0, 1] for each active parameter. The lower bound maps to
x_norm = 0; the upper bound to x_norm = 1.0 (= scale).

`PagmoProblemAdapter::problem()`:

- exports physical bounds and parameter IDs for reporting and CSV/JSON outputs

## Evaluation stages

### 1. Vehicle model

`VehicleScalingModel` builds the physical model from the current architecture:

- total mass
- COM and inertia
- allocation matrix and faulted allocation matrices
- packaging clearance metrics
- structural aggregate metrics such as arm span and normalized bending index

### 2. Structural analysis

`StructuralAnalyzer` evaluates arm section properties and loads to compute:

- per-arm bending stress
- per-arm safety factor
- minimum arm safety factor across the vehicle

This is why `ArmElement` implements `IStructuralBeam` and `ILoadReceiver`.

### 3. ACS / controllability analysis

`AttainableControlSetAnalyzer` evaluates:

- nominal hover trim
- single-fault hover trims
- ACS retention and margin metrics
- the current hover-feasibility margin used by hard constraints

### 4. Powertrain analysis

`PowertrainEvaluator` computes:

- nominal total electrical hover power
- worst-fault total electrical hover power
- worst nominal thrust utilization
- worst nominal power utilization

This phase uses effective disk area derived from arm geometry, not directly from `d_prop`.

### 5. Battery analysis

`BatteryEvaluator` computes:

- available battery energy
- nominal and emergency mission energy demand
- energy reserve fraction
- peak C-rate
- battery mass fraction

These feed the Phase 2 hard constraints.

### 6. Hard constraint evaluation

The evaluator builds `ConstraintEvaluationContext` and runs all registered constraints from:

- `HexacopterArchitecture::registerDefaultConstraints()`
- each element's `registerConstraints()`

Results are stored in `EvaluationResult.constraint_results`.

### 7. Objective aggregation

`ObjectiveAggregator` selects only objectives with positive weights from `EvaluationContext.objective_weights`, then computes:

```text
combined_objective = sum(weight_i * value_i) / sum(weight_i)
```

Inactive objectives can still be computed and exported without affecting the weighted-sum SOO objective.

## Feasibility

A result is feasible only if:

1. nominal hover trim is feasible
2. all-fault hover is feasible when `require_all_fault_acs_feasible` is enabled
3. every active hard constraint is feasible

Infeasible points are still evaluated, exported, and penalized so pagmo can search with a gradient-like signal through violation magnitudes.

## Pagmo integration

`PagmoProblemAdapter` is the single integration point with pagmo.

Responsibilities:

- clone the architecture
- unpack normalized variables
- run Stage 1 evaluation
- expose bounds
- return either:
  - a single weighted-sum fitness for SOO, or
  - an objective vector for MOO
- add weighted hard-constraint penalties

Penalty behavior:

- each violated active hard constraint contributes `penalty_weight * violation`
- an additional infeasibility constant is applied when the result is not feasible

## SOO and MOO defaults

### SOO

Implementation: [SooRunner.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/optimization/SooRunner.cpp:1)

- algorithm: CMA-ES
- objective name: `combined`
- default population: 24
- default generations: 40
- default seed: 42

Outputs:

- baseline result
- best raw result
- best feasible result if one exists
- parameter and comparison exports

### MOO

Implementation: [MooRunner.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/optimization/MooRunner.cpp:1)

- algorithm: NSGA-II
- default population: 32
- default generations: 60
- default seed: 42
- config default objective set in code: `mass`, `power`, `fault_thrust`, `fault_alloc`, `hover_nom`
- CLI currently uses: `mass`, `power`, `fault_alloc`

Outputs:

- full population
- feasible sub-population
- Pareto CSV / JSON exports
- knee-point analysis through `ParetoAnalyzer`

## Export and reporting

Key export points:

- `CsvExporter::writeSooComparisonCsv`
- `CsvExporter::writeSooParametersCsv`
- `CsvExporter::writeParetoCsv`
- `CsvExporter::writeParetoParametersCsv`
- `CsvExporter::writeSooJson`
- `CsvExporter::writeMooJson`

Key reporting points:

Battery pack. Mass = m_bat (Phase 2 design variable) — contributes directly
to total vehicle mass and creates a feedback loop with the hover thrust
requirement.

**Parameters consumed:** m_bat, Tmax, d_prop

**Mass:** m_bat (active design variable, default 400 kg)

Older optimization notes in this folder had drift in several areas. Current code now includes:

- active structural variables: `arm_outer_radius`, `arm_wall_thickness`
- active battery variable: `m_bat`
- structural hard constraint: `arm_yield_failure`
- battery hard constraints: `battery_energy_reserve`, `battery_crate_limit`
- ACS hard constraints: `all_faults_hover_feasible`, `fault_directional_margin`
- additional objective slots: `structural_safety`, `acs_margin`

**Anchors:** `mount`, `center` (both at origin).

**Constraints registered:**
- `battery_mass_positive`: m_bat >= 0

---

### 3.7 PayloadElement

Rigid payload mass body.

**Parameters consumed:** m_pay

**Mass:** m_pay (implements `IPayloadMassContributor`)

**Geometry:** sphere of radius 0.60 m (fixed).

**Anchors:** `center` at origin.

**Constraints registered:**
- `payload_mass_nonnegative`: m_pay >= 0

---

## 4. Rotor Positions and Assembly

**Motor world-frame positions (z = 0 for all rotors):**

| Index | x    | y    |
|-------|------|------|
| 0     | +Lx  | -Lyi |
| 1     | +Lx  | +Lyi |
| 2     | 0    | -Lyo |
| 3     | 0    | +Lyo |
| 4     | -Lx  | -Lyi |
| 5     | -Lx  | +Lyi |

**Attachment chain:**

    root --> body (center)
      body (bottom) --> battery (mount)  + offset (0, 0, +0.02)
      body (bottom) --> payload (center) + offset (0, 0, -0.05)
      body (center) --> arm_i (root)     + direction toward motor_i
        arm_i (tip) --> motor_i (mount)  + rigid
          motor_i (axis) --> rotor_i (axis) + rigid

---

## 5. Physical Model Equations

### 5.1 Mass Properties

    m_total = sum_k( m_k )

    r_COM = sum_k( m_k * r_k ) / m_total

**Inertia** (parallel-axis theorem, world frame):

    I_total = sum_k[ I_k_local + m_k * (||r_k||^2 * Id - r_k * r_k^T) ]

**Body inertia correction** (Ix_0, Iy_0 account for distributed structure):

    Ix_body = Ix_0 - sum_j[ m_mot_j * (yj^2 + zj^2) ]
    Iy_body = Iy_0 - sum_j[ m_mot_j * (xj^2 + zj^2) ]
    Iz_body = Ix_body + Iy_body

These are added to Ixx, Iyy, Izz respectively.

---

### 5.2 Allocation Matrix

The 4x6 matrix **B** maps rotor thrusts T (R^6) to u = [Fz, Mx, My, Mz]^T:

    u = B * T

    Row 0 (Fz):  -1     -1     -1     -1     -1     -1
    Row 1 (Mx):  -Lyi   +Lyi   -Lyo   +Lyo   -Lyi   +Lyi
    Row 2 (My):  +Lx    +Lx     0      0     -Lx    -Lx
    Row 3 (Mz):  -cT    -cT    +cT    +cT    -cT    +cT

- Row 0: vertical thrust (Fz), negative = upward in NED
- Row 1: pitch moment (Mx); coefficient = rotor y-position
- Row 2: roll moment (My); coefficient = rotor x-position
- Row 3: yaw moment (Mz); coefficient = yaw_sign * cT

For faulted rotor f: column f of B is set to zero.

---

### 5.3 Hover Feasibility Solver

Finds T in [0, T_ub] satisfying B * T = [-m_total * g, 0, 0, 0]^T.

**Algorithm:** enumerate all C(6,2) x 4 = 60 basis configurations.
For each pair (i, j) with boundary values {0, T_ub}:

1. Fix Ti and Tj at their boundary values.
2. Solve the 4x4 linear system for the remaining four rotors.
3. Accept if all six thrusts lie in [-1e-6, T_ub + 1e-6].
4. Keep the feasible candidate minimizing sum(Tk).

With failed rotor f: T_ub_f = 0, T_ub_k = Tmax otherwise.

---

### 5.4 Hover Power Proxy

Derived from actuator disk theory (P ~ T^1.5 / sqrt(2 * rho * A));
the density factor cancels in the normalized ratio:

    A = pi * (d_prop / 2)^2

    P_proxy(T) = sum_k[ max(Tk, 0)^1.5 ] / sqrt(A)

---

### 5.5 Structural Proxy

    S_arm = sum_i[ L_arm(i) ]     (total arm span)

    BI = Tmax * S_arm / 6         (bending index)

    BI_norm = BI / (Tmax_0 * Lyo_0) = (Tmax * S_arm / 6) / (7327 * 5.50)

---

### 5.6 Packaging (Rotor Disk Clearance)

    c_min = min over all pairs i < j of clearance(rotor_i, rotor_j)

    p_pkg = max(0, -c_min / d_prop)

---

## 6. Stage 1 Metrics

All metrics are to be minimized.

### 6.1 Normalized Mass

    f_mass = m_total / m_ref = m_total / 2240.73

### 6.2 Normalized Hover Power

    f_power = P_proxy(T_hover) / P_proxy(T_hover_ref)

### 6.3 Nominal Hover Utilization

    T_avg       = (1/6) * sum_k( T_hover_k )
    f_hover_nom = (T_avg / Tmax)^2

Penalizes designs where rotors are near saturation during nominal hover.

### 6.4 Fault Tolerance - Thrust Margin

    gamma(f)     = 5 * Tmax / (m_total * g)   for each failed rotor f
    gamma_worst  = min_f[ gamma(f) ]
    f_fault_thrust = max(0, gamma_req - gamma_worst)^2

Default gamma_req = 1.5.

### 6.5 Fault Tolerance - Control Effectiveness

Minimum singular value of the scaled matrix:

    M = S * B * diag(T_ub)

    S = diag( 1/(m*g),  1/(m*g*L_max),  1/(m*g*L_max),  1/tau_yaw )

    L_max    = max(Lx, Lyi, Lyo)
    tau_yaw  = sum_k[ |B_row3_k| * T_ub_k ]

Worst case over all single-rotor failures:

    sigma_worst = min_f[ sigma_min( M_faulted(f) ) ]   (current design)
    sigma_ref   = min_f[ sigma_min( M_faulted(f) ) ]   (baseline design)

    f_fault_alloc = sigma_ref / sigma_worst

Values > 1 indicate degraded control authority; values < 1 indicate
improvement relative to baseline.

### 6.6 Structural

    f_structural = BI_norm    (disabled by default; weight = 0.0)

### 6.7 Packaging

    f_packaging = p_pkg       (disabled by default; weight = 0.0)

---

## 7. Constraints

All constraints listed below are hard; violation triggers infeasibility.
Constraints are registered in two places: `HexacopterArchitecture::registerDefaultConstraints()`
for architecture-level constraints, and element `registerConstraints()` for element-level ones.

### 7.1 Architecture-level constraints

| Stable ID               | Expression                                    | Bound   | Penalty |
|-------------------------|-----------------------------------------------|---------|---------|
| parameter_bounds        | max over-bound violation across all params    | <= 0    | 1000    |
| minimum_geometry_margin | min(Lx-min_arm, Lyi-min_arm, Lyo-Lyi-delta)  | >= 0    | 1000    |
| rotor_clearance         | minimum rotor-to-rotor disk clearance c_min   | >= 0    | 1000    |
| failed_hover_gamma      | gamma_worst = 5*Tmax/(m*g)                    | >= 1.5  | 2000    |
| fault_allocation_ratio  | sigma_worst / sigma_ref                       | >= 0.05 | 1500    |
| arm_yield_failure       | struct_net_min_safety_factor (Phase 3)        | >= 1.5  | 2000    |
| battery_energy_reserve  | bat_energy_reserve_fraction (Phase 2)         | >= 0    | 2000    |
| battery_crate_limit     | bat_c_rate / C_allow - 1 (Phase 2)            | <= 0    | 1500    |
| arm_tip_deflection      | (delta_max / delta_allow) - 1 (Phase 4)       | <= 0    | 1500    |
| arm_tip_rotation        | (theta_max / theta_allow) - 1 (Phase 4)       | <= 0    | 1500    |

`min_arm = 0.5 m` (minimum_arm_length); `delta = 0.1 m` (minimum_outer_arm_delta);
`delta_allow = 0.10 m`; `theta_allow = 0.10 rad` (~5.7 deg).

### 7.2 Element-level constraints

| Stable ID                | Owner   | Expression             | Bound | Penalty |
|--------------------------|---------|------------------------|-------|---------|
| body_span_order          | body    | min(Lx, Lyi, Lyo-Lyi) | >= 0  | 1000    |
| payload_mass_nonnegative | payload | m_pay                  | >= 0  | 1000    |
| battery_mass_positive    | battery | m_bat                  | >= 0  | 500     |
| arm_length_positive      | arm_i   | L_arm(i)               | >= 0  | 750     |
| motor_thrust_positive    | motor_i | Tmax                   | >= 0  | 1000    |
| rotor_diameter_positive  | rotor_i | d_prop                 | >= 0  | 750     |

### 7.3 Stage1Evaluator-appended constraint

| Stable ID                    | Hard when                            | What it checks                              |
|------------------------------|--------------------------------------|---------------------------------------------|
| acs::all_faults_hover_feasible | require_all_fault_acs_feasible=true | all 6 single-fault hover LP trims feasible |

A design is **feasible** when: (a) nominal hover trim is solvable, (b) all
six single-motor-fault hover trims are solvable (when required), and (c)
all hard constraints hold.

---

## 8. Objective Aggregation

### 8.1 Combined (SOO) Objective

    J = sum_i( w_i * f_i ) / sum_i( w_i )

Default weights (from `EvaluationContext::objective_weights`):

| Objective          | Weight | Notes                                        |
|--------------------|--------|----------------------------------------------|
| mass               | 0.20   |                                              |
| power              | 0.20   |                                              |
| fault_thrust       | 0.25   |                                              |
| fault_alloc        | 0.25   |                                              |
| hover_nom          | 0.10   |                                              |
| acs_margin_penalty | 0.10   | zero when ACS is healthy; soft penalty term  |
| structural         | 0.00   | analysis only unless weight raised           |
| packaging          | 0.00   | analysis only unless weight raised           |
| structural_safety  | 0.00   | analysis only unless weight raised           |

Infeasible designs:

    J = 1e6 + sum_i( penalty_i * |constraint_violation_i| )

### 8.2 MOO Objective Vector

Default set (all minimized):

    f_MOO = [f_mass, f_power, f_fault_thrust, f_fault_alloc, f_hover_nom]

Configurable via `MooRunConfig::objective_names`. Note: `acs_margin_penalty` is not
included in the MOO vector by default but can be added via `objective_names`.

---

## 9. Optimization Runners

### 9.1 Single-Objective (SOO) - CMA-ES

| Parameter       | Default |
|-----------------|---------|
| Population size | 24      |
| Generations     | 40      |
| Seed            | 42      |
| f-tolerance     | 1e-6    |

Minimizes J_combined. Tracks best raw and best feasible results separately.

### 9.2 Multi-Objective (MOO) - NSGA-II

| Parameter       | Default        |
|-----------------|----------------|
| Population size | 32 (mult of 4) |
| Generations     | 60             |
| Seed            | 42             |
| Crossover prob. | 0.95           |
| Crossover eta   | 10.0           |
| Mutation eta    | 50.0           |
| Mutation rate   | 1 / n_vars     |

Output: full population, feasible sub-population, per-point objective vectors.
Pareto front identified by `ParetoAnalyzer`.

### 9.3 Design Vector Mapping

Only active parameters are included. `DesignVectorMapper` converts between
raw parameter values and normalized Pagmo bounds. Each active parameter
contributes one dimension.

---

## 10. Baseline Values Summary

### 10.1 Internal scaling references (calibration constants — do not optimize)

These constants appear in `VehicleScalingModel.cpp` and `Elements.cpp` and calibrate
allometric scaling formulas. They were fixed when the MATLAB vehicle_model.m was ported.

| Quantity              | Value        | Used for                                         |
|-----------------------|--------------|--------------------------------------------------|
| reference_mass (m_ref)| 2240.73 kg   | mass objective normalization: f_mass = m_total/m_ref |
| baseline_payload      | 1500 kg      | frame mass reference: m_frame_ref = m_ref - 1500 - 6*m_mot_0 |
| m_mot_0 (per motor)   | 74.07 kg     | motor mass allometric baseline                   |
| Tmax_0 (motor ref)    | 7327 N       | motor mass and BI normalization reference thrust  |
| Lyo_0                 | 5.50 m       | BI normalization reference arm length             |
| Ix_body               | 12000 kg*m^2 | body inertia correction                          |
| Iy_body               | 9400 kg*m^2  | body inertia correction                          |
| Arm span ref (S_arm_0)| 21.60 m      | frame mass scaling reference                     |

Note: `baseline_payload = 1500 kg` is the CALIBRATION constant embedded in the
frame mass formula — it is not the optimizer's payload. The parameter default is
`m_payload = 800 kg`.

### 10.2 Parameter defaults (optimizer start point)

| Parameter          | Default  |
|--------------------|----------|
| Lx                 | 2.65 m   |
| Lyi                | 2.65 m   |
| Lyo                | 5.50 m   |
| T_max              | 12000 N  |
| cT                 | 0.03     |
| d_prop             | 0.40 m   |
| m_payload          | 800 kg   |
| arm_outer_radius   | 0.08 m   |
| arm_wall_thickness | 0.005 m  |
| m_bat              | 400 kg   |

---

## 11. Phase 2: Powertrain and Battery (D, E)

Added by `PowertrainEvaluator` and `BatteryEvaluator` in Stage1Evaluator after ACS.

### 11.1 Powertrain (actuator-disk model)

Effective disk radius: `r_eff = max_arm_length / 2` (NOT d_prop — d_prop is frozen for cT only).

    A_eff = pi * r_eff^2
    P_i   = (T_i^1.5) / (FM * eta_mot * eta_esc * sqrt(2 * rho * A_eff))

where FM = figure_of_merit = 0.65, eta_mot = 0.85, eta_esc = 0.95, rho = 1.225 kg/m^3.

Worst-fault motor: the fault case with the highest total trim thrust sum.

### 11.2 Battery (energy budget)

    E_avail = eta_pack * DoD * m_bat * e_spec
    E_req   = P_nom * t_nom + P_fault * t_emg + P_aux * (t_nom + t_emg)

Default context: e_spec=250 Wh/kg, DoD=0.85, eta_pack=0.95, t_nom=6 min, t_emg=1 min, P_aux=500 W.

Feasibility:
- `battery_energy_reserve`: (E_avail - E_req) / E_avail >= 0
- `battery_crate_limit`: C_rate = P_peak / E_avail <= 5.0 h^-1

m_bat creates a feedback loop: heavier battery → more mass → more hover thrust → more power → more energy needed.

---

## 12. Phase 3: Structural Network (A, C)

`StructuralNetworkAnalyzer` runs AFTER ACS (needs fault trim thrusts), BEFORE constraints.

**Load cases (8):** max_thrust (all rotors at T_max), nominal_hover (ACS LP trim),
fault_0..fault_5 (6 single-motor fault trims).

**Arm model (horizontal cantilever):** tube cross-section defined by `arm_outer_radius` (r_o)
and `arm_wall_thickness` (t_w). Inner radius r_i = r_o - t_w.

    I = pi/4 * (r_o^4 - r_i^4)    (second moment of area)
    A = pi * (r_o^2 - r_i^2)       (cross-section area)

**Stress model (NED, thrust vertical):** Axial load N=0 (thrust perpendicular to arm axis).
Bending: vertical load T_i -> bending moment M_v = T_i * L_arm; yaw reaction -> M_h = cT * T_i.
Combined bending: M = sqrt(M_v^2 + M_h^2). Torsion tau=0 (dominant load is pure bending).

    sigma_vm = M * r_o / I

**Safety factor:** SF = sigma_yield / sigma_vm. Arm material default: Al7075 (sigma_yield=503 MPa).

**Deflection (Euler-Bernoulli, worst distributed+tip load):**

    delta = P*L^3/(3EI) + q*L^4/(8EI)
    theta = P*L^2/(2EI) + q*L^3/(6EI)

where P = concentrated tip load (motor + rotor mass * g), q = distributed arm self-weight per unit length.

Outputs: `struct_net_min_safety_factor`, `struct_net_max_tip_deflection_m`,
`struct_net_max_tip_rotation_rad`, `struct_net_max_sigma_vm_pa`.

---

## 13. Phase 4: Stiffness / Deflection Constraints (B)

Two hard constraints on worst-case tip deflection and rotation over all arms x load cases:

    arm_tip_deflection: (delta_max / delta_allow) - 1 <= 0    (delta_allow = 0.10 m)
    arm_tip_rotation:   (theta_max / theta_allow) - 1 <= 0    (theta_allow = 0.10 rad)

Penalty 1500 each. At default geometry (Lyo=5.5 m, r_o=0.08 m, t_w=0.005 m, T_max=12 kN):
delta_max ~ 1.27 m → violation ~ 10.7 (1270% over limit). Optimizer must increase wall thickness
or reduce arm length.

---

## 14. Phase 5: MetricRole Labeling

Every metric and constraint carries an explicit `MetricRole`:

| Role             | Meaning                                                         |
|------------------|-----------------------------------------------------------------|
| hard_constraint  | Feasibility gate — violation makes design infeasible           |
| soft_objective   | Enters combined_objective with positive weight                  |
| analysis_only    | Computed and exported; no direct optimization role              |

`MetricDescriptor` table (`stage1MetricDescriptors()`) maps all ~35 Stage1Metrics fields.
JSON output: `objectives[]` and `constraints[]` include `"role"` field.
`metric_descriptors[]` array in each evaluation block gives role + unit for every metric.
