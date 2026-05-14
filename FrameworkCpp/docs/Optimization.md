
## FrameworkCpp - Optimization Reference

This document describes the hexacopter design optimization in `FrameworkCpp`:
design parameters, spatial element models, physical equations, constraints,
and optimizer configuration.

---

## 1. Architecture Overview

`HexacopterArchitecture` is the root object. It owns a `ParameterRegistry`,
a `ConstraintRegistry`, a list of `SpatialElement` objects, and a list of
`Attachment` kinematic links. After any parameter change,
`updateFromParameters()` re-evaluates every element and re-assembles the
scene graph.

Data flow:

    ParameterRegistry --> SpatialElement x N --> AssemblyState
                                                      |
                                            VehicleScalingModel
                                                      |
                                              PhysicalModel
                                                      |
                                             Stage1Evaluator
                                                      |
                                           EvaluationResult
                                          /               \
                                  Stage1Metrics     ConstraintResults

---

## 2. Design Parameters

Parameters are stored in `ParameterRegistry` with stable key
`"<architecture-id>::<name>"`.

| Name       | Symbol  | Unit | Default | Bounds        | Active |
|------------|---------|------|---------|---------------|--------|
| Lx         | Lx      | m    | 2.65    | [1.0, 5.0]    | yes    |
| Lyi        | Lyi     | m    | 2.65    | [1.0, 5.0]    | yes    |
| Lyo        | Lyo     | m    | 5.50    | [2.5, 9.0]    | yes    |
| T_max      | Tmax    | N    | 7327    | [8000, 16000] | yes    |
| cT         | cT      | -    | 0.03    | [0.01, 0.08]  | no     |
| d_prop     | d_prop  | m    | 0.40    | [0.20, 1.20]  | no     |
| m_payload  | m_pay   | kg   | 1500    | [1000, 2500]  | no     |

Active parameters are included in the optimizer's design vector. Inactive
parameters are fixed at their defaults during a run but can be set beforehand.

**Normalization.** The optimizer works in normalized space:

    x_norm = (x - x_default) / scale    (scale = 1.0 for all parameters)

Normalized bounds: [x_lower - x_default, x_upper - x_default].
The default maps to x_norm = 0.

---

## 3. Spatial Elements

### 3.1 Reference Constants

| Symbol    | Value          | Description                        |
|-----------|----------------|------------------------------------|
| m_ref     | 2240.73 kg     | Baseline total vehicle mass        |
| m_pay_0   | 1500.0 kg      | Baseline payload mass              |
| m_mot_0   | 74.07 kg       | Baseline motor mass                |
| Tmax_0    | 7327.0 N       | Baseline motor maximum thrust      |
| Lx_0      | 2.65 m         | Baseline fore/aft arm length       |
| Lyi_0     | 2.65 m         | Baseline inner lateral arm length  |
| Lyo_0     | 5.50 m         | Baseline outer lateral arm length  |
| d_prop_0  | 0.40 m         | Baseline propeller diameter        |
| Ix_0      | 12000 kg*m^2   | Baseline body inertia about x-axis |
| Iy_0      | 9400 kg*m^2    | Baseline body inertia about y-axis |
| g         | 9.81 m/s^2     | Gravitational acceleration         |

---

### 3.2 BodyElement

Central chassis. Zero mass; serves as the attachment root for all other
elements.

**Parameters consumed:** Lx, Lyi, Lyo

**Geometry (box half-extents):**

    hx = max(0.35, 0.10*Lx  + 0.10)
    hy = max(0.35, 0.10*Lyi + 0.10)
    hz = 0.20

Padding = 0.05 m. Mass = 0.

**Anchors:**

| Anchor | Local translation |
|--------|-------------------|
| center | (0, 0, 0)         |
| top    | (0, 0, +hz)       |
| bottom | (0, 0, -hz)       |

**Constraints registered:**
- `body_span_order`: min(Lx, Lyi, Lyo - Lyi) >= 0

---

### 3.3 ArmElement (x6)

Six structural arms connecting the body to the motors.

**Parameters consumed:** Lx, Lyi, Lyo, Tmax

**Arm length by index:**

    L_arm(i) = Lyo                      if i in {2, 3}  (outer lateral)
             = sqrt(Lx^2 + Lyi^2)       otherwise       (diagonal)

**Frame mass (shared across all six arms):**

    S_arm     = 2*Lx + 2*Lyi + 2*Lyo
    S_arm_0   = 2*2.65 + 2*2.65 + 2*5.50 = 21.60 m

    m_frame_0 = m_ref - m_pay_0 - 6*m_mot_0
              = 2240.73 - 1500.0 - 444.42 = 296.31 kg

    m_frame(Lx, Lyi, Lyo) = m_frame_0 * (S_arm / S_arm_0)

    m_arm(i) = m_frame / 6

**Inertia (arm local frame, about mid-span):**

    Ixx = 1e-6
    Iyy = Izz = m_arm * L_arm^2 / 12

**Geometry:** segment of length L_arm, radius 0.05 m.

**Anchors:**

| Anchor | Local translation |
|--------|-------------------|
| root   | (0, 0, 0)         |
| tip    | (L_arm, 0, 0)     |

**Capabilities:** `IStructuralMember` - contributes L_arm to the total
structural span; its mass counts toward `frame_mass`.

**Constraints registered:**
- `arm_length_positive`: L_arm(i) >= 0

---

### 3.4 MotorElement (x6)

Motor + ESC assembly. Mass scales allometrically with peak thrust.

**Parameters consumed:** Tmax

**Motor mass (allometric scaling):**

    m_mot(Tmax) = m_mot_0 * (Tmax / Tmax_0)^(3/3.5)

**Geometry (cylinder, rotated 90 deg about x):**

    r_mot = 0.08 + 0.04 * (Tmax / Tmax_0)
    h_mot = 0.12 m

**Anchors:** `mount`, `axis` (both at origin).

**Capabilities:** `IMotorMassContributor` - world-frame position used to
compute inertia corrections.

**Constraints registered:**
- `motor_thrust_positive`: Tmax >= 0

---

### 3.5 RotorElement (x6)

Propeller disk. Zero mass; position and yaw sign feed the allocation matrix.

**Parameters consumed:** d_prop

**Geometry:** disk of radius d_prop/2, padding 0.01 m.

**Yaw moment signs** (index 0 to 5): [-1, -1, +1, +1, -1, +1]

**Anchors:** `axis`, `center` (both at origin).

**Capabilities:** `IPropulsionRotor` - provides `rotorIndex()` and
`yawMomentSign()` to `VehicleScalingModel`.

**Constraints registered:**
- `rotor_diameter_positive`: d_prop >= 0

---

### 3.6 BatteryElement

Geometry-only placeholder. Mass = 0 in the physical model.

**Parameters consumed:** Tmax, d_prop

**Geometry (box):**

    L_bat = 0.35 + 0.08 * (Tmax / Tmax_0)
    w_bat = 0.25 + 0.10 * d_prop
    h_bat = 0.12 m  (half-height)

**Anchors:** `mount`, `center` (both at origin).

**Constraints registered:**
- `battery_sizing_inputs`: min(Tmax, d_prop) >= 0

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

All constraints are hard; violation triggers infeasibility.

| Stable ID                | Expression                         | Bound   | Penalty |
|--------------------------|------------------------------------|---------|---------|
| parameter_bounds         | max bound violation per param      | <= 0    | 1000    |
| minimum_geometry_margin  | min(Lx-0.5, Lyi-0.5, Lyo-Lyi-0.1) | >= 0    | 1000    |
| rotor_clearance          | c_min                              | >= 0    | 1000    |
| failed_hover_gamma       | gamma_worst                        | >= 1.5  | 2000    |
| fault_allocation_ratio   | sigma_worst / sigma_ref            | >= 0.05 | 1500    |
| payload_mass_nonnegative | m_pay                              | >= 0    | 1000    |
| battery_sizing_inputs    | min(Tmax, d_prop)                  | >= 0    | 500     |
| body_span_order          | min(Lx, Lyi, Lyo - Lyi)            | >= 0    | 1000    |
| arm_length_positive      | L_arm(i)                           | >= 0    | 750     |
| motor_thrust_positive    | Tmax                               | >= 0    | 1000    |
| rotor_diameter_positive  | d_prop                             | >= 0    | 750     |

A design is **feasible** when: (a) nominal hover is solvable, (b) single-
rotor failure hover is solvable for all six rotors, and (c) all hard
constraints hold.

---

## 8. Objective Aggregation

### 8.1 Combined (SOO) Objective

    J = sum_i( w_i * f_i ) / sum_i( w_i )

Default weights:

| Objective    | Weight |
|--------------|--------|
| mass         | 0.20   |
| power        | 0.20   |
| fault_thrust | 0.25   |
| fault_alloc  | 0.25   |
| hover_nom    | 0.10   |
| structural   | 0.00   |
| packaging    | 0.00   |

Infeasible designs:

    J = 1e6 + sum_i( penalty_i * |constraint_violation_i| )

### 8.2 MOO Objective Vector

Default set (all minimized):

    f_MOO = [f_mass, f_power, f_fault_thrust, f_fault_alloc, f_hover_nom]

Configurable via `MooRunConfig::objective_names`.

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

| Quantity          | Value        |
|-------------------|--------------|
| Total mass        | 2240.73 kg   |
| Frame mass        | 296.31 kg    |
| Motor mass (each) | 74.07 kg     |
| Payload           | 1500 kg      |
| Tmax (per rotor)  | 7327 N       |
| Arm span S_arm    | 21.60 m      |
| Ix body           | 12000 kg*m^2 |
| Iy body           | 9400 kg*m^2  |
| d_prop            | 0.40 m       |
| cT                | 0.03         |
