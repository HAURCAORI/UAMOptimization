# IMPLEMENTATION_GUIDE_STAGE1_A_TO_G.md

## Purpose

This guide rewrites the next implementation plan for the fixed-configuration hexacopter Stage 1 framework.

The motor/rotor **topology and configuration are fixed**.  
Therefore, this guide does **not** ask the framework to optimize rotor count, rotor placement, or spin-layout topology.

The goal is to strengthen the current framework in these seven areas:

- **A. multi-member structural network**
- **B. stiffness / deflection constraints**
- **C. better load propagation**
- **D. powertrain limits**
- **E. battery sizing**
- **F. clearer hard/soft separation**
- **G. attainable control set (ACS)**

This is still a **Stage 1 reduced-order framework**.

It is **not**:
- Stage 2 mission simulation
- nonlinear recovery simulation
- FEM
- CAD / mesh geometry
- arbitrary rotor-topology optimization

---

# 1. Fixed assumptions

## 1.1 Vehicle topology
The hexacopter motor configuration is fixed:
- rotor count is fixed
- rotor positions are fixed
- rotor spin directions are fixed
- linearized / reduced-order control-allocation assumptions remain valid

The architecture design variables are therefore limited to:
- structural member sizing
- battery sizing
- selected placement offsets of non-rotor subsystems
- packaging-related geometry
- power and energy related parameters
- mass distribution and related proxies

## 1.2 Modeling level
Use reduced-order engineering models only:
- beam / shaft formulas
- graph-based load propagation
- actuator-limit-based ACS analysis
- hover-oriented power and energy proxies

## 1.3 Design philosophy
Not all `SpatialElement`s are structural.  
The framework must explicitly separate element responsibilities.

---

# 2. Capability model and element responsibilities

## 2.1 Core rule
Every element may contribute in one or more of the following roles:

- **geometry only**
- **mass/inertia contributor**
- **structural member**
- **load receiver**
- **load source**
- **energy storage**
- **propulsion actuator**
- **analysis-only reference object**

No element should be forced to implement structural equations unless it actually carries the `IStructuralMember` capability.

## 2.2 Required capability split

### Structural members
Examples:
- arm
- body frame member
- battery support member
- payload support member
- landing support member
- motor mount member

Required capabilities:
- `IStructuralMember`
- usually `ILoadReceiver`

### Non-structural mass elements
Examples:
- battery pack
- payload
- avionics box
- fairing
- controller box

Required capabilities:
- `IMassContributor`
- optionally `ILoadSource`
- optionally `ILoadReceiver` if mounted and load-transmitting

### Propulsion elements
Examples:
- motor
- rotor

Required capabilities:
- `IPropulsionRotor`
- `IMassContributor`
- `ILoadSource`
- optionally `IMotorPowerConsumer`

### Battery elements
Examples:
- battery module / pack

Required capabilities:
- `IEnergyStorage`
- `IMassContributor`
- `ILoadSource` through gravity
- optional packaging geometry contribution

## 2.3 Active parameter policy

### Structural-member parameters
Typical active parameters:
- member length `L`
- outer diameter `D_o`
- inner diameter `D_i` or thickness `t`
- material id / material struct
- section type
- mount offset if permitted

### Battery parameters
Typical active parameters:
- battery mass `m_bat`
- battery capacity `Q_pack`
- nominal voltage `V_nom`
- usable depth of discharge `DoD_use`
- specific energy `e_spec`
- position offset

### Powertrain parameters
If motor configuration is fixed, keep geometry fixed but allow limit parameters if needed:
- maximum thrust `T_max`
- continuous power `P_cont`
- peak power `P_peak`
- torque/thrust ratio proxy `gamma = Q/T`
- efficiency parameters

### Non-structural placement parameters
Typical active parameters:
- local attachment offsets
- body/battery/payload mounting offsets
- enclosure dimensions

---

# 3. A. Multi-member structural network

## 3.1 Objective
Replace the arm-only structural proxy with a **network of structural members** that can:
- receive loads,
- transmit loads,
- accumulate internal resultants,
- and return stress / stiffness metrics.

## 3.2 Structural graph

Define a structural graph:

- nodes: attachment points / structural connection points
- edges: structural members

For each structural member `e`:
- endpoints: `a`, `b`
- world positions: `x_a`, `x_b`
- length:
\[
L_e = \|x_b - x_a\|
\]
- local unit axis:
\[
\hat{e}_e = \frac{x_b - x_a}{L_e}
\]

## 3.3 Section properties

### Circular hollow section
Use this as the default Stage 1 structural member section.

Cross-sectional area:
\[
A = \frac{\pi}{4}\left(D_o^2 - D_i^2\right)
\]

Second moment of area:
\[
I = \frac{\pi}{64}\left(D_o^4 - D_i^4\right)
\]

Polar moment:
\[
J = \frac{\pi}{32}\left(D_o^4 - D_i^4\right)
\]

Outer radius:
\[
c = \frac{D_o}{2}
\]

### Rectangular hollow section (optional later)
If added later, define:
- `A`
- `I_y`
- `I_z`
- `J` using the chosen thin-wall or closed-section approximation.

## 3.4 Material model

For Stage 1, start with **linear elastic isotropic materials** only.

Each structural material must provide:
- density `rho`
- Young's modulus `E`
- shear modulus `G`
- yield strength `sigma_y`
- allowable shear `tau_allow`
- optional safety factor policy

Recommended material struct:
```cpp
struct Material {
    std::string id;
    double rho;
    double E;
    double G;
    double sigma_y;
    double tau_allow;
};
```

Do **not** hard-code one global material.  
Each structural member should own or reference its own material.

## 3.5 Internal resultants per member

For a member root or analysis section, represent the internal load as:
- axial force `N`
- shear force vector `V`
- bending moment vector `M_b`
- torsion `T`

Given total propagated wrench at the member root:
\[
\mathbf{w}_e =
\begin{bmatrix}
\mathbf{F}_e \\
\mathbf{M}_e
\end{bmatrix}
\]

Decompose with the member axis `\hat{e}_e`.

Axial force:
\[
N_e = \mathbf{F}_e \cdot \hat{e}_e
\]

Transverse force vector:
\[
\mathbf{V}_e = \mathbf{F}_e - N_e \hat{e}_e
\]

Torsion:
\[
T_e = |\mathbf{M}_e \cdot \hat{e}_e|
\]

Bending-moment vector:
\[
\mathbf{M}_{b,e} = \mathbf{M}_e - (\mathbf{M}_e \cdot \hat{e}_e)\hat{e}_e
\]

Bending-moment magnitude:
\[
M_{b,e} = \|\mathbf{M}_{b,e}\|
\]

## 3.6 Stress equations

Axial stress:
\[
\sigma_{ax,e} = \frac{N_e}{A_e}
\]

Bending stress at the outer fiber:
\[
\sigma_{b,e} = \frac{M_{b,e} c_e}{I_e}
\]

Torsional shear stress:
\[
\tau_{t,e} = \frac{T_e c_e}{J_e}
\]

Use a reduced-order von Mises equivalent stress:
\[
\sigma_{vm,e} = \sqrt{(\sigma_{ax,e} + \sigma_{b,e})^2 + 3\tau_{t,e}^2}
\]

Member safety factor:
\[
SF_e = \frac{\sigma_{y,e}}{\sigma_{vm,e}}
\]

## 3.7 Structural outputs
For each structural member, export:
- `N`
- `|V|`
- `M_b`
- `T`
- `sigma_vm`
- `SF`
- `delta_tip`
- `theta_tip`

At the architecture level, export:
- worst safety factor
- max stress ratio
- max deflection ratio
- max torsion ratio

---

# 4. B. Stiffness / deflection constraints

## 4.1 Objective
Stress constraints are not enough.  
Add deflection and rotational compliance constraints.

## 4.2 Member-level deflection formulas

Use Euler-Bernoulli beam theory for Stage 1.

### Cantilever with end transverse load
For transverse end load `P`:
\[
\delta = \frac{P L^3}{3EI}
\]
\[
\theta = \frac{P L^2}{2EI}
\]

### Cantilever with uniformly distributed load `q`
\[
\delta = \frac{qL^4}{8EI}
\]

### Axial extension
\[
\delta_{ax} = \frac{N L}{AE}
\]

### Torsional twist
\[
\phi = \frac{T L}{GJ}
\]

## 4.3 Practical Stage 1 deflection model
For each structural member, compute:
- transverse load magnitude `P = ||V||`
- use the cantilever formula as the first reduced-order approximation
- if the member is not cantilever-like, document the approximation clearly

## 4.4 Hard stiffness constraints
Recommended hard constraints:
\[
g_{\delta,e} = \frac{\delta_e}{\delta_{allow,e}} - 1 \le 0
\]
\[
g_{\theta,e} = \frac{\theta_e}{\theta_{allow,e}} - 1 \le 0
\]
\[
g_{\phi,e} = \frac{\phi_e}{\phi_{allow,e}} - 1 \le 0
\]

At the architecture level:
\[
g_{\delta,max} = \max_e g_{\delta,e} \le 0
\]

## 4.5 Design note
Do **not** require every element to compute stiffness.  
Only elements with `IStructuralMember` contribute.

---

# 5. C. Better load propagation

## 5.1 Objective
Move from isolated local force approximations to **graph-based attachment-aware load propagation**.

## 5.2 Load cases

Implement a Stage 1 load-case library.

Minimum required load cases:
1. nominal hover
2. one-rotor-failed hover
3. maximum-thrust structural case
4. gravity/payload structural case
5. asymmetric thrust redistribution case

Represent each load case as:
```cpp
struct LoadCase {
    std::string id;
    std::string description;
    // scenario parameters, fault multipliers, etc.
};
```

## 5.3 Applied loads

Each load source contributes a wrench.

### Gravity load on element `i`
\[
\mathbf{F}_{g,i} = m_i \mathbf{g}
\]
\[
\mathbf{M}_{g,i} = (\mathbf{r}_i - \mathbf{r}_{ref}) \times \mathbf{F}_{g,i}
\]

### Rotor thrust load for rotor `i`
Let `\hat{a}_i` be the rotor thrust axis and `f_i` the thrust magnitude.

\[
\mathbf{F}_{T,i} = f_i \hat{a}_i
\]

Reaction torque:
\[
\mathbf{M}_{Q,i} = s_i \gamma_i f_i \hat{a}_i
\]
where:
- `s_i ∈ {+1,-1}` is the spin sign
- `gamma_i = Q_i / T_i` or a fixed yaw-torque proxy

Moment about reference point:
\[
\mathbf{M}_{T,i} = (\mathbf{r}_i - \mathbf{r}_{ref}) \times \mathbf{F}_{T,i} + \mathbf{M}_{Q,i}
\]

## 5.4 Propagation rule

### Stage 1 implementation rule
For each applied load:
1. assign the load at the source element
2. walk upward through the attachment graph
3. accumulate transmitted wrench at parent receivers
4. record local member-root wrench for every structural member crossed

This is **not** full FEM.  
It is a graph-based reduced-order load transfer.

## 5.5 Member-root resultant
At each member root:
\[
\mathbf{F}_{root} = \sum_{j \in descendants} \mathbf{F}_j
\]
\[
\mathbf{M}_{root} = \sum_{j \in descendants} \left[(\mathbf{r}_j - \mathbf{r}_{root}) \times \mathbf{F}_j + \mathbf{M}_j\right]
\]

Use these root resultants to compute:
- `N`
- `V`
- `M_b`
- `T`

## 5.6 Architecture outputs
For each load case, store:
- per-member loads
- worst member load
- worst member safety factor
- worst member deflection

Global structural metrics should be the worst over all load cases:
\[
SF_{min} = \min_{\lambda \in \mathcal{L}} \min_{e \in \mathcal{E}_s} SF_{e,\lambda}
\]

---

# 6. D. Powertrain limits

## 6.1 Objective
Add actuator and power limits, not just thrust feasibility.

## 6.2 Rotor / motor model
For Stage 1 reduced-order analysis, use:
\[
T_i = k_{T,i}\,\omega_i^2
\]
\[
Q_i = k_{Q,i}\,\omega_i^2
\]
\[
P_{mech,i} = Q_i \omega_i = k_{Q,i}\,\omega_i^3
\]

If thrust `T_i` is used as the optimization/evaluation variable, then:
\[
\omega_i = \sqrt{\frac{T_i}{k_{T,i}}}
\]
and therefore
\[
P_{mech,i} = \frac{k_{Q,i}}{k_{T,i}^{3/2}}\,T_i^{3/2}
\]

Electrical power:
\[
P_{elec,i} = \frac{P_{mech,i}}{\eta_{mot,i}\eta_{esc,i}}
\]

## 6.3 Hard actuator constraints
For each motor/rotor:
\[
0 \le T_i \le T_{i,max}^{eff}
\]
where:
- nominal case: `T_{i,max}^{eff} = T_{i,max}`
- fault case: `T_{i,max}^{eff} = \alpha_i T_{i,max}`, `0 \le \alpha_i \le 1`

Continuous-power constraint:
\[
P_{elec,i} \le P_{cont,i}
\]

Peak-power check if desired:
\[
P_{elec,i} \le P_{peak,i}
\]

## 6.4 Architecture-level limits
Bus or total-pack power:
\[
\sum_i P_{elec,i} + P_{aux} \le P_{pack,max}
\]

Current limit:
\[
I_{pack} = \frac{\sum_i P_{elec,i} + P_{aux}}{V_{pack}} \le I_{cont,pack}
\]

## 6.5 Powertrain metrics
Store at least:
- worst motor thrust utilization
\[
u_{T,max} = \max_i \frac{T_i}{T_{i,max}^{eff}}
\]
- worst motor power utilization
\[
u_{P,max} = \max_i \frac{P_{elec,i}}{P_{cont,i}}
\]
- total electrical power
- nominal and faulted reserve margins

---

# 7. E. Battery sizing

## 7.1 Objective
Treat the battery as a real Stage 1 subsystem, not only a lumped mass.

## 7.2 Battery energy model

### Available energy
Using capacity in ampere-hours:
\[
E_{avail} = \eta_{pack}\,DoD_{use}\,Q_{pack}\,V_{nom}
\]

Using specific energy:
\[
E_{avail} = \eta_{pack}\,DoD_{use}\,m_{bat}\,e_{spec}
\]

These two must be consistent:
\[
m_{bat}\,e_{spec} = Q_{pack}\,V_{nom}
\]

## 7.3 Battery reserve constraints

### Nominal reserve
If nominal hover reserve time is `t_nom`:
\[
E_{req,nom} = P_{nom}\,t_{nom}
\]

### Emergency / fault reserve
If faulted hover or emergency reserve time is `t_emg`:
\[
E_{req,fault} = P_{fault}\,t_{emg}
\]

### Total requirement
\[
E_{req,total} = E_{req,nom} + E_{req,fault} + E_{aux}
\]

Hard constraint:
\[
g_{energy} = \frac{E_{req,total}}{E_{avail}} - 1 \le 0
\]

## 7.4 Battery current / C-rate proxy
If the pack is capacity-limited by current:
\[
I_{pack} = \frac{P_{pack}}{V_{nom}}
\]

Continuous current limit:
\[
I_{pack} \le I_{cont}
\]

Equivalent C-rate:
\[
C_{eq} = \frac{I_{pack}}{Q_{pack}}
\]

Hard or soft limit:
\[
g_C = \frac{C_{eq}}{C_{allow}} - 1 \le 0
\]

## 7.5 Battery sizing outputs
Store:
- available energy
- required nominal energy
- required emergency energy
- reserve fraction
- pack current
- C-rate
- battery mass fraction

---

# 8. F. Clearer hard/soft separation

## 8.1 Objective
Every metric must belong to exactly one of these categories:

- **hard constraint**
- **soft objective / penalty**
- **analysis-only metric**

Do not leave this implicit.

## 8.2 Recommended hard constraints
Use hard constraints for feasibility:

- packaging validity
- no forbidden overlaps
- stress limit
- deflection limit
- torsion/twist limit
- motor thrust limit
- motor power limit
- battery energy reserve
- battery current/C-rate limit
- hover trim feasibility
- faulted hover trim feasibility
- minimum ACS containment / margin

## 8.3 Recommended soft objectives
Use soft objectives for preference/tradeoff:

- total mass
- nominal hover power
- maximum utilization penalty
- structural margin penalty
- battery mass penalty
- ACS margin penalty
- packaging compactness penalty (optional)

## 8.4 Implementation rule
Introduce explicit metadata:
```cpp
enum class MetricRole {
    hard_constraint,
    soft_objective,
    analysis_only
};
```

Every evaluation term must carry this role.

## 8.5 Example weighted objective
A Stage 1 weighted objective may be:
\[
J =
w_m J_{mass} +
w_p J_{power} +
w_s J_{struct} +
w_b J_{battery} +
w_a J_{ACS} +
w_u J_{util}
\]

where the `J_*` terms are normalized soft objectives only.

---

# 9. G. Attainable Control Set (ACS)

## 9.1 Objective
Use the attainable control set as the main controllability/fault-tolerance analysis tool.

## 9.2 Virtual control vector
For the fixed hexacopter Stage 1 problem, use the wrench:
\[
\mathbf{u} =
\begin{bmatrix}
F_z \\
M_x \\
M_y \\
M_z
\end{bmatrix}
\in \mathbb{R}^4
\]

If a more general wrench is needed later, upgrade to 6D:
\[
\mathbf{w} =
\begin{bmatrix}
\mathbf{F} \\
\mathbf{M}
\end{bmatrix}
\in \mathbb{R}^6
\]

## 9.3 Control-allocation matrix
Let the actuator force vector be:
\[
\mathbf{f} =
\begin{bmatrix}
f_1 & \cdots & f_n
\end{bmatrix}^T
\]

Then:
\[
\mathbf{u} = B \mathbf{f}
\]

For a fixed-thrust-axis rotor `i`, define:
- position `r_i`
- thrust axis `a_i`
- spin sign `s_i`
- yaw torque ratio `gamma_i`

The full wrench contribution column is:
\[
b_i =
\begin{bmatrix}
a_i \\
r_i \times a_i + s_i \gamma_i a_i
\end{bmatrix}
\]

For the Stage 1 4D hover-oriented model, use the projected `B` mapping to:
\[
[F_z,\,M_x,\,M_y,\,M_z]^T
\]

## 9.4 Actuator bounds and faults
Actuator bounds:
\[
0 \le f_i \le f_{i,max}^{eff}
\]

Nominal:
\[
f_{i,max}^{eff} = f_{i,max}
\]

Faulted:
\[
f_{i,max}^{eff} = \alpha_i f_{i,max}, \qquad 0 \le \alpha_i \le 1
\]

Complete failure:
\[
\alpha_i = 0
\]

## 9.5 ACS definition
The attainable control set is:
\[
\mathcal{U} = \{ Bf \;|\; 0 \le f \le f_{max}^{eff} \}
\]

This is the image of the actuator hyper-rectangle under the linear map `B`.

## 9.6 Required-wrench containment
For hover or faulted hover, define the required wrench:
\[
u_{req} =
\begin{bmatrix}
m g \\
0 \\
0 \\
0
\end{bmatrix}
\]
or the modified trimmed wrench if CG offset or asymmetric steady loads are included.

Containment test:
find `f` such that
\[
Bf = u_{req}
\]
subject to
\[
0 \le f \le f_{max}^{eff}
\]

If feasible, the required trim wrench is inside the ACS.

## 9.7 Directional margin
For a direction vector `d` in wrench space, define the support function:
\[
h_{\mathcal{U}}(d) = \sum_{i=1}^{n} \max(0, d^T b_i)\,f_{i,max}^{eff}
\]

Directional margin relative to required wrench:
\[
m(d) = h_{\mathcal{U}}(d) - d^T u_{req}
\]

A sufficient sampled-direction check is:
\[
m(d_k) \ge 0 \quad \forall k
\]

Use a set of normalized direction samples `d_k` relevant to:
- pure thrust
- roll
- pitch
- yaw
- mixed roll/yaw
- mixed pitch/yaw

## 9.8 ACS-based metrics

### Hard feasibility metrics
- nominal trim containment
- faulted trim containment

### Soft controllability metrics
- minimum sampled directional margin
\[
m_{min} = \min_k m(d_k)
\]
- yaw reserve
- roll reserve
- pitch reserve
- faulted-to-nominal margin ratio

Optional analysis-only metric:
- approximate ACS volume or hull volume proxy

## 9.9 Recommended ACS role in optimization
Use ACS in three ways:

1. **hard constraint**  
   Required faulted trim must be inside the faulted ACS.

2. **soft objective**  
   Maximize margin to ACS boundary at the required operating point.

3. **analysis metric**  
   Report directional reserves and fault degradation.

---

# 10. Required source materials

Use the following source materials when implementing or validating the above topics.

## Beam / stiffness / stress
- Euler-Bernoulli beam theory lecture notes
- beam deflection tables for cantilever and distributed loads
- torsion of circular shafts notes
- second-moment/polar-moment references for hollow circular sections

## Control allocation / ACS
- constrained control allocation and attainable-moment literature
- control allocation survey references
- generalized multirotor allocation references

## Hover power / battery
- actuator-disk / momentum-theory hover power references
- battery design/safety/use guidelines
- UAV battery/power consumption sizing references

The implementation should remain reduced-order, but the equations must be kept close to these standard engineering references.

---

# 11. Recommended class/module additions

## `core/`
Add or refine:
- `Material`
- `LoadCase`
- `AppliedLoad`
- `IStructuralMember`
- `ILoadReceiver`
- `ILoadSource`
- `IEnergyStorage`
- battery parameter structs
- structural section structs

## `physics/`
Add:
- `StructuralNetworkAnalyzer`
- `LoadPropagationSolver`
- `PowertrainEvaluator`
- `BatteryEvaluator`
- `AttainableControlSetAnalyzer`

## `evaluation/`
Add:
- structural case aggregation
- battery metrics
- powertrain metrics
- ACS containment and margin metrics
- explicit `MetricRole`

## `analysis/`
Add:
- ACS report
- structural-member report
- load-case report
- battery reserve report
- powertrain utilization report

---

# 12. Recommended implementation order

## Phase 1
Implement ACS:
- `B`
- actuator bounds
- faulted bounds
- trim containment
- sampled directional margins

## Phase 2
Implement powertrain + battery:
- motor limits
- power evaluation
- battery reserve constraints

## Phase 3
Implement structural network:
- multi-member support
- load propagation
- stress metrics

## Phase 4
Implement stiffness/deflection constraints

## Phase 5
Refactor hard/soft metric roles and export/reporting

---

# 13. Acceptance criteria

The implementation is acceptable only if all of the following are true:

1. Not all `SpatialElement`s are treated as structural members.
2. Structural equations are applied only to elements carrying structural capability.
3. Load cases are explicit and attachment-aware.
4. Structural stress and deflection are reported per member and per load case.
5. Powertrain and battery limits are enforced explicitly.
6. Hard and soft terms are labeled separately in evaluation outputs.
7. ACS containment is used as a fault-tolerance feasibility check.
8. ACS margins are exported as analysis and/or optimization metrics.
9. The framework remains Stage 1 reduced-order and does not drift into unbounded high-fidelity scope.

---

# 14. Final design rule

The next version should not try to become a full aircraft simulator.

It should become a **clean, reduced-order, multi-disciplinary Stage 1 co-design framework** with:

- structural member network logic,
- stiffness-aware constraints,
- attachment-aware load propagation,
- powertrain and battery feasibility,
- explicit hard/soft optimization semantics,
- and ACS-based controllability analysis.
