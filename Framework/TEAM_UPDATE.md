# Team Update
## 1. Current Stage
- Optimization framework implemented
- Single-motor fault screening implemented
- SOO (Single Objective Optimization) workflow running
- MOO (Multi Objective Optimization) workflow running
- Parameterized mass and inertia model connected
- Mission validation not completed

There are currently two optimization directions under consideration:
- ACS-based optimization
- Mission-based optimization

## 2. Current Design Variables
- Active variables: `Lx`, `Lyi`, `Lyo` (motor placement), `T_max` (maximum thrust)
- Reserved variable
  `d_prop` exists in the code and is frozen by default
- Current meaning
  geometry, thrust sizing, fault-tolerant hover capability, cost coupling

## 3. Current Objective
- Main terms: `FII`, `J_hover`, `J_cost`
- Meaning
  fault-balance, hover survivability, parameterized cost
- Current mode
  ACS / hover-based optimization
- Not yet included in the real optimization loop
  mission performance, path-following performance, recovery performance

## 4. Current Optimization Status
- SOO
  one best Stage 1 design
- MOO
  Pareto trade-off between fault tolerance and cost
- Current result trend
  improved `FII`, positive hover margin, `6/6` hover-feasible designs
- Current limitation
  full simulation still diverges

This means the current Stage 1 result is reasonable as a screening /
optimization result, but not yet as a fully mission-validated result.

## 5. What the Team Should Decide Next
- Final decision variables
  geometry only, or additional propulsion / structure variables
- Final objectives
  ACS / hover only, or ACS + mission tracking + recovery
- Final novelty direction
  parameterized design model, staged optimization, mission-coupled MDO

  