# Implementation Step 2 — Packaging Infrastructure

## Status entering Step 2

Phases 1–5 are complete:

| Phase | Content | Status |
|---|---|---|
| Phase 1 | ACS analysis (trim, margins, directional reserves, named constraint) | Done |
| Phase 2 | Powertrain (actuator-disk) + Battery sizing (`m_bat` variable) | Done |
| Phase 3 | Structural network analyzer (multi-load-case von Mises + deflection) | Done |
| Phase 4 | Stiffness/deflection hard constraints | Done |
| Phase 5 | `MetricRole` enum + descriptor table + JSON `"role"` fields | Done |
| Power unification | `power` objective now uses `PowertrainEvaluator` (arm-geometry disk area) | Done |

Active design variables (7): `Lx`, `Lyi`, `Lyo`, `T_max`, `arm_outer_radius`, `arm_wall_thickness`, `m_bat`.

---

## Step 2 Scope

**Primary goal:** Separate packaging analysis from `VehicleScalingModel` into a dedicated
`ArchitecturePackagingEvaluator`; add `IEnvelopeProvider` as an architectural capability interface.
This decouples mass/geometry/propulsion scaling from spatial reasoning and enables future
containment and keep-out constraints without modifying VehicleScalingModel.

**Secondary goal:** Fix two correctness bugs (B1, B2) that have been deferred since the initial
audit.

### In scope

1. `LocalAABB` struct + `IEnvelopeProvider` capability interface
2. `IEnvelopeProvider` implementation on `BodyElement` and `BatteryElement`
3. `ArchitecturePackagingEvaluator` class (moves rotor clearance out of VehicleScalingModel)
4. Rename `PackagingReport::minimum_clearance` → `minimum_rotor_clearance` (field semantics)
5. New `Stage1Metrics` field: `pkg_rotor_clearance_m`
6. `MetricRole` descriptor entry for the new metric
7. Bug fix B1: `DesignParameter::normalizedAt` missing `scale == 0` guard
8. Bug fix B2: `Constraint::evaluate` unguarded callback — add try/catch

### Out of scope for Step 2 (with reasons)

| Item | Reason deferred |
|---|---|
| Battery containment constraint | Battery attaches to body "bottom" at {0,0,−0.18} world → hangs *below* hub, not inside it. Body box is z∈[−0.20,+0.20]. Battery bottom reaches z=−0.32. Constraint would always be violated at baseline. Requires `CabinEnvelopeElement` (virtual) or body geometry recalibration first. |
| Payload containment constraint | Same reason — payload sphere (r=0.60 m) is below the body hub, not inside it. |
| `KeepOutZoneElement` | No placement DOFs yet for the elements that would interact with it. |
| Per-element placement variables | Design decision needed: which elements get placement DOFs and what are their physical bounds. |
| CG window constraint | CG is already in `model.mass_properties.center_of_mass`; the constraint needs a target CG parameter that has not been scoped. |

### Step 3 preview (not implemented here)

- `CabinEnvelopeElement`: virtual element (zero mass, no structure) that defines the fuselage
  interior as a box or capsule in local coordinates.
- Battery containment inside cabin envelope (battery bay).
- Payload containment inside cabin envelope.
- CG lateral offset constraint: `|cg_y| ≤ threshold` (CG is already computed, only the
  threshold parameter and constraint registration are needed).

---

## Implementation Steps

### Step 2-A: `LocalAABB` struct + `IEnvelopeProvider` capability

**File:** `include/core/ElementCapabilities.hpp`

Add after the existing capability interfaces (after `IEnergyStorage`):

```cpp
#include "eigen3/Eigen/Dense"   // add at top of file if not already present

// Axis-aligned bounding box in the element's local frame.
// min_corner / max_corner define the inner containment volume (no padding).
// Used by ArchitecturePackagingEvaluator for future containment checks.
struct LocalAABB {
    Eigen::Vector3d min_corner = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_corner = Eigen::Vector3d::Zero();

    // Returns max(0, amount by which 'inner' protrudes outside 'this').
    // Zero means inner is fully contained.
    [[nodiscard]] double containmentViolation(const LocalAABB& inner) const {
        double v = 0.0;
        for (int i = 0; i < 3; ++i) {
            v = std::max(v, inner.max_corner[i] - max_corner[i]);
            v = std::max(v, min_corner[i]       - inner.min_corner[i]);
        }
        return v;
    }
};

// Implemented by elements that define a spatial envelope (hub, fuselage, cabin, bay).
// The returned AABB is in the element's local frame (before world pose transform).
// ArchitecturePackagingEvaluator transforms it to world space via assemblyState().
class IEnvelopeProvider {
public:
    virtual ~IEnvelopeProvider() = default;
    [[nodiscard]] virtual LocalAABB localEnvelope() const = 0;
};
```

`LocalAABB::containmentViolation` must be `std::max` of six signed face penetrations. The
implementation given above is correct (it checks whether inner's max exceeds outer's max on each
axis, and whether inner's min is less than outer's min).

**Requires:** `#include <algorithm>` in the translation units that call `containmentViolation`, or
replace `std::max` with an inline ternary if the header becomes heavy. Since this is header-only
(inline method), include `<algorithm>` at the top of `ElementCapabilities.hpp`.

---

### Step 2-B: Implement `IEnvelopeProvider` on `BodyElement` and `BatteryElement`

**File:** `include/core/Elements.hpp`

Update class declarations:

```cpp
class BodyElement final : public BasicSpatialElement, public IEnvelopeProvider {
    ...
    [[nodiscard]] LocalAABB localEnvelope() const override;
    ...
};

class BatteryElement final : public BasicSpatialElement, public IEnergyStorage, public IEnvelopeProvider {
    ...
    [[nodiscard]] LocalAABB localEnvelope() const override;
    ...
};
```

**File:** `src/core/Elements.cpp`

Add `localEnvelope()` implementations. `BodyElement` exposes its inner volume (dimensions without
padding); `BatteryElement` exposes its outer volume (dimensions plus padding) so the containment
check uses the full battery footprint:

```cpp
LocalAABB BodyElement::localEnvelope() const {
    // Body inner hull: box half-sizes without padding.
    // Caller must account for the body's world pose if checking containment.
    const double half_x = std::max(kBodyHalfSizeBase, kBodyHalfSizeScale * Lx_->value + kBodyHalfSizeOffset);
    const double half_y = std::max(kBodyHalfSizeBase, kBodyHalfSizeScale * Lyi_->value + kBodyHalfSizeOffset);
    return {
        Eigen::Vector3d{-half_x, -half_y, -kBodyHalfHeight},
        Eigen::Vector3d{+half_x, +half_y, +kBodyHalfHeight}
    };
}

LocalAABB BatteryElement::localEnvelope() const {
    const double normalized_thrust = thrust_max_->value / kBaselineMotorTmax;
    const double half_x = 0.5 * (kBatteryLengthBase + kBatteryLengthScale * normalized_thrust) + kBatteryBoxPadding;
    const double half_y = 0.5 * (kBatteryWidthBase  + kBatteryWidthScale  * propeller_diameter_->value) + kBatteryBoxPadding;
    const double half_z = kBatteryHalfHeight + kBatteryBoxPadding;
    return {
        Eigen::Vector3d{-half_x, -half_y, -half_z},
        Eigen::Vector3d{+half_x, +half_y, +half_z}
    };
}
```

These reference the same `constexpr` values already in the anonymous namespace of `Elements.cpp`
(`kBodyHalfSizeBase`, `kBatteryLengthBase`, etc.) — no new constants needed.

**Note:** The implementations duplicate the geometry formula from `updateFromParameters()` by
design. They must stay in sync if the primitive geometry changes. If this becomes a maintenance
burden in Step 3, extract the half-size computation into a private helper and call it from both
`updateFromParameters()` and `localEnvelope()`.

---

### Step 2-C: `ArchitecturePackagingEvaluator` — new class

This class moves the rotor clearance check out of `VehicleScalingModel` and establishes the
extension point for future containment checks.

**New file:** `include/physics/ArchitecturePackagingEvaluator.hpp`

```cpp
#pragma once

#include "core/HexacopterArchitecture.hpp"
#include "physics/PhysicsTypes.hpp"

namespace hexaarch::physics {

// Computes all packaging metrics for the assembled architecture.
// Called from Stage1Evaluator after VehicleScalingModel::evaluate().
// Writes results into model.packaging.
class ArchitecturePackagingEvaluator {
public:
    void analyze(PhysicalModel& model,
                 const core::HexacopterArchitecture& architecture) const;
};

}  // namespace hexaarch::physics
```

**New file:** `src/physics/ArchitecturePackagingEvaluator.cpp`

Move the rotor clearance block (currently lines ~131–158 of `VehicleScalingModel.cpp`) here:

```cpp
#include "physics/ArchitecturePackagingEvaluator.hpp"

#include <limits>
#include "core/ElementCapabilities.hpp"
#include "physics/PrimitiveDistance.hpp"

namespace hexaarch::physics {

void ArchitecturePackagingEvaluator::analyze(
    PhysicalModel& model,
    const core::HexacopterArchitecture& architecture) const {

    // --- Rotor-to-rotor disk clearance ---
    // Only inter-rotor checks are physically meaningful at this fidelity.
    // Arm/body/battery geometry is placeholder — containment checks require
    // CabinEnvelopeElement or recalibrated geometry (Step 3).
    std::vector<const core::AssembledElement*> rotor_assembled;
    for (const auto& assembled : architecture.assemblyState().elements) {
        if (assembled.element != nullptr &&
            dynamic_cast<const core::IPropulsionRotor*>(assembled.element) != nullptr) {
            rotor_assembled.push_back(&assembled);
        }
    }

    double min_rotor_clearance = rotor_assembled.empty()
        ? 0.0
        : (std::numeric_limits<double>::max)();

    for (std::size_t i = 0; i < rotor_assembled.size(); ++i) {
        const auto& lhs = *rotor_assembled.at(i);
        for (std::size_t j = i + 1; j < rotor_assembled.size(); ++j) {
            const auto& rhs = *rotor_assembled.at(j);
            for (const auto& lhs_prim : lhs.local_primitives) {
                for (const auto& rhs_prim : rhs.local_primitives) {
                    const double cl = primitiveClearance(lhs_prim, lhs.world_pose,
                                                         rhs_prim, rhs.world_pose);
                    min_rotor_clearance = std::min(min_rotor_clearance, cl);
                }
            }
        }
    }

    model.packaging.minimum_rotor_clearance = min_rotor_clearance;
    model.packaging.valid = min_rotor_clearance >= 0.0;
    model.packaging.overlap_penalty = min_rotor_clearance >= 0.0
        ? 0.0
        : -min_rotor_clearance / std::max(architecture.propellerDiameter(), 1e-9);
}

}  // namespace hexaarch::physics
```

Note: `(std::numeric_limits<double>::max)()` uses the extra parentheses to prevent the `max` macro
from expanding on MSVC (per CLAUDE.md style rule).

**Register in vcxproj** (`FrameworkCpp.vcxproj`):

```xml
<ClCompile Include="src\physics\ArchitecturePackagingEvaluator.cpp" />
<ClInclude Include="include\physics\ArchitecturePackagingEvaluator.hpp" />
```

Add under the existing `StructuralNetworkAnalyzer` entries in both item groups.

---

### Step 2-D: Rename field in `PackagingReport` + expand with reserved placeholder

**File:** `include/physics/PhysicsTypes.hpp`

```cpp
struct PackagingReport {
    bool valid = true;
    double minimum_rotor_clearance = 0.0;  // renamed from minimum_clearance [m]
    double overlap_penalty = 0.0;
    // Step 3: activate once CabinEnvelopeElement is added and geometry is calibrated.
    // double battery_containment_violation = 0.0;
    // double payload_containment_violation = 0.0;
};
```

The rename propagates to two places:
- `src/physics/ArchitecturePackagingEvaluator.cpp` (new, writes the field)
- `src/evaluation/Stage1Evaluator.cpp` (reads `model.packaging.minimum_clearance` for the
  `struct_net_*` block — search for any direct packaging field reads and update)

Check with: `grep -rn "minimum_clearance" FrameworkCpp/src/`

---

### Step 2-E: Remove packaging code from `VehicleScalingModel`

**File:** `src/physics/VehicleScalingModel.cpp`

Delete the rotor clearance block (currently lines ~131–158). The block begins with the comment
`// Collect rotor elements for inter-rotor clearance` and ends at `model.packaging.overlap_penalty`.

After deletion, `VehicleScalingModel::evaluate()` no longer touches `model.packaging` at all.
`packaging` is zero-initialized by the struct default member initializers, so no residual state.

Also remove the `#include "physics/PrimitiveDistance.hpp"` from VehicleScalingModel if it is no
longer used after this removal (check for other usages of `primitiveClearance` or `boundingRadius`
in that file first).

---

### Step 2-F: Call `ArchitecturePackagingEvaluator` from `Stage1Evaluator`

**File:** `src/evaluation/Stage1Evaluator.cpp`

Add include:

```cpp
#include "physics/ArchitecturePackagingEvaluator.hpp"
```

Add the evaluator call after `VehicleScalingModel` (Phase 1 — ACS analysis) and before the
constraint loop. A clean insertion point is immediately after Phase 3 (structural network), before
the mass objective:

```cpp
// --- Phase 4 (packaging) ---
physics::ArchitecturePackagingEvaluator{}.analyze(result.physical_model, architecture);
```

This placement means packaging runs after structural (which also writes to `physical_model`) but
before constraints (which read `physical_model.packaging.*`).

---

### Step 2-G: New `Stage1Metrics` field + propagation

**File:** `include/evaluation/EvaluationResult.hpp`

In the `Stage1Metrics` struct, add after the existing `packaging` field:

```cpp
double pkg_rotor_clearance_m = 0.0;   // minimum inter-rotor disk clearance [m]; < 0 = overlap
```

**File:** `src/evaluation/Stage1Evaluator.cpp`

After the packaging evaluator call:

```cpp
result.stage1.pkg_rotor_clearance_m = result.physical_model.packaging.minimum_rotor_clearance;
```

**File:** `src/evaluation/MetricRole.cpp`

Add to the `stage1MetricDescriptors()` table (under the packaging group):

```cpp
{"pkg_rotor_clearance_m",  MetricRole::analysis_only,  "m"},
```

**File:** `src/analysis/CsvExporter.cpp`

Find the JSON object that maps `stage1` metrics and add:

```cpp
{"pkg_rotor_clearance_m", result.stage1.pkg_rotor_clearance_m},
```

Keep near the existing `"packaging"` entry for grouping.

---

### Step 2-H: Bug fix B1 — `DesignParameter::normalizedAt` scale guard

**File:** `src/core/DesignParameter.cpp`

`setFromNormalized` guards `span <= 0.0 || scale == 0.0` but `normalizedAt` only guards
`span <= 0.0`. Find `normalizedAt` and mirror the guard:

```cpp
// Before:
double DesignParameter::normalizedAt(double v) const {
    const double span = upper_bound - lower_bound;
    if (span <= 0.0) return 0.0;
    return (v - lower_bound) / span * scale;
}

// After:
double DesignParameter::normalizedAt(double v) const {
    const double span = upper_bound - lower_bound;
    if (span <= 0.0 || scale == 0.0) return 0.0;
    return (v - lower_bound) / span * scale;
}
```

---

### Step 2-I: Bug fix B2 — `Constraint::evaluate` unguarded callback

**File:** `src/core/Constraint.cpp`

The callback invocation has no exception protection. Any throw inside a constraint lambda crashes
the optimizer. Wrap:

```cpp
// Before:
ConstraintEvaluation Constraint::evaluate(const ConstraintEvaluationContext& context) const {
    ...
    const double value = callback_(context);
    ...
}

// After:
ConstraintEvaluation Constraint::evaluate(const ConstraintEvaluationContext& context) const {
    ...
    double value = 0.0;
    try {
        value = callback_(context);
    } catch (...) {
        // Treat exception as hard violation with large magnitude so the optimizer
        // moves away from this region rather than crashing.
        value = -1e6;
    }
    ...
}
```

The sign convention is important: `value = -1e6` combined with `ConstraintSense::greater_equal`
and threshold = 0 means violation. Verify that `ConstraintSense::less_equal` constraints use the
opposite convention and adjust if needed.

---

## File Checklist

| File | Change |
|---|---|
| `include/core/ElementCapabilities.hpp` | Add `LocalAABB`, `IEnvelopeProvider` |
| `include/core/Elements.hpp` | Add `IEnvelopeProvider` to `BodyElement`, `BatteryElement` |
| `src/core/Elements.cpp` | Implement `localEnvelope()` on both |
| `include/physics/ArchitecturePackagingEvaluator.hpp` | **New file** |
| `src/physics/ArchitecturePackagingEvaluator.cpp` | **New file** |
| `include/physics/PhysicsTypes.hpp` | Rename field; add reserved comment |
| `src/physics/VehicleScalingModel.cpp` | Remove rotor clearance block |
| `src/evaluation/Stage1Evaluator.cpp` | Add include + evaluator call + metric assignment |
| `include/evaluation/EvaluationResult.hpp` | Add `pkg_rotor_clearance_m` |
| `src/evaluation/MetricRole.cpp` | Add descriptor entry |
| `src/analysis/CsvExporter.cpp` | Export new metric |
| `src/core/DesignParameter.cpp` | B1: add scale guard to `normalizedAt` |
| `src/core/Constraint.cpp` | B2: wrap callback in try/catch |
| `FrameworkCpp/FrameworkCpp.vcxproj` | Register new `.cpp` and `.hpp` |

---

## Build and Verification

After implementing all steps:

1. **Build Release x64.** Expect 0 errors; the pre-existing ~377 C4819 Eigen codepage warnings
   are benign.

2. **Run `eval` mode** (single baseline evaluation):
   ```
   FrameworkCpp.exe eval
   ```
   Check JSON output for `"pkg_rotor_clearance_m"` in the `stage1` block.
   Baseline value should be positive (rotors do not overlap at baseline arm lengths).

3. **Run `soo` mode** to confirm the optimizer still converges:
   ```
   FrameworkCpp.exe soo
   ```
   Baseline J and best-feasible J should be consistent with previous run (J≈0.59 baseline,
   J≈0.52 best-feasible). The packaging refactor is functionally equivalent — no new constraints
   are active.

4. **Confirm `minimum_clearance` field is gone.** `grep -rn "minimum_clearance" FrameworkCpp/src/`
   should return no matches after the rename.

---

## CODEBASE.md Updates Required

After completing Step 2, add the following to `CODEBASE.md`:

- Under **Directory Layout**: add `ArchitecturePackagingEvaluator.hpp/.cpp` alongside the other
  physics analyzers.
- Under **Invariants**: add note 15 — "`IEnvelopeProvider` is NOT connected to containment
  constraints yet. Battery hangs below the body hub (z≈−0.18 world), so battery containment inside
  body box would violate at baseline. Step 3 adds `CabinEnvelopeElement` before activating
  containment constraints."
- Update the `PackagingReport` description to reflect `minimum_rotor_clearance` field name.

---

## Step 3 Specification (preview — not implemented here)

Step 3 establishes the first true containment constraint by adding a virtual envelope element.

### `CabinEnvelopeElement`

A zero-mass, zero-structure element that declares the fuselage/cabin inner volume.
Implements `IEnvelopeProvider`. Not `IStructuralMember`, not `IMotorMassContributor`.

Construction parameters:
- `half_x`, `half_y`, `half_z` — inner hull half-sizes, either fixed or driven by a new design
  parameter (e.g., `cabin_half_height` for vertical cabin sizing).

Attachment: to body "bottom" anchor (same parent as battery, displacing downward with a configurable
z offset so the cabin volume brackets both battery and payload).

Example local envelope at baseline:
- half_x = 0.50 m, half_y = 0.50 m, half_z = 0.60 m (brackets battery height 0.14 m + payload
  sphere radius 0.60 m with some margin)
- Cabin bottom at z ≈ −0.20 − 0.60 = −0.80 world (below body, around battery and payload)

### Containment constraints (Step 3)

Once `CabinEnvelopeElement` exists and its AABB is calibrated to enclose battery and payload at
baseline:

- `packaging::battery_in_cabin`: `LocalAABB::containmentViolation(cabin_world_aabb, battery_world_aabb) ≤ 0`
  Hard, penalty 1000.
- `packaging::payload_in_cabin`: `LocalAABB::containmentViolation(cabin_world_aabb, payload_world_aabb) ≤ 0`
  Hard, penalty 1000.

Both read from `model.packaging.battery_containment_violation` and
`model.packaging.payload_containment_violation` (the reserved fields from Step 2-D).

### World-space AABB transform

For axis-aligned elements with no rotation (body and cabin both have identity rotation in the
assembly), the world-space AABB is simply the local AABB shifted by the element's world translation:

```cpp
LocalAABB worldAABB(const LocalAABB& local, const Eigen::Isometry3d& world_pose) {
    const Eigen::Vector3d t = world_pose.translation();
    return { local.min_corner + t, local.max_corner + t };
}
```

This assumes identity rotation. If the element has a non-trivial rotation, the world-space AABB
must be computed by transforming all 8 corners and taking the min/max. For Step 3, the identity
assumption is valid for body and cabin.
