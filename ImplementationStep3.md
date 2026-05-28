# Implementation Step 3 — First Real Placement Constraints

## Status entering Step 3

Step 2 established the packaging infrastructure:
- `LocalAABB` + `IEnvelopeProvider` capability
- `BodyElement` and `BatteryElement` implement `localEnvelope()`
- `ArchitecturePackagingEvaluator` owns all packaging analysis
- `PackagingReport::minimum_rotor_clearance` + `pkg_rotor_clearance_m` exported
- Reserved fields `battery_containment_violation` / `payload_containment_violation` exist in `PackagingReport` but are commented out

Step 3 activates the first real containment constraint.

---

## Step 3 Scope

**Primary goal:** Add `CabinEnvelopeElement` (virtual, zero mass) and wire the first real
containment constraint: battery AABB must lie inside the cabin inner hull.

**Secondary goal:** Replace the `PayloadElement` sphere primitive with a box primitive
(same mass, better packaging geometry for future containment).

**Tertiary goal:** Export CG lateral offset as an analysis metric (uses already-computed
`model.mass_properties.center_of_mass`; foundation for a future CG window constraint).

### In scope

1. `CabinEnvelopeElement` — zero-mass virtual element, fixed inner hull, `IEnvelopeProvider`
2. Battery containment constraint `packaging::battery_in_cabin` — first real containment, hard, penalty 1000
3. Activate `PackagingReport::battery_containment_violation` field (remove comment from Step 2)
4. New Stage1Metrics field `pkg_battery_containment_m` and `cg_y_offset_m`
5. Replace sphere primitive on `PayloadElement` with a box (same mass, physical cabin footprint)

### Out of scope for Step 3 (with reasons)

| Item | Reason deferred |
|---|---|
| Payload containment in cabin | `PayloadElement` sphere (r=0.60m) extends above body hub (z=+0.35 world at baseline). Geometry replacement in Step 3 gives the right primitive; calibrated containment constraint follows in Step 4 after verifying dimensions. |
| `KeepOutZoneElement` | Rotor disks are 3–6 m from all central elements. Constraint trivially inactive until per-element placement DOFs exist. Infrastructure deferred to Step 4. |
| CG window hard constraint | `cg_y_offset_m` is always ~0 in symmetric design — no optimization gradient until battery or payload placement DOFs are added (Step 4). |
| Per-element placement DOFs | Design decision on which elements get DOFs; requires new parameters, DesignVectorMapper changes, bounds calibration. Step 4. |
| Passenger / avionics elements | Require splitting frame mass allometric model — calibration break risk. Step 4+. |

---

## Geometry Calibration (derive before coding)

### Battery geometry at T_max bounds (d_prop = 0.40 m frozen)

| T_max [N] | half_x [m] | half_y [m] | half_z [m] | world z∈ |
|---|---|---|---|---|
| 8 000 | 0.239 | 0.165 | 0.140 | [−0.32, −0.04] |
| 12 000 | 0.261 | 0.165 | 0.140 | [−0.32, −0.04] |
| 16 000 | 0.282 | 0.165 | 0.140 | [−0.32, −0.04] |
| 20 000 | 0.304 | 0.165 | 0.140 | [−0.32, −0.04] |

Battery center is always at world (0, 0, −0.18) — body "bottom" (z=−0.20) + attachment offset (+0.02).

### Cabin inner hull target

The cabin must:
1. Contain battery at all T_max values in the feasible region (T_max ≤ ~15 kN after ACS constraint),
   with at least 5 mm margin at baseline.
2. Be violated at T_max near the upper bound (creates optimization gradient).

Chosen dimensions (local frame of `CabinEnvelopeElement`):

```
kCabinHalfX = 0.29 m   →  baseline margin 29 mm; violated at T_max ≈ 16 700 N
kCabinHalfY = 0.20 m   →  fixed (d_prop frozen, battery y = 0.165 fixed)
kCabinHalfZ = 0.20 m   →  battery z ∈ [−0.14, +0.14], cabin z ∈ [−0.20, +0.20] — always clear
```

Cabin attachment: body "bottom" (world z=−0.20) + offset {0, 0, +0.02} → **cabin center at world (0, 0, −0.18)**.

Cabin inner AABB world: x∈[−0.29, +0.29], y∈[−0.20, +0.20], z∈[−0.38, +0.02].

Containment at baseline: battery (x±0.261, y±0.165, z∈[−0.32,−0.04]) ⊆ cabin ✓.

Containment at T_max=20 000: battery x half=0.304 > cabin half=0.29 → **violated by 14 mm**.

World-space AABB transform (valid for identity-rotation elements):
```cpp
LocalAABB worldAABB(const LocalAABB& local, const Eigen::Isometry3d& world_pose) {
    const Eigen::Vector3d t = world_pose.translation();
    return { local.min_corner + t, local.max_corner + t };
}
```
Both body and cabin have identity rotation in the assembly (pure translation attachment), so this is exact.

---

## Implementation Steps

### Step 3-A: Activate `battery_containment_violation` in `PackagingReport`

**File:** `include/physics/PhysicsTypes.hpp`

Uncomment the reserved field (remove the comment):

```cpp
struct PackagingReport {
    bool valid = true;
    double minimum_rotor_clearance = 0.0;
    double overlap_penalty = 0.0;
    double battery_containment_violation = 0.0;  // > 0 = battery protrudes outside cabin [m]
    // double payload_containment_violation = 0.0;  // Step 4
};
```

---

### Step 3-B: `CabinEnvelopeElement` — new element

**New file:** `include/core/Elements.hpp` — add the class declaration alongside the existing element classes:

```cpp
// Zero-mass virtual element that defines the battery compartment inner hull.
// Implements IEnvelopeProvider. Not structural, not a mass contributor.
// Attached to body "bottom" + {0, 0, +0.02} so its center aligns with the battery center.
class CabinEnvelopeElement final : public BasicSpatialElement, public IEnvelopeProvider {
public:
    explicit CabinEnvelopeElement(std::string id);
    [[nodiscard]] std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;
    [[nodiscard]] LocalAABB localEnvelope() const override;
};
```

`CabinEnvelopeElement` has no design parameters — its hull is fixed (regulatory/design requirement).
`registerParameters()` and `registerConstraints()` are empty no-ops (same as existing stub pattern).

**Implementation in `src/core/Elements.cpp`:**

Add geometry constants to the anonymous namespace:

```cpp
// CabinEnvelopeElement — battery compartment inner hull [m]
constexpr double kCabinHalfX = 0.29;
constexpr double kCabinHalfY = 0.20;
constexpr double kCabinHalfZ = 0.20;
```

Implement the class:

```cpp
CabinEnvelopeElement::CabinEnvelopeElement(std::string id)
    : BasicSpatialElement(std::move(id), "CabinEnvelopeElement") {}

std::unique_ptr<SpatialElement> CabinEnvelopeElement::clone() const {
    return std::make_unique<CabinEnvelopeElement>(*this);
}

void CabinEnvelopeElement::registerParameters(ParameterRegistry&) {}
void CabinEnvelopeElement::rebindParameters(ParameterRegistry&) {}
void CabinEnvelopeElement::registerConstraints(ConstraintRegistry&) const {}

void CabinEnvelopeElement::updateFromParameters() {
    mass_ = 0.0;
    local_com_.setZero();
    local_inertia_.setZero();
    primitives_ = {GeometryPrimitive::makeBox({kCabinHalfX, kCabinHalfY, kCabinHalfZ}, 0.0)};
    anchors_.clear();
    setAnchor("center", Eigen::Isometry3d::Identity());
}

LocalAABB CabinEnvelopeElement::localEnvelope() const {
    return {
        Eigen::Vector3d{-kCabinHalfX, -kCabinHalfY, -kCabinHalfZ},
        Eigen::Vector3d{+kCabinHalfX, +kCabinHalfY, +kCabinHalfZ}
    };
}
```

---

### Step 3-C: Add `CabinEnvelopeElement` to the default assembly

**File:** `src/core/DefaultHexacopterBuilder.cpp`

In `buildElements()`, add after the battery element:

```cpp
elements.push_back(std::make_unique<CabinEnvelopeElement>("cabin_envelope"));
```

In `buildAttachments()`, add after the battery attachment:

```cpp
// Cabin envelope co-located with battery (same offset from body bottom).
// Identity rotation — cabin center at world (0, 0, −0.18).
attachments.push_back({
    "body", "cabin_envelope", "bottom", "center",
    AttachmentRelationship::localOffset({0.0, 0.0, 0.02}),
    true, "central",
    AttachmentContactPolicy::bonded_overlap, nullptr
});
```

The cabin element contributes zero mass and has no design variable, so no changes to
`DefaultHexacopterParameters`, `HexacopterArchitecture`, or `HexacopterArchitecture.cpp`
parameter bindings are needed.

---

### Step 3-D: Battery containment check in `ArchitecturePackagingEvaluator`

**File:** `src/physics/ArchitecturePackagingEvaluator.cpp`

Add a `worldAABB()` free function in the anonymous namespace:

```cpp
namespace {
LocalAABB worldAABB(const core::LocalAABB& local, const Eigen::Isometry3d& world_pose) {
    // Valid for identity-rotation elements (pure translation in assembly).
    const Eigen::Vector3d t = world_pose.translation();
    return {local.min_corner + t, local.max_corner + t};
}
}  // namespace
```

Add the containment check at the end of `analyze()`:

```cpp
// --- Battery containment inside cabin envelope ---
// Find cabin and battery in assembly. The cabin element has type() == "CabinEnvelopeElement".
const core::AssembledElement* cabin_el  = nullptr;
const core::AssembledElement* battery_el = nullptr;
for (const auto& assembled : architecture.assemblyState().elements) {
    if (assembled.element == nullptr) continue;
    if (assembled.element->type() == "CabinEnvelopeElement") cabin_el  = &assembled;
    if (dynamic_cast<const core::IEnergyStorage*>(assembled.element) != nullptr) battery_el = &assembled;
}

if (cabin_el != nullptr && battery_el != nullptr) {
    const auto* cabin_prov   = dynamic_cast<const core::IEnvelopeProvider*>(cabin_el->element);
    const auto* battery_prov = dynamic_cast<const core::IEnvelopeProvider*>(battery_el->element);
    if (cabin_prov != nullptr && battery_prov != nullptr) {
        const LocalAABB cabin_world   = worldAABB(cabin_prov->localEnvelope(),   cabin_el->world_pose);
        const LocalAABB battery_world = worldAABB(battery_prov->localEnvelope(), battery_el->world_pose);
        model.packaging.battery_containment_violation = cabin_world.containmentViolation(battery_world);
    }
}
```

Add the required include at the top:

```cpp
#include "core/Elements.hpp"   // for IEnergyStorage
```

Wait — `IEnergyStorage` is declared in `ElementCapabilities.hpp`, not `Elements.hpp`.
Add: `#include "core/ElementCapabilities.hpp"` (already included, check) — yes, it's already there.

Actually `IEnergyStorage` is in `ElementCapabilities.hpp`, which is already included. The
`type()` check for "CabinEnvelopeElement" uses the string set by the `BasicSpatialElement`
constructor — no additional include is needed.

---

### Step 3-E: Register `packaging::battery_in_cabin` hard constraint

**File:** `src/core/HexacopterArchitecture.cpp`

In `registerDefaultConstraints()`, add after the existing `rotor_clearance` constraint:

```cpp
constraints_.add({
    "packaging::battery_in_cabin",
    id_,
    ConstraintSense::less_equal,
    0.0,
    true,
    true,
    1000.0,
    [](const ConstraintEvaluationContext& context) {
        Constraint c{"packaging::battery_in_cabin",
                     context.architecture.id(),
                     ConstraintSense::less_equal, 0.0};
        return c.evaluate(context.physical_model.packaging.battery_containment_violation);
    }
});
```

`ConstraintSense::less_equal` with threshold 0.0 means:
- `violation = max(0, value - 0) = max(0, battery_containment_violation)` → feasible when ≤ 0.

---

### Step 3-F: New `Stage1Metrics` fields

**File:** `include/evaluation/EvaluationResult.hpp`

In the packaging section:

```cpp
// Phase 6 — Packaging (Step 2)
double pkg_rotor_clearance_m = 0.0;

// Phase 7 — Placement constraints (Step 3)
double pkg_battery_containment_m = 0.0;  // > 0 = battery protrudes outside cabin [m]
double cg_y_offset_m = 0.0;              // lateral CG offset from geometric center [m]
```

**File:** `src/evaluation/Stage1Evaluator.cpp`

After the packaging evaluator call:

```cpp
physics::ArchitecturePackagingEvaluator{}.analyze(result.physical_model, architecture);
result.stage1.pkg_rotor_clearance_m       = result.physical_model.packaging.minimum_rotor_clearance;
result.stage1.pkg_battery_containment_m   = result.physical_model.packaging.battery_containment_violation;
result.stage1.cg_y_offset_m              = result.physical_model.mass_properties.center_of_mass.y();
```

---

### Step 3-G: MetricRole descriptors

**File:** `src/evaluation/MetricRole.cpp`

In the packaging group:

```cpp
// --- Packaging (Phase 6/7) ---
{"pkg_rotor_clearance_m",              MetricRole::analysis_only,    "m"},
{"pkg_battery_containment_m",          MetricRole::hard_constraint,  "m"},
{"cg_y_offset_m",                      MetricRole::analysis_only,    "m"},
```

`pkg_battery_containment_m` is `hard_constraint` because `packaging::battery_in_cabin` is hard.
`cg_y_offset_m` is `analysis_only` — the CG window hard constraint is Step 4.

---

### Step 3-H: CsvExporter export

**File:** `src/analysis/CsvExporter.cpp`

In `stage1ToJson()`, add after `pkg_rotor_clearance_m`:

```cpp
{"pkg_battery_containment_m", m.pkg_battery_containment_m},
{"cg_y_offset_m",             m.cg_y_offset_m},
```

---

### Step 3-I: Replace `PayloadElement` sphere with box

**File:** `src/core/Elements.cpp`

In `PayloadElement::updateFromParameters()`, replace the sphere primitive with a box
representing the cabin footprint:

```cpp
// Before:
primitives_ = {GeometryPrimitive::makeSphere(kBaselinePayloadRadius)};

// After:
// Box replaces sphere: same mass model, better packaging primitive.
// Dimensions represent a 4-passenger cabin floor plan (1.2 × 1.2 × 1.0 m usable).
// kBaselinePayloadRadius constant is kept for other consumers; the primitive changes only.
constexpr double kPayloadBoxHalfX = 0.60;
constexpr double kPayloadBoxHalfY = 0.60;
constexpr double kPayloadBoxHalfZ = 0.50;
primitives_ = {GeometryPrimitive::makeBox({kPayloadBoxHalfX, kPayloadBoxHalfY, kPayloadBoxHalfZ}, 0.0)};
```

This changes the 3D visual representation but not the mass model. The `packaging.overlap_penalty`
(weight=0) is the only consumer of payload primitive geometry; it is not an active objective.

Note: the new box (x∈[−0.60, +0.60], z∈[−0.50, +0.50] in payload local frame) placed at
world z≈−0.25 gives payload world z∈[−0.75, +0.25]. The cabin envelope (z∈[−0.38, +0.02])
does NOT yet contain the payload box. **Payload containment is Step 4**, after the cabin
envelope is extended vertically (or a separate lower cabin section is added).

---

### Step 3-J: Register new files in `FrameworkCpp.vcxproj`

`CabinEnvelopeElement` is declared in `Elements.hpp` (already registered) and implemented in
`Elements.cpp` (already registered). No new file registrations needed for Step 3.

---

## File Checklist

| File | Change |
|---|---|
| `include/physics/PhysicsTypes.hpp` | Uncomment `battery_containment_violation` in `PackagingReport` |
| `include/core/Elements.hpp` | Add `CabinEnvelopeElement` class declaration |
| `src/core/Elements.cpp` | Add `kCabin*` constants + `CabinEnvelopeElement` implementation; replace `PayloadElement` sphere with box |
| `src/core/DefaultHexacopterBuilder.cpp` | Add cabin element + attachment |
| `src/physics/ArchitecturePackagingEvaluator.cpp` | Add `worldAABB()` helper + battery containment check |
| `src/core/HexacopterArchitecture.cpp` | Register `packaging::battery_in_cabin` hard constraint |
| `include/evaluation/EvaluationResult.hpp` | Add `pkg_battery_containment_m`, `cg_y_offset_m` |
| `src/evaluation/Stage1Evaluator.cpp` | Assign new Stage1Metrics fields |
| `src/evaluation/MetricRole.cpp` | Add descriptors for new metrics |
| `src/analysis/CsvExporter.cpp` | Export new metrics in `stage1ToJson()` |

No new `.cpp`/`.hpp` files — `CabinEnvelopeElement` lives in the existing `Elements.hpp/.cpp`.
No vcxproj changes needed.

---

## Build and Verification

1. **Build Release x64.** Expect 0 errors.

2. **Run `eval` mode:**
   ```
   FrameworkCpp.exe eval
   ```
   In JSON output, check:
   - `pkg_battery_containment_m` ≈ 0.0 at baseline (battery just fits in cabin with ~29 mm margin)
   - `cg_y_offset_m` ≈ 0.0 (symmetric design)
   - Constraint `packaging::battery_in_cabin` feasible at baseline

3. **Verify constraint activates at high T_max:**
   Manually set T_max=20000 in a custom eval and confirm `pkg_battery_containment_m` > 0.

4. **Run `soo` mode:**
   ```
   FrameworkCpp.exe soo
   ```
   The new constraint gives the optimizer a signal against large T_max values (which also conflict
   with ACS feasibility). Expect modest change in best-feasible J (< 0.02). Confirm optimizer
   still finds a feasible solution and baseline J is consistent.

5. **Confirm `cabin_envelope` appears in assembly:**
   The `eval` JSON should show a `cabin_envelope` element in the physical model elements list.

---

## CODEBASE.md Updates Required

After completing Step 3:

- Under **Directory Layout** `Elements.hpp`: add `CabinEnvelopeElement` to the element list.
- Add invariant 18: `CabinEnvelopeElement` calibration — cabin inner hull (0.29 × 0.20 × 0.20 m,
  centered at world z=−0.18), battery constraint active at T_max ≈ 16 700 N. Payload containment
  deferred to Step 4 (cabin z-extent too short to contain payload box at baseline).
- Update invariant 12: note that `BodyElement` and `BatteryElement` implement `IEnvelopeProvider`,
  and `CabinEnvelopeElement` is the first element whose envelope is used in an active constraint.

---

## Step 4 Preview

Step 4 activates placement DOFs and makes the remaining constraints non-trivial.

### Battery z-offset placement DOF

Add `z_bat_offset` as an active design parameter (bounds [−0.10, +0.10] m, default 0.0).
`BatteryElement::updateFromParameters()` shifts `local_com_.z()` and updates the attachment
offset. This makes CG deviate from zero → CG lateral/vertical constraint becomes active.

### Cabin vertical extension

Extend `CabinEnvelopeElement` inner hull to z∈[−0.50, +0.10] (add lower cabin section).
Activate `packaging::payload_in_cabin` hard constraint.

### `KeepOutZoneElement`

Add one per rotor (type "RotorDiskZoneElement"), r = d_prop/2 + 0.05 m. Constraint: all
non-rotor elements' bounding sphere centers must be at distance ≥ r from each rotor axis.
Set `active=false` (analysis only) until battery z-offset placement DOF makes it non-trivial.

### CG window hard constraint

Activate `cg_y_offset_m` as a hard constraint with threshold ≤ 0.05 m (5 cm lateral CG
tolerance for ACS hover trim margin).
