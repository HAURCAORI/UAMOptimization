# FrameworkCpp — Spatial Element Extension Manual

This manual explains how to implement a new `SpatialElement` and integrate it into `HexacopterArchitecture`. It covers the required interface, available geometry primitives, capability mix-ins, anchor frame usage, and the attachment system.

---

## 1. The SpatialElement Interface

Every element in the scene graph derives from `SpatialElement` (`include/core/SpatialElement.hpp`). The contract is:

```cpp
class SpatialElement {
public:
    virtual std::string id() const = 0;           // unique string identifier
    virtual std::string type() const = 0;          // human-readable class name
    virtual std::unique_ptr<SpatialElement> clone() const = 0; // deep copy

    // Parameter lifecycle
    virtual void registerParameters(ParameterRegistry&) = 0;
    virtual void rebindParameters(ParameterRegistry&) = 0;
    virtual void registerConstraints(ConstraintRegistry&) const = 0;

    // Physical properties (updated by updateFromParameters)
    virtual double mass() const = 0;
    virtual Eigen::Vector3d localCOM() const = 0;
    virtual Eigen::Matrix3d localInertiaAtLocalCOM() const = 0;

    // Geometry and pose
    virtual GeometryPrimitives localPrimitives() const = 0;
    virtual Eigen::Isometry3d localPose() const = 0;
    virtual std::vector<AnchorFrame> anchors() const = 0;
    virtual std::optional<Eigen::Isometry3d> anchorPose(std::string_view) const = 0;

    // Called whenever any parameter changes
    virtual void updateFromParameters() = 0;
};
```

`BasicSpatialElement` (`include/core/Elements.hpp`) provides default storage for all properties and implements everything except `registerParameters`, `rebindParameters`, `registerConstraints`, `clone`, and `updateFromParameters`. Derive from it to avoid boilerplate.

---

## 2. Capability Mix-in Interfaces

Declare these on your class to opt into framework subsystems.

### 2.1 `IPropulsionRotor`

Marks an element as a rotor that contributes to the allocation matrix and packaging clearance check.

```cpp
class IPropulsionRotor {
public:
    virtual int rotorIndex() const = 0;     // 0-based index into B columns
    virtual double yawMomentSign() const = 0; // +1 or -1
};
```

`VehicleScalingModel` queries this interface to build the $4\times 6$ allocation matrix. The rotor's world-frame position (from the assembly) determines the thrust and pitch/roll rows. `yawMomentSign() * cT` sets the yaw row.

**When to use:** any rotating disk that generates thrust and a reaction torque.

### 2.2 `IStructuralMember`

Marks an element whose length contributes to the total arm span (used in the bending index).

```cpp
class IStructuralMember {
public:
    virtual double structuralSpanContribution() const = 0; // [m]
    virtual bool contributesToFrameMass() const = 0;
};
```

`structuralSpanContribution()` should return the physical span of the member (e.g. the length of the first segment primitive). When `contributesToFrameMass()` returns `true`, the element's mass is summed into `PhysicalModel::frame_mass`.

**When to use:** beams, booms, or any load-bearing member whose length drives structural sizing.

### 2.3 `IMotorMassContributor`

Tags an element as a motor-class mass whose world position is used to correct the body inertia tensor.

```cpp
class IMotorMassContributor {
    // tag only — no methods
};
```

`VehicleScalingModel` subtracts the inertia contribution of each tagged element from the baseline body inertia ($I_{x,0}$, $I_{y,0}$) and adds the geometrically computed value.

**When to use:** motor, ESC, or any concentrated mass at the rotor mount.

### 2.4 `IPayloadMassContributor`

Tags an element whose `mass()` is recorded as the payload mass in `PhysicalModel`. Only one element should carry this tag per architecture.

```cpp
class IPayloadMassContributor {
    // tag only — no methods
};
```

**When to use:** cargo container, passenger pod, or any mission payload.

---

## 3. Geometry Primitives

Primitives define the element's bounding geometry for clearance checking and visualisation. They are expressed in the element's local frame.

All primitives support an optional `padding` [m] that expands the effective boundary.

### 3.1 `makeSphere(radius, padding = 0)`

A sphere centred at the primitive's local pose origin.

```cpp
primitives_ = {GeometryPrimitive::makeSphere(0.60)};
```

**Use for:** roughly isotropic objects (payload pods, sensor balls).

### 3.2 `makeBox(extents, padding = 0)`

An axis-aligned box. `extents` is the **half-extent** vector $(h_x, h_y, h_z)$.

```cpp
primitives_ = {GeometryPrimitive::makeBox({0.35, 0.25, 0.12}, 0.02)};
```

**Use for:** batteries, fuselage sections, avionics bays.

### 3.3 `makeCylinder(radius, length, padding = 0)`

A cylinder. The axis is along the primitive's local x-direction by default; rotate the `local_pose` to change the orientation.

```cpp
GeometryPrimitive cyl = GeometryPrimitive::makeCylinder(0.10, 0.15, 0.01);
cyl.local_pose.linear() =
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
primitives_ = {cyl};
```

**Use for:** motor cans, tanks, tubes.

### 3.4 `makeDisk(radius, padding = 0)`

A flat disk (zero thickness). The disk lies in the primitive's local xy-plane.

```cpp
primitives_ = {GeometryPrimitive::makeDisk(0.5 * d_prop_, 0.01)};
```

**Use for:** rotor disks; the clearance algorithm computes disk-to-disk separation using this primitive.

### 3.5 `makeSegment(length, padding = 0)`

A line segment along the primitive's local x-axis from the origin to $(length, 0, 0)$.

```cpp
primitives_ = {GeometryPrimitive::makeSegment(arm_length, 0.05)};
```

**Use for:** arms, spars, cables. The segment length is what `IStructuralMember::structuralSpanContribution()` should return.

---

## 4. Anchor Frames

An `AnchorFrame` is a named local pose used by the attachment system. Declare as many as needed; the attachment specifies which parent anchor and child anchor to align.

```cpp
// Convention: anchor at the element's origin
setAnchor("center", Eigen::Isometry3d::Identity());

// Anchor at the end of an arm
Eigen::Isometry3d tip = Eigen::Isometry3d::Identity();
tip.translation() = Eigen::Vector3d(arm_length, 0.0, 0.0);
setAnchor("tip", tip);
```

Anchors must be set (or re-set) inside `updateFromParameters()` because their positions may depend on parameter values.

---

## 5. Attachment Contact Policies

When two elements are attached, the contact policy controls whether their geometry is exempted from clearance checking.

| Policy | Effect |
|---|---|
| `enforce_clearance` | Primitives of parent and child must not overlap. |
| `allow_touch` | Primitives may touch (clearance ≥ 0) but not overlap. |
| `bonded_overlap` | Primitives may freely overlap. Propagates up the chain: all ancestors with `bonded_overlap` are mutually exempted from clearance checking. |

Use `bonded_overlap` for elements that are bolted together (arm-to-motor, motor-to-rotor). Use `enforce_clearance` between independently positioned elements (rotor-to-rotor).

---

## 6. Attachment Relationship Kinds

The `AttachmentRelationship` struct encodes how the child's anchor is positioned relative to the parent's anchor.

| Kind | Factory | Description |
|---|---|---|
| `rigid_mount` | `AttachmentRelationship::rigidMount()` | Child anchor coincides with parent anchor. No additional transform. |
| `local_offset` | `AttachmentRelationship::localOffset({dx, dy, dz})` | Translate child anchor by a fixed offset in the parent's local frame. |
| `mirrored_local_offset` | `AttachmentRelationship::mirroredLocalOffset(offset, signs)` | As above but each component is multiplied by the corresponding sign ($\pm 1$). Used for symmetric pairs (e.g. left/right arms sharing one template attachment). |

A fully custom transform can also be supplied as a `std::function<Eigen::Isometry3d(const HexacopterArchitecture&)>` in the attachment's `relative_transform` field. This is how the arm mounts are computed from the parameter-dependent motor positions.

---

## 7. Step-by-Step: Adding a New Element

### Step 1 — Declare the class

Create a header in `include/core/` or your own include directory.

```cpp
#include "core/Elements.hpp"

namespace hexaarch::core {

class WingletElement final : public BasicSpatialElement, public IStructuralMember {
public:
    WingletElement(std::string id, DesignParameter* span);

    std::unique_ptr<SpatialElement> clone() const override;
    void registerParameters(ParameterRegistry& registry) override;
    void rebindParameters(ParameterRegistry& registry) override;
    void registerConstraints(ConstraintRegistry& registry) const override;
    void updateFromParameters() override;

    // IStructuralMember
    double structuralSpanContribution() const override;
    bool contributesToFrameMass() const override { return false; }

private:
    DesignParameter* span_;
};

} // namespace hexaarch::core
```

### Step 2 — Implement the source

```cpp
#include "core/WingletElement.hpp"

namespace hexaarch::core {

WingletElement::WingletElement(std::string id, DesignParameter* span)
    : BasicSpatialElement(std::move(id), "WingletElement"),
      span_(span) {}

std::unique_ptr<SpatialElement> WingletElement::clone() const {
    return std::make_unique<WingletElement>(*this);
}

void WingletElement::registerParameters(ParameterRegistry&) {
    span_->addConsumer(id_);
}

void WingletElement::rebindParameters(ParameterRegistry& registry) {
    span_ = registry.find(span_->stable_id());
    if (!span_) throw std::invalid_argument("Missing parameter: " + span_->stable_id());
}

void WingletElement::registerConstraints(ConstraintRegistry& registry) const {
    const std::string pid = span_->stable_id();
    registry.add({
        "winglet_span_positive", id_,
        ConstraintSense::greater_equal, 0.0,
        true, true, 500.0,
        [pid](const ConstraintEvaluationContext& ctx) {
            const auto& p = *ctx.architecture.parameters().find(pid);
            Constraint c{"winglet_span_positive", "winglet", ConstraintSense::greater_equal, 0.0};
            return c.evaluate(p.value);
        }
    });
}

void WingletElement::updateFromParameters() {
    const double half_span = 0.5 * span_->value;
    mass_ = 3.5 * span_->value;   // 3.5 kg/m linear density
    local_com_ = Eigen::Vector3d(half_span, 0.0, 0.0);
    local_inertia_.setZero();
    local_inertia_(1, 1) = mass_ * span_->value * span_->value / 12.0;
    local_inertia_(2, 2) = local_inertia_(1, 1);
    primitives_ = {GeometryPrimitive::makeSegment(span_->value, 0.04)};
    anchors_.clear();
    setAnchor("root", Eigen::Isometry3d::Identity());
    Eigen::Isometry3d tip = Eigen::Isometry3d::Identity();
    tip.translation() = Eigen::Vector3d(span_->value, 0.0, 0.0);
    setAnchor("tip", tip);
}

double WingletElement::structuralSpanContribution() const {
    return primitives_.empty() ? 0.0 : primitives_.front().dimensions.x();
}

} // namespace hexaarch::core
```

### Step 3 — Register a new parameter (if needed)

In `HexacopterArchitecture::registerDefaultParameters()` (or before calling `addElement`):

```cpp
parameters_.add({
    "b_winglet",          // name
    id_,                  // owner
    "m",                  // unit
    "Winglet span",       // description
    1.20,                 // default value
    0.50, 3.00,           // [lower, upper] bounds
    1.20,                 // default_value (repeated)
    true,                 // active (included in design vector)
    1.0                   // scale
});
```

### Step 4 — Add the element to the architecture

If extending `DefaultHexacopterBuilder`, add the instantiation to `buildElements()`:

```cpp
auto* b_winglet = parameters_.find(id_ + "::b_winglet");
elements.push_back(std::make_unique<WingletElement>("winglet_left", b_winglet));
```

If adding at runtime to an existing architecture object:

```cpp
HexacopterArchitecture arch;
arch.parameters().add({...});        // register parameter first
auto element = std::make_unique<WingletElement>("winglet_left", arch.parameters().find("..::b_winglet"));
arch.addElement(std::move(element)); // registers, updates, rebinds automatically
```

### Step 5 — Define the attachment

```cpp
Eigen::Isometry3d winglet_pose = Eigen::Isometry3d::Identity();
winglet_pose.linear() = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();

arch.addAttachment({
    "body",          // parent element id
    "winglet_left",  // child element id
    "top",           // parent anchor
    "root",          // child anchor
    AttachmentRelationship::rigidMount(),
    true,            // enabled
    "",              // symmetry_tag (set for mirrored pairs)
    AttachmentContactPolicy::bonded_overlap,
    [winglet_pose](const HexacopterArchitecture&) { return winglet_pose; }
});
arch.rebuildAssembly();
```

### Step 6 — Verify

```cpp
Stage1Evaluator evaluator;
EvaluationContext ctx;
auto result = evaluator.evaluate(arch, ctx);

std::cout << "Feasible: " << result.feasible << "\n";
std::cout << "Mass: " << result.stage1.mass << "\n";
```

---

## 8. Available Spatial Element Types (Built-In)

| Class | Type string | Key parameters | Capabilities |
|---|---|---|---|
| `BodyElement` | `"BodyElement"` | $L_x$, $L_{yi}$, $L_{yo}$ | — |
| `ArmElement` | `"ArmElement"` | $L_x$, $L_{yi}$, $L_{yo}$, $T_{max}$ | `IStructuralMember` |
| `MotorElement` | `"MotorElement"` | $T_{max}$ | `IMotorMassContributor` |
| `RotorElement` | `"RotorElement"` | $d_{prop}$ | `IPropulsionRotor` |
| `BatteryElement` | `"BatteryElement"` | $T_{max}$, $d_{prop}$ | — |
| `PayloadElement` | `"PayloadElement"` | $m_{pay}$ | `IPayloadMassContributor` |

---

## 9. Constraint Registration Guidelines

- Call `registry.add(...)` in `registerConstraints()`, not in `updateFromParameters()`.
- Capture only the parameter **stable ID** (`parameter->stable_id()`) in the lambda — never a raw pointer, because elements are cloned and rebinding changes pointer addresses.
- Inside the lambda, re-fetch the parameter via `requireParameter(context.architecture.parameters(), stable_id)`.
- Set `hard = true` for geometric and physical limits that make the design infeasible.
- Set `active = true` so the constraint is evaluated; set to `false` to disable temporarily.
- Choose a penalty in proportion to the severity: 500–1000 for auxiliary checks, 1500–2000 for safety-critical bounds.

---

## 10. Parameter Naming Conventions

| Field | Convention | Example |
|---|---|---|
| `name` | short snake_case | `"b_winglet"` |
| Stable ID | `"<arch-id>::<name>"` | `"default-architecture::b_winglet"` |
| `unit` | SI symbol string | `"m"`, `"N"`, `"kg"`, `"-"` |
| `description` | sentence fragment | `"Winglet semi-span"` |
| `scale` | keep at 1.0 | 1.0 |

The stable ID is used by the optimizer (`DesignVectorMapper`) and by constraint lambdas to locate parameters after cloning. Always derive it via `parameter->stable_id()` rather than constructing the string manually.

---

## 11. Assembly Validation

After calling `rebuildAssembly()` or `addElement()`, the assembly checks for:

- **Orphaned elements** — every element must have an attachment path to the tree root. An exception is thrown for elements with no attachment.
- **Multiple parents** — each element may have at most one parent attachment. A second attachment to the same child raises an exception.
- **Cycles** — circular attachment chains are detected and raise an exception.
- **Odd-count symmetry tags** — a warning is printed if a `symmetry_tag` appears an odd number of times (expected to come in mirrored pairs).

Rotor coincidence (two rotors at the same world position) is logged as a warning but does not throw.
