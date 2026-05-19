# FrameworkCpp Spatial Element Manual

This manual explains how to add or modify `SpatialElement` implementations and how they interact with the assembly, packaging, structural, propulsion, and battery subsystems.

## Core interface

Every element implements `SpatialElement` from [SpatialElement.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/core/SpatialElement.hpp:1).

Required responsibilities:

- identify itself with `id()` and `type()`
- support deep copy through `clone()`
- register and rebind parameters
- register constraints
- provide mass, COM, inertia, local geometry, local pose, and anchors
- recompute all derived state inside `updateFromParameters()`

For most element types, derive from `BasicSpatialElement` in [Elements.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/core/Elements.hpp:1) instead of implementing storage manually.

## Capability interfaces

Elements opt into framework subsystems by implementing capability interfaces from [ElementCapabilities.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/core/ElementCapabilities.hpp:1).

### `IPropulsionRotor`

Use when the element represents a thrust-producing rotor disk.

- contributes rotor position to the allocation matrix
- contributes yaw sign through `yawMomentSign()`
- participates in rotor clearance checks

### `IStructuralMember`

Use when the element contributes structural span and optionally frame mass.

- `structuralSpanContribution()` feeds structural span metrics
- `contributesToFrameMass()` controls whether the element counts toward frame mass

### `IStructuralBeam`

Use when the structural analyzer needs cross-section properties.

- extends `IStructuralMember`
- provides outer radius, inner radius, area, and second moment of area
- required for the current arm structural safety analysis

### `IMotorMassContributor`

Use for concentrated motor-class masses whose world positions affect inertia buildup.

### `IPayloadMassContributor`

Use for the payload element whose mass should be reported as payload mass in the physical model.

### `IEnergyStorage`

Use for battery-like elements that expose stored-energy mass through `batteryMass()`.

## Geometry primitives

Geometry primitives are defined in [GeometryPrimitive.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/core/GeometryPrimitive.hpp:1).

Available helpers:

- `makeSphere(radius, padding)`
- `makeBox(half_extents, padding)`
- `makeCylinder(radius, length, padding)`
- `makeDisk(radius, padding)`
- `makeSegment(length, padding)`

Guidance:

- use `makeDisk()` for rotors
- use `makeSegment()` for beams / arms
- use `makeBox()` for body or battery volumes
- use padding only when you want the packaging check to treat the element as effectively larger than its raw geometry

## Anchors and attachments

Anchors are named local poses stored on each element. Attachments connect a parent anchor to a child anchor.

Current attachment machinery lives in [Attachment.hpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/include/core/Attachment.hpp:1).

Relationship helpers:

- `AttachmentRelationship::rigidMount()`
- `AttachmentRelationship::localOffset(offset)`
- `AttachmentRelationship::mirroredLocalOffset(offset, mirror_signs)`

Contact policies:

- `enforce_clearance`: parent and child geometry must not overlap
- `allow_touch`: contact at zero clearance is acceptable
- `bonded_overlap`: overlap is allowed and exempted from packaging checks

Important rule:

- update anchors inside `updateFromParameters()` whenever anchor locations depend on design parameters

## Current built-in elements

### `BodyElement`

- consumes `Lx`, `Lyi`, `Lyo`
- provides the central body box and body anchors
- registers `body_span_order`

### `ArmElement`

- consumes `Lx`, `Lyi`, `Lyo`, `arm_outer_radius`, `arm_wall_thickness`
- implements `IStructuralBeam` and `ILoadReceiver`
- registers `arm_length_positive`
- provides span and section data to the structural analyzer

### `MotorElement`

- consumes `T_max`
- implements `IMotorMassContributor`
- registers `motor_thrust_positive`

### `RotorElement`

- consumes `d_prop`
- implements `IPropulsionRotor`
- registers `rotor_diameter_positive`

### `BatteryElement`

- consumes `m_bat`, `T_max`, `d_prop`
- implements `IEnergyStorage`
- contributes actual mass to the vehicle
- registers `battery_mass_positive`

### `PayloadElement`

- consumes `m_payload`
- implements `IPayloadMassContributor`
- registers `payload_mass_nonnegative`

## Default builder flow

The default architecture is assembled in [DefaultHexacopterBuilder.cpp](C:/Local/Matlab/UAMOptimization/FrameworkCpp/src/core/DefaultHexacopterBuilder.cpp:1).

Element creation:

- body
- battery
- payload
- 6 arms
- 6 motors
- 6 rotors

Attachment pattern:

- root -> body
- body -> battery
- body -> payload
- body -> each arm
- arm -> motor
- motor -> rotor

If you introduce a new default element, update both:

- `DefaultHexacopterParameters`
- `DefaultHexacopterBuilder::buildElements()` and possibly `buildAttachments()`

## Step-by-step element extension

1. Add the class declaration in a header.
2. Derive from `BasicSpatialElement`.
3. Add parameter pointers as members.
4. Implement `clone()`.
5. In `registerParameters()`, call `parameter->addConsumer(id_)` for every consumed parameter.
6. In `rebindParameters()`, reacquire parameters from `ParameterRegistry`.
7. In `registerConstraints()`, capture stable IDs, not raw parameter pointers.
8. In `updateFromParameters()`, recompute:
   - `mass_`
   - `local_com_`
   - `local_inertia_`
   - `primitives_`
   - `anchors_`
9. Add the element to the default builder or architecture.
10. Register any new files in `FrameworkCpp.vcxproj`.

## Constraint registration guidance

Prefer this pattern inside `registerConstraints()`:

1. capture `stable_id`
2. refetch the parameter from `context.architecture.parameters()` inside the lambda
3. build a small `Constraint` object with the target sense / threshold
4. return `constraint.evaluate(measured_value)`

Do not capture raw `DesignParameter*` directly in the lambda. Elements are cloned and rebound, so pointer capture is the wrong lifetime.

## Common mistakes

- forgetting to call `addConsumer()` for a parameter
- forgetting to rebind parameters after cloning
- setting anchors only once in the constructor instead of `updateFromParameters()`
- registering new files in the filesystem but not in `FrameworkCpp.vcxproj`
- using an element capability that does not match the analyzer expectations
- adding a new structural beam but not supplying `IStructuralBeam` section properties

## Validation checklist

After adding an element:

1. build succeeds
2. `updateFromParameters()` changes geometry as expected
3. assembly rebuild completes without orphan or cycle errors
4. packaging behaves as intended
5. exported JSON contains the expected physical-model changes
6. constraint results show the new constraint if one was added
