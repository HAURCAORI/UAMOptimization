#pragma once

#include <algorithm>

#include "eigen3/Eigen/Dense"
#include "core/AppliedLoad.hpp"

namespace hexaarch::core {

// Axis-aligned bounding box in an element's local frame [m].
// min_corner / max_corner define the inner containment volume (no padding).
struct LocalAABB {
    Eigen::Vector3d min_corner = Eigen::Vector3d::Zero();
    Eigen::Vector3d max_corner = Eigen::Vector3d::Zero();

    // Returns max(0, amount by which 'inner' protrudes outside 'this').
    // Zero means 'inner' is fully contained.
    [[nodiscard]] double containmentViolation(const LocalAABB& inner) const {
        double v = 0.0;
        for (int i = 0; i < 3; ++i) {
            v = std::max(v, inner.max_corner[i] - max_corner[i]);
            v = std::max(v, min_corner[i]       - inner.min_corner[i]);
        }
        return v;
    }

    // Returns the minimum penetration depth if this AABB overlaps 'other', 0 if separated.
    // Positive result = overlap; use as constraint value (must stay ≤ 0 for non-overlap).
    [[nodiscard]] double overlapMagnitude(const LocalAABB& other) const {
        double min_pen = 1e30;
        for (int i = 0; i < 3; ++i) {
            const double pen = std::min(max_corner[i], other.max_corner[i])
                             - std::max(min_corner[i], other.min_corner[i]);
            if (pen <= 0.0) { return 0.0; }
            min_pen = std::min(min_pen, pen);
        }
        return min_pen;
    }
};

// Implemented by elements that define a spatial containment envelope (hub, fuselage, bay).
// The returned AABB is in the element's local frame; callers transform via assembly world pose.
class IEnvelopeProvider {
public:
    virtual ~IEnvelopeProvider() = default;
    [[nodiscard]] virtual LocalAABB localEnvelope() const = 0;
};

class IPropulsionRotor {
public:
    virtual ~IPropulsionRotor() = default;
    [[nodiscard]] virtual int rotorIndex() const = 0;
    [[nodiscard]] virtual double yawMomentSign() const = 0;
};

class IStructuralMember {
public:
    virtual ~IStructuralMember() = default;
    [[nodiscard]] virtual double structuralSpanContribution() const = 0;
    [[nodiscard]] virtual bool contributesToFrameMass() const = 0;
};

class IStructuralBeam : public IStructuralMember {
public:
    [[nodiscard]] virtual double outerRadius() const = 0;
    [[nodiscard]] virtual double innerRadius() const = 0;
    [[nodiscard]] virtual double crossSectionArea() const = 0;
    [[nodiscard]] virtual double secondMomentOfArea() const = 0;
    // Polar moment of area J [m⁴]. For circular hollow: J = 2I exactly.
    [[nodiscard]] virtual double polarMomentOfArea() const = 0;
};

class IMotorMassContributor {
public:
    virtual ~IMotorMassContributor() = default;
};

class IPayloadMassContributor {
public:
    virtual ~IPayloadMassContributor() = default;
};

// Implemented by elements that store energy (battery packs).
// BatteryEvaluator queries this to obtain the battery mass design variable.
class IEnergyStorage {
public:
    virtual ~IEnergyStorage() = default;
    [[nodiscard]] virtual double batteryMass() const = 0;
};

}  // namespace hexaarch::core
