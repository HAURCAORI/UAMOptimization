#pragma once

#include "core/AppliedLoad.hpp"

namespace hexaarch::core {

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
