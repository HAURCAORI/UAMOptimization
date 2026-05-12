#pragma once

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

class IMotorMassContributor {
public:
    virtual ~IMotorMassContributor() = default;
};

class IPayloadMassContributor {
public:
    virtual ~IPayloadMassContributor() = default;
};

}  // namespace hexaarch::core
