#pragma once

#include "core/HexacopterArchitecture.hpp"
#include "physics/PhysicsTypes.hpp"

namespace hexaarch::physics {

class VehicleScalingModel {
public:
    [[nodiscard]] MassProperties evaluate(const core::HexacopterArchitecture& architecture) const;
};

}  // namespace hexaarch::physics
