#include "physics/VehicleScalingModel.hpp"

namespace hexaarch::physics {

MassProperties VehicleScalingModel::evaluate(const core::HexacopterArchitecture&) const {
    return MassProperties{0.0};
}

}  // namespace hexaarch::physics
