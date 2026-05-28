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
