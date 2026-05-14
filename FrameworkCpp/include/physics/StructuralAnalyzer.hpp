#pragma once

#include "physics/PhysicsTypes.hpp"

namespace hexaarch::core {
class HexacopterArchitecture;
}

namespace hexaarch::evaluation {
struct EvaluationContext;
}

namespace hexaarch::physics {

class StructuralAnalyzer {
public:
    void analyze(
        PhysicalModel& model,
        const core::HexacopterArchitecture& arch,
        const evaluation::EvaluationContext& context) const;
};

}  // namespace hexaarch::physics
