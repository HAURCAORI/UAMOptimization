#pragma once

#include "core/HexacopterArchitecture.hpp"
#include "evaluation/EvaluationContext.hpp"
#include "evaluation/EvaluationResult.hpp"

namespace hexaarch::evaluation {

class ArchitectureEvaluator {
public:
    [[nodiscard]] EvaluationResult evaluate(
        const core::HexacopterArchitecture& architecture,
        const EvaluationContext& context = {}) const;
};

}  // namespace hexaarch::evaluation
