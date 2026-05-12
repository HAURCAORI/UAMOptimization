#include "evaluation/ArchitectureEvaluator.hpp"

#include "core/HexacopterArchitecture.hpp"
#include "evaluation/Stage1Evaluator.hpp"

namespace hexaarch::evaluation {

EvaluationResult ArchitectureEvaluator::evaluate(
    const core::HexacopterArchitecture& architecture,
    const EvaluationContext& context) const {
    core::HexacopterArchitecture working_copy = architecture;
    working_copy.updateFromParameters();
    return Stage1Evaluator{}.evaluate(working_copy, context);
}

}  // namespace hexaarch::evaluation
