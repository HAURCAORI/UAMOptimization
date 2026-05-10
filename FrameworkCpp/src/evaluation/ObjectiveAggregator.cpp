#include "evaluation/ObjectiveAggregator.hpp"

namespace hexaarch::evaluation {

double ObjectiveAggregator::aggregate(const EvaluationResult& result) const {
    return result.combined_objective;
}

}  // namespace hexaarch::evaluation
