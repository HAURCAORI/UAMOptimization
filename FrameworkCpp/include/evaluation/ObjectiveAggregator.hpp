#pragma once

#include "evaluation/EvaluationContext.hpp"
#include "evaluation/EvaluationResult.hpp"

namespace hexaarch::evaluation {

class ObjectiveAggregator {
public:
    double aggregate(EvaluationResult& result, const EvaluationContext& context) const;
};

}  // namespace hexaarch::evaluation
