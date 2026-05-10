#pragma once

#include "evaluation/EvaluationResult.hpp"

namespace hexaarch::evaluation {

class ObjectiveAggregator {
public:
    [[nodiscard]] double aggregate(const EvaluationResult& result) const;
};

}  // namespace hexaarch::evaluation
