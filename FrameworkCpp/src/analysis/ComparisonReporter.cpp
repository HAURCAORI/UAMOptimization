#include "analysis/ComparisonReporter.hpp"

namespace hexaarch::analysis {

std::string ComparisonReporter::summarize(const evaluation::EvaluationResult& result) {
    return result.message;
}

}  // namespace hexaarch::analysis
