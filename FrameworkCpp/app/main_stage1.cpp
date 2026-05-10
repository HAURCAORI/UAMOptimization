#include <iostream>

#include "analysis/ComparisonReporter.hpp"
#include "core/HexacopterArchitecture.hpp"
#include "evaluation/ArchitectureEvaluator.hpp"

int main() {
    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::ArchitectureEvaluator evaluator;

    const auto result = evaluator.evaluate(architecture);

    std::cout << "HexaArch Phase 0 smoke run\n";
    std::cout << "Architecture id: " << architecture.id() << '\n';
    std::cout << "Placeholder objective: " << result.combined_objective << '\n';
    std::cout << "Summary: " << hexaarch::analysis::ComparisonReporter::summarize(result) << '\n';
    return 0;
}
