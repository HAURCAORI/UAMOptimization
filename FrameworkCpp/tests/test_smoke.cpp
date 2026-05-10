#include <gtest/gtest.h>

#include "core/HexacopterArchitecture.hpp"
#include "evaluation/ArchitectureEvaluator.hpp"

TEST(Phase0Smoke, EvaluatesDefaultArchitecture) {
    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::ArchitectureEvaluator evaluator;

    const auto result = evaluator.evaluate(architecture);

    EXPECT_TRUE(result.feasible);
    EXPECT_EQ(result.combined_objective, 0.0);
}
