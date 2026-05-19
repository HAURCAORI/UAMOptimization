#pragma once

#include <array>

#include "physics/PhysicsTypes.hpp"

namespace hexaarch::evaluation {
struct EvaluationContext;
}

namespace hexaarch::physics {

// Computes per-motor and total electrical hover power using actuator-disk theory.
// Effective disk radius = max_arm_length / 2 (uses arm geometry, not d_prop).
// This separates the power model from d_prop, which is frozen for cT computation only.
class PowertrainEvaluator {
public:
    [[nodiscard]] PowertrainResult evaluate(
        const std::array<double, kNumRotors>& trim_thrust_nominal,
        const std::array<double, kNumRotors>& trim_thrust_faulted,
        double thrust_max,
        double max_arm_length,
        const evaluation::EvaluationContext& context) const;
};

}  // namespace hexaarch::physics
