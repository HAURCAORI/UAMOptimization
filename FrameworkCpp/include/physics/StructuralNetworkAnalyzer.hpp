#pragma once

#include <array>
#include <vector>

#include "physics/PhysicsTypes.hpp"

namespace hexaarch::core {
class HexacopterArchitecture;
}

namespace hexaarch::evaluation {
struct EvaluationContext;
}

namespace hexaarch::physics {

// Phase 3 structural network analyzer.
// Replaces StructuralAnalyzer. Runs multiple load cases (max_thrust, nominal_hover,
// all 6 single-fault hover trims) for every IStructuralBeam element, computes full
// von Mises stress (axial + bending + torsion), Euler-Bernoulli tip deflection, and
// safety factor per member per load case.
//
// Populates model.network_structural (per-member per-load-case results),
// model.arm_structural (max_thrust case only, for backwards-compatible JSON export),
// and model.structural.{min_safety_factor, network_*} summary fields.
class StructuralNetworkAnalyzer {
public:
    // nominal_thrust: ACS LP trim solution for nominal hover [N per rotor, 0-indexed].
    // fault_thrusts:  trim solution for each single-fault case; fault_thrusts[i] has motor i off.
    void analyze(
        PhysicalModel& model,
        const core::HexacopterArchitecture& arch,
        const evaluation::EvaluationContext& context,
        const std::array<double, kNumRotors>& nominal_thrust,
        const std::vector<std::array<double, kNumRotors>>& fault_thrusts) const;
};

}  // namespace hexaarch::physics
