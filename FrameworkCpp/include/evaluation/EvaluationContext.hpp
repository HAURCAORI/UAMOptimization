#pragma once

#include <string>
#include <vector>

#include "physics/Material.hpp"

namespace hexaarch::evaluation {

struct ObjectiveWeight {
    std::string name;
    double weight = 0.0;
};

struct EvaluationContext {
    bool stage1_only = true;
    double gamma_thrust_required = 1.5;
    double minimum_fault_allocation_ratio = 0.05;
    double minimum_arm_length = 0.5;
    double minimum_outer_arm_delta = 0.1;
    physics::Material arm_material = physics::Materials::Al7075();
    double minimum_arm_safety_factor = 1.5;
    std::vector<ObjectiveWeight> objective_weights{
        {"mass", 0.20},
        {"power", 0.20},
        {"fault_thrust", 0.25},
        {"fault_alloc", 0.25},
        {"hover_nom", 0.10},
        {"structural", 0.0},
        {"packaging", 0.0},
        {"structural_safety", 0.0}
    };
};

}  // namespace hexaarch::evaluation
