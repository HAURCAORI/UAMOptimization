#pragma once

#include <string>
#include <vector>

#include "core/Constraint.hpp"
#include "physics/PhysicsTypes.hpp"

namespace hexaarch::evaluation {

struct ObjectiveValue {
    std::string name;
    double value = 0.0;
    double weight = 0.0;
};

struct ConstraintResult {
    std::string stable_id;
    std::string owner_id;
    std::string name;
    bool hard = true;
    bool active = true;
    core::ConstraintEvaluation evaluation;
};

struct Stage1Metrics {
    double mass = 0.0;
    double power = 0.0;
    double fault_thrust = 0.0;
    double fault_alloc = 0.0;
    double hover_nom = 0.0;
    double structural = 0.0;
    double packaging = 0.0;
    double gamma_worst = 0.0;
    double sigma_worst = 0.0;
    double sigma_reference = 0.0;
    double hover_utilization_nominal = 0.0;
    bool hover_ok_nominal = false;
};

struct EvaluationResult {
    bool feasible = true;
    double combined_objective = 0.0;
    std::string message;
    Stage1Metrics stage1;
    std::vector<ObjectiveValue> objectives;
    std::vector<ConstraintResult> constraint_results;
    physics::PhysicalModel physical_model;
};

}  // namespace hexaarch::evaluation
