#include "evaluation/Stage1Evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <vector>

#include "evaluation/ObjectiveAggregator.hpp"
#include "physics/AllocationMatrixBuilder.hpp"
#include "physics/StructuralAnalyzer.hpp"
#include "physics/VehicleScalingModel.hpp"

namespace hexaarch::evaluation {
namespace {

struct HoverSolution {
    bool feasible = false;
    double utilization = std::numeric_limits<double>::infinity();
    std::array<double, 6> thrust{};
};

Eigen::Vector4d hoverTarget(const physics::PhysicalModel& model) {
    return {
        -model.mass_properties.mass * model.propulsion.gravity,
        0.0,
        0.0,
        0.0
    };
}

std::optional<std::array<double, 6>> solveBoundedHover(
    const physics::AllocationMatrix& matrix,
    const std::array<double, 6>& upper_bounds,
    const Eigen::Vector4d& target) {
    std::array<double, 6> best{};
    double best_sum = std::numeric_limits<double>::infinity();
    bool found = false;

    for (int i = 0; i < 6; ++i) {
        for (int j = i + 1; j < 6; ++j) {
            std::array<int, 4> basic{};
            int cursor = 0;
            for (int index = 0; index < 6; ++index) {
                if (index != i && index != j) {
                    basic.at(cursor++) = index;
                }
            }

            for (int mask = 0; mask < 4; ++mask) {
                Eigen::Vector4d rhs = target;
                std::array<double, 6> candidate{};
                candidate.fill(0.0);

                candidate.at(i) = (mask & 1) ? upper_bounds.at(i) : 0.0;
                candidate.at(j) = (mask & 2) ? upper_bounds.at(j) : 0.0;
                rhs -= matrix.col(i) * candidate.at(i);
                rhs -= matrix.col(j) * candidate.at(j);

                Eigen::Matrix4d basis;
                for (int col = 0; col < 4; ++col) {
                    basis.col(col) = matrix.col(basic.at(col));
                }

                const Eigen::FullPivLU<Eigen::Matrix4d> lu(basis);
                if (!lu.isInvertible()) {
                    continue;
                }

                const Eigen::Vector4d solution = lu.solve(rhs);
                bool feasible = true;
                for (int col = 0; col < 4; ++col) {
                    const int variable_index = basic.at(col);
                    candidate.at(variable_index) = solution(col);
                    if (candidate.at(variable_index) < -1e-6 ||
                        candidate.at(variable_index) > upper_bounds.at(variable_index) + 1e-6) {
                        feasible = false;
                        break;
                    }
                }

                if (!feasible) {
                    continue;
                }

                double sum = 0.0;
                for (int index = 0; index < 6; ++index) {
                    candidate.at(index) = std::clamp(candidate.at(index), 0.0, upper_bounds.at(index));
                    sum += candidate.at(index);
                }

                if (sum < best_sum) {
                    best = candidate;
                    best_sum = sum;
                    found = true;
                }
            }
        }
    }

    if (!found) {
        return std::nullopt;
    }
    return best;
}

HoverSolution hoverFeasibility(
    const physics::AllocationMatrix& matrix,
    const double thrust_max,
    const physics::PhysicalModel& model,
    const std::array<double, 6>& loss_of_effectiveness) {
    HoverSolution solution;

    std::array<double, 6> upper_bounds{};
    for (int index = 0; index < 6; ++index) {
        upper_bounds.at(index) = thrust_max * (1.0 - loss_of_effectiveness.at(index));
    }

    const auto candidate = solveBoundedHover(matrix, upper_bounds, hoverTarget(model));
    if (!candidate.has_value()) {
        return solution;
    }

    solution.feasible = true;
    solution.thrust = candidate.value();
    solution.utilization = 0.0;
    for (int index = 0; index < 6; ++index) {
        solution.utilization = std::max(solution.utilization, solution.thrust.at(index) / std::max(upper_bounds.at(index), 1e-12));
    }
    return solution;
}

double hoverPowerProxy(const std::array<double, 6>& thrust, const double propeller_diameter) {
    constexpr double pi = 3.14159265358979323846;
    const double disk_area = pi * std::pow(0.5 * propeller_diameter, 2);
    double power = 0.0;
    for (const double thrust_i : thrust) {
        power += std::pow(std::max(thrust_i, 0.0), 1.5);
    }
    return power / std::max(std::sqrt(disk_area), 1e-9);
}

double scaledControlEffectiveness(
    const physics::AllocationMatrix& matrix,
    const std::array<double, 6>& upper_bounds,
    const physics::PhysicalModel& model) {
    const double length_scale = std::max(model.structural.max_arm_length, 1e-3);
    double yaw_ref = 0.0;
    for (int index = 0; index < 6; ++index) {
        yaw_ref += std::abs(matrix(3, index)) * upper_bounds.at(index);
    }
    yaw_ref = std::max(yaw_ref, 1e-9);

    Eigen::Matrix4d scale = Eigen::Matrix4d::Zero();
    const double mg = model.mass_properties.mass * model.propulsion.gravity;
    scale(0, 0) = 1.0 / std::max(mg, 1e-9);
    scale(1, 1) = 1.0 / std::max(mg * length_scale, 1e-9);
    scale(2, 2) = 1.0 / std::max(mg * length_scale, 1e-9);
    scale(3, 3) = 1.0 / yaw_ref;

    Eigen::Matrix<double, 6, 6> thrust_scale = Eigen::Matrix<double, 6, 6>::Zero();
    for (int index = 0; index < 6; ++index) {
        thrust_scale(index, index) = upper_bounds.at(index);
    }

    const Eigen::JacobiSVD<Eigen::Matrix<double, 4, 6>> svd(scale * matrix * thrust_scale, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto singular_values = svd.singularValues();
    return singular_values.size() > 0 ? singular_values(singular_values.size() - 1) : 0.0;
}

}  // namespace

EvaluationResult Stage1Evaluator::evaluate(
    const core::HexacopterArchitecture& architecture,
    const EvaluationContext& context) const {
    EvaluationResult result;

    physics::VehicleScalingModel model_builder;
    result.physical_model = model_builder.evaluate(architecture);

    physics::StructuralAnalyzer{}.analyze(result.physical_model, architecture, context);

    static const physics::PhysicalModel s_reference_model = []() {
        return physics::VehicleScalingModel{}.evaluate(core::HexacopterArchitecture{});
    }();
    const physics::PhysicalModel& reference_model = s_reference_model;
    const auto nominal_hover = hoverFeasibility(
        result.physical_model.allocation_matrix,
        result.physical_model.propulsion.thrust_max,
        result.physical_model,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    result.stage1.hover_ok_nominal = nominal_hover.feasible;
    result.stage1.hover_utilization_nominal = nominal_hover.utilization;

    result.stage1.mass =
        result.physical_model.mass_properties.mass / std::max(result.physical_model.reference_mass, 1e-9);
    result.stage1.hover_nom = nominal_hover.feasible
        ? std::pow(
              (std::accumulate(nominal_hover.thrust.begin(), nominal_hover.thrust.end(), 0.0) / 6.0) /
                  std::max(result.physical_model.propulsion.thrust_max, 1e-9),
              2)
        : std::numeric_limits<double>::infinity();

    const double nominal_power = nominal_hover.feasible
        ? hoverPowerProxy(nominal_hover.thrust, result.physical_model.propulsion.propeller_diameter)
        : std::numeric_limits<double>::infinity();
    const auto ref_hover = hoverFeasibility(
        reference_model.allocation_matrix,
        reference_model.propulsion.thrust_max,
        reference_model,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    const double reference_power = ref_hover.feasible
        ? hoverPowerProxy(ref_hover.thrust, reference_model.propulsion.propeller_diameter)
        : std::max(result.physical_model.reference_power, 1e-9);
    result.stage1.power = nominal_power / std::max(reference_power, 1e-9);

    double gamma_worst = std::numeric_limits<double>::infinity();
    double sigma_worst = std::numeric_limits<double>::infinity();
    double sigma_reference = std::numeric_limits<double>::infinity();
    bool all_single_fault_hover_ok = true;

    for (int index = 0; index < 6; ++index) {
        std::array<double, 6> loss{};
        loss.fill(0.0);
        loss.at(index) = 1.0;

        std::array<double, 6> upper_bounds{};
        std::array<double, 6> upper_bounds_ref{};
        for (int motor = 0; motor < 6; ++motor) {
            upper_bounds.at(motor) = result.physical_model.propulsion.thrust_max * (1.0 - loss.at(motor));
            upper_bounds_ref.at(motor) = reference_model.propulsion.thrust_max * (1.0 - loss.at(motor));
        }

        gamma_worst = std::min(
            gamma_worst,
            std::accumulate(upper_bounds.begin(), upper_bounds.end(), 0.0) /
                std::max(result.physical_model.mass_properties.mass * result.physical_model.propulsion.gravity, 1e-9));
        sigma_worst = std::min(
            sigma_worst,
            scaledControlEffectiveness(
                result.physical_model.allocation_matrix,
                upper_bounds,
                result.physical_model));
        sigma_reference = std::min(
            sigma_reference,
            scaledControlEffectiveness(reference_model.allocation_matrix, upper_bounds_ref, reference_model));

        const auto fault_hover = hoverFeasibility(
            result.physical_model.allocation_matrix,
            result.physical_model.propulsion.thrust_max,
            result.physical_model,
            loss);
        all_single_fault_hover_ok = all_single_fault_hover_ok && fault_hover.feasible;
    }

    result.stage1.gamma_worst = gamma_worst;
    result.stage1.sigma_worst = sigma_worst;
    result.stage1.sigma_reference = sigma_reference;
    result.stage1.fault_thrust = std::pow(std::max(0.0, context.gamma_thrust_required - gamma_worst), 2);
    result.stage1.fault_alloc = sigma_reference / std::max(sigma_worst, 1e-9);
    result.stage1.structural = result.physical_model.structural.normalized_bending_index;
    result.stage1.packaging = result.physical_model.packaging.overlap_penalty;
    {
        const double min_sf = result.physical_model.structural.min_safety_factor;
        result.stage1.structural_safety = context.minimum_arm_safety_factor / std::max(min_sf, 1e-9);
    }

    result.constraint_results.clear();
    const core::ConstraintEvaluationContext constraint_context{
        architecture,
        result.physical_model,
        result.stage1,
        context
    };
    for (const auto& constraint : architecture.constraints().constraints()) {
        result.constraint_results.push_back({
            constraint.stable_id(),
            constraint.owner_id,
            constraint.name,
            constraint.hard,
            constraint.active,
            constraint.evaluate(constraint_context)
        });
    }

    result.feasible =
        nominal_hover.feasible &&
        all_single_fault_hover_ok &&
        std::all_of(result.constraint_results.begin(), result.constraint_results.end(), [](const ConstraintResult& entry) {
            return !entry.hard || entry.evaluation.feasible;
        });

    ObjectiveAggregator aggregator;
    aggregator.aggregate(result, context);

    result.message = result.feasible ? "Stage 1 evaluation completed" : "Stage 1 evaluation detected infeasible constraints";
    return result;
}

}  // namespace hexaarch::evaluation
