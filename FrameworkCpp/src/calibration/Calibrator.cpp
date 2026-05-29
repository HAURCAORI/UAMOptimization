#include "calibration/Calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

#include "evaluation/EvaluationContext.hpp"

namespace hexaarch::calibration {
namespace {

// Box-project x into [lower, upper]. Operates in-place.
void clampToBox(std::vector<double>& x,
                const std::vector<double>& lower,
                const std::vector<double>& upper) {
    for (std::size_t i = 0; i < x.size(); ++i) {
        x[i] = std::clamp(x[i], lower[i], upper[i]);
    }
}

double evaluateCost(const CalibrationProblem& problem,
                    const std::vector<double>& x,
                    const CalibrationParameters& initial) {
    const auto params = problem.unpack(x, initial);
    return problem.residualCost(params);
}

}  // namespace

CalibrationOutcome Calibrator::fit(
    const CalibrationProblem& problem,
    const CalibrationParameters& initial,
    const CalibrationOptions& options) const {

    CalibrationOutcome outcome;
    outcome.parameters = initial;
    outcome.diagnostics.initial_cost = problem.residualCost(initial);
    outcome.diagnostics.final_cost = outcome.diagnostics.initial_cost;

    const auto lower = problem.lowerActive();
    const auto upper = problem.upperActive();
    const int n = problem.numActive();
    if (n == 0) {
        outcome.diagnostics.converged = true;
        outcome.diagnostics.mean_residual_w = 0.0;
        outcome.diagnostics.max_residual_w = 0.0;
        return outcome;
    }

    // Build initial simplex: x0 plus n vertices offset along each axis by step·(upper-lower).
    std::vector<std::vector<double>> simplex;
    simplex.reserve(static_cast<std::size_t>(n + 1));
    std::vector<double> x0 = problem.pack(initial);
    clampToBox(x0, lower, upper);
    simplex.push_back(x0);
    for (int i = 0; i < n; ++i) {
        std::vector<double> xi = x0;
        const double span = upper[static_cast<std::size_t>(i)] - lower[static_cast<std::size_t>(i)];
        const double step = options.initial_step_fraction * (span > 0.0 ? span : 1.0);
        xi[static_cast<std::size_t>(i)] += step;
        clampToBox(xi, lower, upper);
        simplex.push_back(xi);
    }

    std::vector<double> costs(simplex.size(), 0.0);
    for (std::size_t i = 0; i < simplex.size(); ++i) {
        costs[i] = evaluateCost(problem, simplex[i], initial);
    }

    // Nelder–Mead coefficients.
    constexpr double alpha = 1.0;  // reflection
    constexpr double gamma = 2.0;  // expansion
    constexpr double rho   = 0.5;  // contraction
    constexpr double sigma = 0.5;  // shrink

    int iter = 0;
    bool converged = false;
    for (; iter < options.max_iterations; ++iter) {
        // Sort simplex by cost ascending.
        std::vector<std::size_t> order(simplex.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(),
                  [&](std::size_t a, std::size_t b) { return costs[a] < costs[b]; });
        std::vector<std::vector<double>> sorted_x(simplex.size());
        std::vector<double> sorted_f(costs.size());
        for (std::size_t i = 0; i < order.size(); ++i) {
            sorted_x[i] = simplex[order[i]];
            sorted_f[i] = costs[order[i]];
        }
        simplex = std::move(sorted_x);
        costs = std::move(sorted_f);

        // Centroid of best n vertices (exclude worst).
        std::vector<double> centroid(static_cast<std::size_t>(n), 0.0);
        for (std::size_t i = 0; i < simplex.size() - 1; ++i) {
            for (int k = 0; k < n; ++k) centroid[static_cast<std::size_t>(k)] += simplex[i][static_cast<std::size_t>(k)];
        }
        for (auto& c : centroid) c /= static_cast<double>(n);

        // Reflection.
        std::vector<double> xr(static_cast<std::size_t>(n));
        for (int k = 0; k < n; ++k) {
            xr[static_cast<std::size_t>(k)] =
                centroid[static_cast<std::size_t>(k)]
                + alpha * (centroid[static_cast<std::size_t>(k)] - simplex.back()[static_cast<std::size_t>(k)]);
        }
        clampToBox(xr, lower, upper);
        const double fr = evaluateCost(problem, xr, initial);

        if (fr < costs[0]) {
            // Expand.
            std::vector<double> xe(static_cast<std::size_t>(n));
            for (int k = 0; k < n; ++k) {
                xe[static_cast<std::size_t>(k)] =
                    centroid[static_cast<std::size_t>(k)]
                    + gamma * (xr[static_cast<std::size_t>(k)] - centroid[static_cast<std::size_t>(k)]);
            }
            clampToBox(xe, lower, upper);
            const double fe = evaluateCost(problem, xe, initial);
            if (fe < fr) {
                simplex.back() = xe;
                costs.back() = fe;
            } else {
                simplex.back() = xr;
                costs.back() = fr;
            }
        } else if (fr < costs[simplex.size() - 2]) {
            simplex.back() = xr;
            costs.back() = fr;
        } else {
            // Contraction.
            std::vector<double> xc(static_cast<std::size_t>(n));
            for (int k = 0; k < n; ++k) {
                xc[static_cast<std::size_t>(k)] =
                    centroid[static_cast<std::size_t>(k)]
                    + rho * (simplex.back()[static_cast<std::size_t>(k)] - centroid[static_cast<std::size_t>(k)]);
            }
            clampToBox(xc, lower, upper);
            const double fc = evaluateCost(problem, xc, initial);
            if (fc < costs.back()) {
                simplex.back() = xc;
                costs.back() = fc;
            } else {
                // Shrink toward best.
                for (std::size_t i = 1; i < simplex.size(); ++i) {
                    for (int k = 0; k < n; ++k) {
                        simplex[i][static_cast<std::size_t>(k)] =
                            simplex[0][static_cast<std::size_t>(k)]
                            + sigma * (simplex[i][static_cast<std::size_t>(k)] - simplex[0][static_cast<std::size_t>(k)]);
                    }
                    clampToBox(simplex[i], lower, upper);
                    costs[i] = evaluateCost(problem, simplex[i], initial);
                }
            }
        }

        // Convergence: range of costs in the simplex below tolerance.
        const double fmin = *std::min_element(costs.begin(), costs.end());
        const double fmax = *std::max_element(costs.begin(), costs.end());
        if (options.verbose && (iter % 20 == 0)) {
            std::cout << "[Calibrator] iter=" << iter
                      << " fmin=" << fmin << " fmax=" << fmax << "\n";
        }
        if (fmax - fmin < options.tolerance) {
            converged = true;
            ++iter;
            break;
        }
    }

    // Final answer: best simplex vertex.
    const auto best_it = std::min_element(costs.begin(), costs.end());
    const std::size_t best_idx = static_cast<std::size_t>(std::distance(costs.begin(), best_it));
    outcome.parameters = problem.unpack(simplex[best_idx], initial);
    outcome.diagnostics.iterations = iter;
    outcome.diagnostics.final_cost = *best_it;
    outcome.diagnostics.converged = converged;

    // Residual stats in absolute Watts (more interpretable than the squared-relative cost).
    double sum_abs = 0.0;
    double max_abs = 0.0;
    for (const auto& p : problem.data()) {
        const double pred = problem.predictPower(outcome.parameters, p);
        const double err = std::abs(pred - p.power_total_w);
        sum_abs += err;
        max_abs = std::max(max_abs, err);
    }
    if (!problem.data().empty()) {
        outcome.diagnostics.mean_residual_w = sum_abs / static_cast<double>(problem.data().size());
    }
    outcome.diagnostics.max_residual_w = max_abs;

    return outcome;
}

void applyToContext(const CalibrationParameters& params,
                    evaluation::EvaluationContext& context) {
    context.figure_of_merit                    = params.figure_of_merit;
    context.motor_efficiency                   = params.motor_efficiency;
    context.esc_efficiency                     = params.esc_efficiency;
    context.battery_specific_energy_wh_per_kg  = params.battery_specific_energy_wh_per_kg;
    context.battery_pack_efficiency            = params.battery_pack_efficiency;
    context.parasite_drag_area_m2              = params.parasite_drag_area_m2;
}

}  // namespace hexaarch::calibration
