#pragma once

#include <vector>

#include "calibration/CalibrationProblem.hpp"

namespace hexaarch::evaluation { struct EvaluationContext; }

namespace hexaarch::calibration {

struct CalibrationOptions {
    int max_iterations = 800;
    double tolerance = 1e-7;
    double initial_step_fraction = 0.10;  // 10% of (upper-lower) per axis for the initial simplex
    unsigned random_seed = 12345U;        // restart seed when reflecting outside the box
    bool verbose = false;
};

struct CalibrationDiagnostics {
    int iterations = 0;
    double final_cost = 0.0;
    double initial_cost = 0.0;
    double mean_residual_w = 0.0;
    double max_residual_w = 0.0;
    bool converged = false;
};

struct CalibrationOutcome {
    CalibrationParameters parameters;
    CalibrationDiagnostics diagnostics;
};

// Identifies physics parameters from flight data by minimizing the calibration residual.
// Uses Nelder–Mead in box-projected mode (we project iterates back into [lower, upper] on every
// step; this is the simplest bound-respecting variant and is enough for ≤6-D problems). Returns
// the calibrated parameters plus a diagnostics struct with mean/max residual after fit.
class Calibrator {
public:
    [[nodiscard]] CalibrationOutcome fit(
        const CalibrationProblem& problem,
        const CalibrationParameters& initial,
        const CalibrationOptions& options) const;
};

// Apply calibrated parameters to a context (copies the relevant fields). Parameters not exposed
// by the context (parasite_drag_area_m2) are written to the new context field.
void applyToContext(
    const CalibrationParameters& params,
    evaluation::EvaluationContext& context);

}  // namespace hexaarch::calibration
