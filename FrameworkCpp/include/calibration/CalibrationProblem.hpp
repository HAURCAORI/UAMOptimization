#pragma once

#include <array>
#include <string>
#include <vector>

#include "calibration/FlightDataPoint.hpp"

namespace hexaarch::evaluation { struct EvaluationContext; }

namespace hexaarch::calibration {

// Calibratable physics parameters identified from measured flight data. Each parameter has a
// physical lower/upper bound; the optimizer projects iterates back into the box at every step.
//
//   figure_of_merit         hover induced-power efficiency (PowertrainEvaluator + cruise)
//   motor_efficiency        mechanical motor efficiency
//   esc_efficiency          electrical ESC efficiency
//   battery_specific_energy battery Wh/kg at cell level
//   battery_pack_efficiency pack-level discharge efficiency
//   parasite_drag_area      airframe Cd·A_ref [m²] driving cruise power
//
// Disk area is implied by the architecture's max_arm_length: r_eff = max_arm_length/2.
// The user passes a representative effective single-rotor disk area in CalibrationProblem so the
// residual function doesn't need to drag in the full HexacopterArchitecture.
struct CalibrationParameters {
    double figure_of_merit = 0.65;
    double motor_efficiency = 0.85;
    double esc_efficiency = 0.95;
    double battery_specific_energy_wh_per_kg = 250.0;
    double battery_pack_efficiency = 0.95;
    double parasite_drag_area_m2 = 0.6;
};

struct CalibrationBounds {
    std::array<double, 6> lower{ 0.30, 0.50, 0.70, 100.0, 0.70, 0.10 };
    std::array<double, 6> upper{ 0.90, 0.98, 0.99, 400.0, 0.99, 2.50 };
};

// Active parameter mask: each free-running parameter contributes one dimension to the optimizer.
// Disabling a parameter freezes it at its current value (useful for ground-test data, where
// parasite drag is unobservable). All true by default.
struct CalibrationMask {
    bool figure_of_merit = true;
    bool motor_efficiency = true;
    bool esc_efficiency = true;
    bool battery_specific_energy = false;  // not observable from power telemetry; freeze unless E_avail data is supplied
    bool battery_pack_efficiency = false;
    bool parasite_drag_area = true;
};

class CalibrationProblem {
public:
    CalibrationProblem(
        std::vector<FlightDataPoint> data,
        double effective_single_rotor_disk_area_m2,
        int num_rotors,
        CalibrationBounds bounds,
        CalibrationMask mask);

    // Mean squared relative residual in electrical power. Used as the optimizer cost; residual is
    // (P_pred - P_meas) / max(P_meas, 1) so per-row weighting matches order of magnitude.
    [[nodiscard]] double residualCost(const CalibrationParameters& params) const;

    // Predicts electrical pack power [W] for one operating point using the calibrated parameters
    // (mirrors PowertrainEvaluator + CruisePowerModel exactly so calibration is self-consistent).
    [[nodiscard]] double predictPower(
        const CalibrationParameters& params,
        const FlightDataPoint& point) const;

    [[nodiscard]] const std::vector<FlightDataPoint>& data() const { return data_; }
    [[nodiscard]] const CalibrationBounds& bounds() const { return bounds_; }
    [[nodiscard]] const CalibrationMask& mask() const { return mask_; }
    [[nodiscard]] int numActive() const;

    // Pack/unpack between a dense vector (size = numActive) and the full struct.
    [[nodiscard]] std::vector<double> pack(const CalibrationParameters& params) const;
    [[nodiscard]] CalibrationParameters unpack(
        const std::vector<double>& x,
        const CalibrationParameters& fixed) const;
    [[nodiscard]] std::vector<double> lowerActive() const;
    [[nodiscard]] std::vector<double> upperActive() const;
    [[nodiscard]] std::vector<std::string> activeNames() const;

private:
    std::vector<FlightDataPoint> data_;
    double single_rotor_disk_area_ = 0.0;
    int num_rotors_ = 6;
    CalibrationBounds bounds_;
    CalibrationMask mask_;
};

}  // namespace hexaarch::calibration
