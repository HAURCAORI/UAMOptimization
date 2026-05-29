#include "calibration/CalibrationProblem.hpp"

#include <algorithm>
#include <cmath>

#include "evaluation/EvaluationContext.hpp"
#include "mission/CruisePowerModel.hpp"

namespace hexaarch::calibration {

CalibrationProblem::CalibrationProblem(
    std::vector<FlightDataPoint> data,
    const double effective_single_rotor_disk_area_m2,
    const int num_rotors,
    CalibrationBounds bounds,
    CalibrationMask mask)
    : data_(std::move(data)),
      single_rotor_disk_area_(effective_single_rotor_disk_area_m2),
      num_rotors_(num_rotors),
      bounds_(bounds),
      mask_(mask) {}

double CalibrationProblem::predictPower(
    const CalibrationParameters& params,
    const FlightDataPoint& point) const {

    // Hover-only (airspeed ≈ 0): apply the same per-rotor actuator-disk model as
    // PowertrainEvaluator, summed over n rotors with even thrust split. After summing,
    // P_total = T_total^(3/2) / (η · sqrt(2 · ρ · A_total)) with A_total = n · A_single.
    if (point.airspeed_mps <= 0.1 && std::abs(point.climb_rate_mps) <= 0.1) {
        const int n = std::max(num_rotors_, 1);
        const double A_total = single_rotor_disk_area_ * static_cast<double>(n);
        const double eta = std::max(
            params.figure_of_merit * params.motor_efficiency * params.esc_efficiency, 1e-6);
        const double denom = eta * std::sqrt(2.0 * point.air_density_kg_m3 * A_total);
        const double T_total = std::max(point.thrust_total_n, 0.0);
        return std::pow(T_total, 1.5) / std::max(denom, 1e-9);
    }

    mission::CruiseQuery q;
    q.vehicle_mass_kg    = point.mass_kg;
    q.airspeed_mps       = point.airspeed_mps;
    q.climb_rate_mps     = point.climb_rate_mps;
    q.disk_area_m2       = single_rotor_disk_area_ * static_cast<double>(std::max(num_rotors_, 1));
    q.parasite_cd_a_m2   = params.parasite_drag_area_m2;
    q.air_density_kg_m3  = point.air_density_kg_m3;

    evaluation::EvaluationContext scratch;
    scratch.figure_of_merit   = params.figure_of_merit;
    scratch.motor_efficiency  = params.motor_efficiency;
    scratch.esc_efficiency    = params.esc_efficiency;
    scratch.air_density       = point.air_density_kg_m3;
    return mission::CruisePowerModel{}.evaluate(q, scratch).electrical_power_w;
}

double CalibrationProblem::residualCost(const CalibrationParameters& params) const {
    if (data_.empty()) return 0.0;
    double sum_sq = 0.0;
    for (const auto& p : data_) {
        const double pred = predictPower(params, p);
        const double meas = std::max(p.power_total_w, 1.0);
        const double rel = (pred - p.power_total_w) / meas;
        sum_sq += rel * rel;
    }
    return sum_sq / static_cast<double>(data_.size());
}

int CalibrationProblem::numActive() const {
    int n = 0;
    n += mask_.figure_of_merit ? 1 : 0;
    n += mask_.motor_efficiency ? 1 : 0;
    n += mask_.esc_efficiency ? 1 : 0;
    n += mask_.battery_specific_energy ? 1 : 0;
    n += mask_.battery_pack_efficiency ? 1 : 0;
    n += mask_.parasite_drag_area ? 1 : 0;
    return n;
}

std::vector<double> CalibrationProblem::pack(const CalibrationParameters& params) const {
    std::vector<double> x;
    if (mask_.figure_of_merit)         x.push_back(params.figure_of_merit);
    if (mask_.motor_efficiency)        x.push_back(params.motor_efficiency);
    if (mask_.esc_efficiency)          x.push_back(params.esc_efficiency);
    if (mask_.battery_specific_energy) x.push_back(params.battery_specific_energy_wh_per_kg);
    if (mask_.battery_pack_efficiency) x.push_back(params.battery_pack_efficiency);
    if (mask_.parasite_drag_area)      x.push_back(params.parasite_drag_area_m2);
    return x;
}

CalibrationParameters CalibrationProblem::unpack(
    const std::vector<double>& x,
    const CalibrationParameters& fixed) const {
    CalibrationParameters out = fixed;
    std::size_t i = 0;
    if (mask_.figure_of_merit)         out.figure_of_merit = x.at(i++);
    if (mask_.motor_efficiency)        out.motor_efficiency = x.at(i++);
    if (mask_.esc_efficiency)          out.esc_efficiency = x.at(i++);
    if (mask_.battery_specific_energy) out.battery_specific_energy_wh_per_kg = x.at(i++);
    if (mask_.battery_pack_efficiency) out.battery_pack_efficiency = x.at(i++);
    if (mask_.parasite_drag_area)      out.parasite_drag_area_m2 = x.at(i++);
    return out;
}

std::vector<double> CalibrationProblem::lowerActive() const {
    std::vector<double> out;
    if (mask_.figure_of_merit)         out.push_back(bounds_.lower[0]);
    if (mask_.motor_efficiency)        out.push_back(bounds_.lower[1]);
    if (mask_.esc_efficiency)          out.push_back(bounds_.lower[2]);
    if (mask_.battery_specific_energy) out.push_back(bounds_.lower[3]);
    if (mask_.battery_pack_efficiency) out.push_back(bounds_.lower[4]);
    if (mask_.parasite_drag_area)      out.push_back(bounds_.lower[5]);
    return out;
}

std::vector<double> CalibrationProblem::upperActive() const {
    std::vector<double> out;
    if (mask_.figure_of_merit)         out.push_back(bounds_.upper[0]);
    if (mask_.motor_efficiency)        out.push_back(bounds_.upper[1]);
    if (mask_.esc_efficiency)          out.push_back(bounds_.upper[2]);
    if (mask_.battery_specific_energy) out.push_back(bounds_.upper[3]);
    if (mask_.battery_pack_efficiency) out.push_back(bounds_.upper[4]);
    if (mask_.parasite_drag_area)      out.push_back(bounds_.upper[5]);
    return out;
}

std::vector<std::string> CalibrationProblem::activeNames() const {
    std::vector<std::string> out;
    if (mask_.figure_of_merit)         out.emplace_back("figure_of_merit");
    if (mask_.motor_efficiency)        out.emplace_back("motor_efficiency");
    if (mask_.esc_efficiency)          out.emplace_back("esc_efficiency");
    if (mask_.battery_specific_energy) out.emplace_back("battery_specific_energy_wh_per_kg");
    if (mask_.battery_pack_efficiency) out.emplace_back("battery_pack_efficiency");
    if (mask_.parasite_drag_area)      out.emplace_back("parasite_drag_area_m2");
    return out;
}

}  // namespace hexaarch::calibration
