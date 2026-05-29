#pragma once

#include <string>
#include <vector>

#include "mission/MissionProfile.hpp"
#include "physics/PhysicsTypes.hpp"

namespace hexaarch::evaluation { struct EvaluationContext; }

namespace hexaarch::mission {

// Per-segment energy accounting, returned together with totals so the CLI / CSV exporters can
// surface where energy is going (typical eVTOL pattern: cruise dominates time, hover dominates W).
struct SegmentResult {
    std::string label;
    SegmentKind kind = SegmentKind::hover;
    double duration_s = 0.0;
    double airspeed_mps = 0.0;
    double climb_rate_mps = 0.0;
    double electrical_power_w = 0.0;
    double energy_wh = 0.0;
    double distance_m = 0.0;
};

struct MissionResult {
    std::vector<SegmentResult> segments;
    double total_time_s = 0.0;
    double total_distance_m = 0.0;
    double cruise_distance_m = 0.0;          // only the cruise/climb/descent legs count toward range
    double total_energy_wh = 0.0;            // Σ P_i · t_i, electrical
    double total_energy_with_aux_wh = 0.0;   // + P_aux · t_total
    double peak_power_w = 0.0;               // max P_i across all segments (sized for C-rate)
    double hover_energy_wh = 0.0;
    double cruise_energy_wh = 0.0;
};

// Evaluates a multi-segment UAM mission against a given physical model + powertrain. Hover legs
// use the nominal- or worst-fault- hover trim thrust vector already computed by the upstream
// AttainableControlSetAnalyzer (so it inherits ACS-consistent fault modelling). Cruise/climb/
// descent legs use the CruisePowerModel.
class MissionEvaluator {
public:
    [[nodiscard]] MissionResult evaluate(
        const MissionProfile& profile,
        const physics::PowertrainResult& powertrain,
        const physics::PhysicalModel& model,
        double effective_disk_area_m2,
        const evaluation::EvaluationContext& context) const;
};

}  // namespace hexaarch::mission
