#include "mission/MissionEvaluator.hpp"

#include <algorithm>
#include <cmath>

#include "evaluation/EvaluationContext.hpp"
#include "mission/CruisePowerModel.hpp"

namespace hexaarch::mission {

MissionResult MissionEvaluator::evaluate(
    const MissionProfile& profile,
    const physics::PowertrainResult& powertrain,
    const physics::PhysicalModel& model,
    const double effective_disk_area_m2,
    const evaluation::EvaluationContext& context) const {

    MissionResult result;
    if (profile.empty()) {
        return result;
    }

    // Total disk area is per-rotor area × number of rotors; PowertrainEvaluator uses single-rotor
    // r_eff = max_arm_length/2, so we multiply by kNumRotors here.
    const double disk_area_total =
        effective_disk_area_m2 * static_cast<double>(physics::kNumRotors);

    const CruisePowerModel cruise;
    const double vehicle_mass = model.mass_properties.mass;

    for (const auto& seg : profile.segments) {
        SegmentResult sr;
        sr.label          = seg.label.empty() ? segmentKindToString(seg.kind) : seg.label;
        sr.kind           = seg.kind;
        sr.duration_s     = std::max(seg.duration_s, 0.0);
        sr.airspeed_mps   = seg.airspeed_mps;
        sr.climb_rate_mps = seg.climb_rate_mps;

        switch (seg.kind) {
            case SegmentKind::hover:
            case SegmentKind::reserve_hover:
                sr.electrical_power_w = powertrain.total_power_nominal_w;
                break;
            case SegmentKind::emergency_hover:
                sr.electrical_power_w = powertrain.total_power_faulted_w;
                break;
            case SegmentKind::cruise:
            case SegmentKind::climb:
            case SegmentKind::descent: {
                CruiseQuery q;
                q.vehicle_mass_kg    = vehicle_mass;
                q.airspeed_mps       = seg.airspeed_mps;
                q.climb_rate_mps     = seg.climb_rate_mps;
                q.disk_area_m2       = disk_area_total;
                q.parasite_cd_a_m2   = context.parasite_drag_area_m2;
                q.air_density_kg_m3  = context.air_density;
                const CruiseResult cr = cruise.evaluate(q, context);
                sr.electrical_power_w = cr.electrical_power_w;
                break;
            }
        }

        sr.energy_wh = sr.electrical_power_w * (sr.duration_s / 3600.0);
        sr.distance_m = std::max(seg.airspeed_mps, 0.0) * sr.duration_s;

        result.total_time_s    += sr.duration_s;
        result.total_energy_wh += sr.energy_wh;
        result.peak_power_w     = std::max(result.peak_power_w, sr.electrical_power_w);
        result.total_distance_m += sr.distance_m;

        const bool is_cruise_like =
            seg.kind == SegmentKind::cruise ||
            seg.kind == SegmentKind::climb ||
            seg.kind == SegmentKind::descent;
        if (is_cruise_like) {
            result.cruise_distance_m += sr.distance_m;
            result.cruise_energy_wh += sr.energy_wh;
        } else {
            result.hover_energy_wh += sr.energy_wh;
        }
        result.segments.push_back(std::move(sr));
    }

    // Auxiliary draw runs continuously and adds to the energy budget but not to per-segment
    // propulsion bookkeeping above (keeps the propulsion column comparable across segments).
    result.total_energy_with_aux_wh =
        result.total_energy_wh + context.power_auxiliary_w * (result.total_time_s / 3600.0);

    return result;
}

}  // namespace hexaarch::mission
