#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace hexaarch::mission {

// One leg of a multi-segment UAM mission. Hover-style legs leave airspeed_mps == 0;
// cruise/climb/descent legs use airspeed_mps and (for climb/descent) climb_rate_mps.
enum class SegmentKind {
    hover,
    cruise,
    climb,
    descent,
    emergency_hover,  // worst-fault hover sized to mission_time_emergency_min
    reserve_hover     // contingency hover energy added on top of regular legs
};

struct MissionSegment {
    SegmentKind kind = SegmentKind::hover;
    std::string label;
    double duration_s = 0.0;
    double airspeed_mps = 0.0;       // forward airspeed (TAS) [m/s]
    double climb_rate_mps = 0.0;     // positive = climbing, negative = descending [m/s]
    double altitude_m = 0.0;         // segment start altitude (used for air-density variation)
    bool worst_fault = false;        // true ⇒ size with worst-fault thrust vector
};

struct MissionProfile {
    std::string name;
    std::vector<MissionSegment> segments;
    double payload_mass_kg = 0.0;    // optional: payload deviation for this mission (≥0 ⇒ override)

    [[nodiscard]] bool empty() const { return segments.empty(); }
};

// Load a MissionProfile from a JSON file. Expected schema:
//   {
//     "name": "city_taxi_round_trip",
//     "payload_mass_kg": 600,
//     "segments": [
//       {"kind":"hover","label":"takeoff","duration_s":30},
//       {"kind":"climb","duration_s":40,"airspeed_mps":15,"climb_rate_mps":4},
//       {"kind":"cruise","duration_s":480,"airspeed_mps":40,"altitude_m":300},
//       {"kind":"descent","duration_s":60,"airspeed_mps":15,"climb_rate_mps":-3},
//       {"kind":"hover","label":"landing","duration_s":30},
//       {"kind":"reserve_hover","label":"reserve","duration_s":120},
//       {"kind":"emergency_hover","label":"OEI","duration_s":60,"worst_fault":true}
//     ]
//   }
// On parse failure returns std::nullopt; the optional return makes the caller responsible for
// reporting the failure with the source path.
std::optional<MissionProfile> loadMissionProfileJson(const std::filesystem::path& path);

// Built-in profile mirroring the legacy `EvaluationContext.mission_time_nominal_min` +
// `mission_time_emergency_min` behaviour, used when no JSON is supplied. Lets callers compare
// the multi-segment evaluator output against the legacy hover-only energy budget.
[[nodiscard]] MissionProfile hoverOnlyLegacyProfile(double nominal_minutes, double emergency_minutes);

[[nodiscard]] std::string segmentKindToString(SegmentKind kind);
[[nodiscard]] std::optional<SegmentKind> segmentKindFromString(const std::string& token);

}  // namespace hexaarch::mission
