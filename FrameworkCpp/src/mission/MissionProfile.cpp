#include "mission/MissionProfile.hpp"

#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"

namespace hexaarch::mission {

std::string segmentKindToString(SegmentKind kind) {
    switch (kind) {
        case SegmentKind::hover:           return "hover";
        case SegmentKind::cruise:          return "cruise";
        case SegmentKind::climb:           return "climb";
        case SegmentKind::descent:         return "descent";
        case SegmentKind::emergency_hover: return "emergency_hover";
        case SegmentKind::reserve_hover:   return "reserve_hover";
    }
    return "hover";
}

std::optional<SegmentKind> segmentKindFromString(const std::string& token) {
    if (token == "hover")           return SegmentKind::hover;
    if (token == "cruise")          return SegmentKind::cruise;
    if (token == "climb")           return SegmentKind::climb;
    if (token == "descent")         return SegmentKind::descent;
    if (token == "emergency_hover") return SegmentKind::emergency_hover;
    if (token == "reserve_hover")   return SegmentKind::reserve_hover;
    return std::nullopt;
}

std::optional<MissionProfile> loadMissionProfileJson(const std::filesystem::path& path) {
    std::ifstream stream(path);
    if (!stream) {
        std::cerr << "MissionProfile: cannot open " << path.string() << "\n";
        return std::nullopt;
    }
    const auto doc = nlohmann::json::parse(stream, nullptr, false);
    if (doc.is_discarded()) {
        std::cerr << "MissionProfile: invalid JSON in " << path.string() << "\n";
        return std::nullopt;
    }

    MissionProfile profile;
    if (doc.contains("name")) {
        profile.name = doc["name"].get<std::string>();
    }
    if (doc.contains("payload_mass_kg")) {
        profile.payload_mass_kg = doc["payload_mass_kg"].get<double>();
    }
    if (!doc.contains("segments") || !doc["segments"].is_array()) {
        std::cerr << "MissionProfile: 'segments' array missing in " << path.string() << "\n";
        return std::nullopt;
    }
    for (const auto& seg : doc["segments"]) {
        MissionSegment s;
        const std::string kind_token = seg.value("kind", std::string{"hover"});
        if (const auto kind = segmentKindFromString(kind_token)) {
            s.kind = *kind;
        } else {
            std::cerr << "MissionProfile: unknown segment kind '" << kind_token
                      << "' in " << path.string() << "\n";
            return std::nullopt;
        }
        s.label          = seg.value("label", std::string{});
        s.duration_s     = seg.value("duration_s", 0.0);
        s.airspeed_mps   = seg.value("airspeed_mps", 0.0);
        s.climb_rate_mps = seg.value("climb_rate_mps", 0.0);
        s.altitude_m     = seg.value("altitude_m", 0.0);
        s.worst_fault    = seg.value("worst_fault", s.kind == SegmentKind::emergency_hover);
        profile.segments.push_back(std::move(s));
    }
    return profile;
}

MissionProfile hoverOnlyLegacyProfile(double nominal_minutes, double emergency_minutes) {
    MissionProfile profile;
    profile.name = "legacy_hover_only";
    {
        MissionSegment s;
        s.kind = SegmentKind::hover;
        s.label = "nominal_hover";
        s.duration_s = nominal_minutes * 60.0;
        profile.segments.push_back(s);
    }
    {
        MissionSegment s;
        s.kind = SegmentKind::emergency_hover;
        s.label = "emergency_hover";
        s.duration_s = emergency_minutes * 60.0;
        s.worst_fault = true;
        profile.segments.push_back(s);
    }
    return profile;
}

}  // namespace hexaarch::mission
