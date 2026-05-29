#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include "calibration/FlightDataPoint.hpp"

namespace hexaarch::calibration {

// CSV layout (case-insensitive headers, missing columns default to 0):
//   mass_kg, airspeed_mps, climb_rate_mps, thrust_total_n, power_total_w, air_density_kg_m3, label
// Blank lines and lines starting with '#' are treated as comments. Whitespace around values is
// stripped. Numeric parse failures are reported and the offending row is skipped.
[[nodiscard]] std::optional<std::vector<FlightDataPoint>>
loadFlightDataCsv(const std::filesystem::path& path);

}  // namespace hexaarch::calibration
