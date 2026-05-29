#pragma once

#include <string>

namespace hexaarch::calibration {

// One measured operating point from a real flight log. All values in SI units.
// thrust_total_n is the measured sum of motor thrusts at that instant (e.g. from a load cell or
// from inertial reconstruction). power_total_w is the measured electrical pack power (V·I).
// airspeed_mps and climb_rate_mps describe the flight condition; for static stand tests both
// are zero. mass_kg is the instantaneous vehicle mass (e.g., during fuel/battery drain or
// payload sweep). One row per logged sample; the calibrator weights each row equally.
struct FlightDataPoint {
    double mass_kg = 0.0;
    double airspeed_mps = 0.0;
    double climb_rate_mps = 0.0;
    double thrust_total_n = 0.0;
    double power_total_w = 0.0;
    double air_density_kg_m3 = 1.225;
    std::string label;
};

}  // namespace hexaarch::calibration
