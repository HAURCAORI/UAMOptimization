#pragma once

namespace hexaarch::evaluation { struct EvaluationContext; }

namespace hexaarch::mission {

// Forward-flight (cruise/climb/descent) power model for a multi-rotor.
//
// Method: momentum-theory induced velocity in forward flight, plus airframe parasite drag,
// plus a m·g·v_climb potential term. All efficiency losses (FoM, motor, ESC) are applied at
// the end to convert mechanical disk power to electrical pack power, identical to
// PowertrainEvaluator's hover model so calibration can compare apples-to-apples.
//
// Inputs in physical units, output is electrical pack power [W].
struct CruiseQuery {
    double vehicle_mass_kg = 0.0;
    double airspeed_mps = 0.0;            // forward TAS
    double climb_rate_mps = 0.0;          // +up; m·g·v_z propulsive term
    double disk_area_m2 = 0.0;            // total disk area of all rotors at hover (n_rotors · π · r_eff²)
    double parasite_cd_a_m2 = 0.5;        // equivalent flat-plate drag area (Cd·A_ref) of airframe
    double air_density_kg_m3 = 1.225;     // ISA at altitude
};

struct CruiseResult {
    double induced_power_w = 0.0;
    double parasite_power_w = 0.0;
    double climb_power_w = 0.0;
    double mechanical_power_w = 0.0;
    double electrical_power_w = 0.0;
    double thrust_required_n = 0.0;
};

class CruisePowerModel {
public:
    [[nodiscard]] CruiseResult evaluate(const CruiseQuery& query,
                                        const evaluation::EvaluationContext& context) const;

    // Helper: induced velocity v_i from momentum theory in forward flight.
    // Solves T = 2·ρ·A·v_i·sqrt(V_h² + (V_v + v_i)²) where V_h is airspeed and V_v is climb rate.
    // Uses Newton iteration starting from hover v_i0 = sqrt(T/(2ρA)).
    [[nodiscard]] static double inducedVelocityForwardFlight(
        double thrust_n,
        double disk_area_m2,
        double airspeed_mps,
        double climb_rate_mps,
        double air_density_kg_m3);
};

}  // namespace hexaarch::mission
