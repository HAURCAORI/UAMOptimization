#pragma once

#include <string>

namespace hexaarch::physics {

struct Material {
    std::string name = "unknown";
    double density = 0.0;
    double yield_strength = 0.0;
    double elastic_modulus = 0.0;
    double shear_modulus = 0.0;   // G [Pa] — used for torsional stiffness GJ
};

namespace Materials {

inline Material Al7075() {
    // G = E / (2*(1+nu)), nu≈0.33 → G ≈ 26.9 GPa
    return {"Al7075", 2810.0, 503e6, 71.7e9, 26.9e9};
}

inline Material CFRP() {
    // Quasi-isotropic layup; G dominated by ±45° plies ≈ 5 GPa
    return {"CFRP", 1600.0, 600e6, 70.0e9, 5.0e9};
}

inline Material Steel304() {
    // G = E / (2*(1+nu)), nu≈0.27 → G ≈ 77 GPa
    return {"Steel304", 7900.0, 215e6, 193e9, 77.0e9};
}

}  // namespace Materials
}  // namespace hexaarch::physics
