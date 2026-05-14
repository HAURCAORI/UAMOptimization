#pragma once

#include <string>

namespace hexaarch::physics {

struct Material {
    std::string name = "unknown";
    double density = 0.0;
    double yield_strength = 0.0;
    double elastic_modulus = 0.0;
};

namespace Materials {

inline Material Al7075() {
    return {"Al7075", 2810.0, 503e6, 71.7e9};
}

inline Material CFRP() {
    return {"CFRP", 1600.0, 600e6, 70.0e9};
}

inline Material Steel304() {
    return {"Steel304", 7900.0, 215e6, 193e9};
}

}  // namespace Materials
}  // namespace hexaarch::physics
