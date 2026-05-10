#pragma once

#include <string>

namespace hexaarch::core {

struct DesignParameter {
    std::string name;
    std::string owner_id;
    double value = 0.0;
    double lower_bound = 0.0;
    double upper_bound = 0.0;
    double default_value = 0.0;
    bool active = false;
    double scale = 1.0;
};

}  // namespace hexaarch::core
