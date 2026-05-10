#pragma once

#include <string>

namespace hexaarch::core {

struct Constraint {
    std::string name;
    std::string owner_id;
    bool hard = true;
    bool active = true;
    double penalty = 0.0;
};

}  // namespace hexaarch::core
