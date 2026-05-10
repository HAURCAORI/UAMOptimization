#pragma once

#include <vector>

#include "core/Constraint.hpp"

namespace hexaarch::core {

class ConstraintRegistry {
public:
    void add(Constraint constraint);
    [[nodiscard]] const std::vector<Constraint>& constraints() const;

private:
    std::vector<Constraint> constraints_;
};

}  // namespace hexaarch::core
