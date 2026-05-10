#include "core/ConstraintRegistry.hpp"

#include <utility>

namespace hexaarch::core {

void ConstraintRegistry::add(Constraint constraint) {
    constraints_.push_back(std::move(constraint));
}

const std::vector<Constraint>& ConstraintRegistry::constraints() const {
    return constraints_;
}

}  // namespace hexaarch::core
