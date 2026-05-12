#include "core/ConstraintRegistry.hpp"

#include <algorithm>
#include <utility>

namespace hexaarch::core {

Constraint& ConstraintRegistry::add(Constraint constraint) {
    constraints_.push_back(std::move(constraint));
    return constraints_.back();
}

void ConstraintRegistry::clear() {
    constraints_.clear();
}

bool ConstraintRegistry::remove(const std::string_view stable_id) {
    const auto previous_size = constraints_.size();
    constraints_.erase(
        std::remove_if(constraints_.begin(), constraints_.end(), [&](const Constraint& constraint) {
            return constraint.stable_id() == stable_id;
        }),
        constraints_.end());
    return constraints_.size() != previous_size;
}

std::size_t ConstraintRegistry::removeByOwner(const std::string_view owner_id) {
    const auto previous_size = constraints_.size();
    constraints_.erase(
        std::remove_if(constraints_.begin(), constraints_.end(), [&](const Constraint& constraint) {
            return constraint.owner_id == owner_id;
        }),
        constraints_.end());
    return previous_size - constraints_.size();
}

const std::vector<Constraint>& ConstraintRegistry::constraints() const {
    return constraints_;
}

const Constraint* ConstraintRegistry::find(const std::string_view stable_id) const {
    const auto it = std::find_if(constraints_.begin(), constraints_.end(), [&](const Constraint& constraint) {
        return constraint.stable_id() == stable_id;
    });

    return it == constraints_.end() ? nullptr : &(*it);
}

}  // namespace hexaarch::core
