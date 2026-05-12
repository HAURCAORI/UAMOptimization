#pragma once

#include <string_view>
#include <vector>

#include "core/Constraint.hpp"

namespace hexaarch::core {

class ConstraintRegistry {
public:
    Constraint& add(Constraint constraint);
    void clear();
    bool remove(std::string_view stable_id);
    std::size_t removeByOwner(std::string_view owner_id);

    [[nodiscard]] const std::vector<Constraint>& constraints() const;
    [[nodiscard]] const Constraint* find(std::string_view stable_id) const;

private:
    std::vector<Constraint> constraints_;
};

}  // namespace hexaarch::core
