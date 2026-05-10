#pragma once

#include <vector>

#include "core/DesignParameter.hpp"

namespace hexaarch::core {

class ParameterRegistry {
public:
    void add(DesignParameter parameter);
    [[nodiscard]] const std::vector<DesignParameter>& parameters() const;

private:
    std::vector<DesignParameter> parameters_;
};

}  // namespace hexaarch::core
