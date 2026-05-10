#include "core/ParameterRegistry.hpp"

#include <utility>

namespace hexaarch::core {

void ParameterRegistry::add(DesignParameter parameter) {
    parameters_.push_back(std::move(parameter));
}

const std::vector<DesignParameter>& ParameterRegistry::parameters() const {
    return parameters_;
}

}  // namespace hexaarch::core
