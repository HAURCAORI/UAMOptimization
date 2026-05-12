#include "optimization/DesignVectorMapper.hpp"

#include <stdexcept>

namespace hexaarch::optimization {

std::vector<double> DesignVectorMapper::pack(const core::HexacopterArchitecture& architecture) const {
    std::vector<double> packed;
    const auto active = architecture.parameters().activeParameters();
    packed.reserve(active.size());

    for (const auto* parameter : active) {
        packed.push_back(parameter->normalized());
    }

    return packed;
}

void DesignVectorMapper::unpackNormalized(
    core::HexacopterArchitecture& architecture,
    const std::vector<double>& normalized_values) const {
    auto active = architecture.parameters().activeParameters();
    if (active.size() != normalized_values.size()) {
        throw std::invalid_argument("DesignVectorMapper::unpackNormalized size mismatch.");
    }

    for (std::size_t index = 0; index < active.size(); ++index) {
        active.at(index)->setFromNormalized(normalized_values.at(index));
        active.at(index)->clamp();
    }

    architecture.updateFromParameters();
}

std::vector<std::string> DesignVectorMapper::parameterIds(const core::HexacopterArchitecture& architecture) const {
    std::vector<std::string> ids;
    const auto active = architecture.parameters().activeParameters();
    ids.reserve(active.size());

    for (const auto* parameter : active) {
        ids.push_back(parameter->stable_id());
    }

    return ids;
}

}  // namespace hexaarch::optimization
