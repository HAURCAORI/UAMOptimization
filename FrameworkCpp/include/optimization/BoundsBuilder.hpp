#pragma once

#include <vector>

#include "core/HexacopterArchitecture.hpp"

namespace hexaarch::optimization {

class BoundsBuilder {
public:
    [[nodiscard]] std::vector<double> lowerBounds(const core::HexacopterArchitecture& architecture) const {
        std::vector<double> bounds;
        const auto active = architecture.parameters().activeParameters();
        bounds.reserve(active.size());
        for (const auto* parameter : active) {
            bounds.push_back(parameter->normalizedAt(parameter->lower_bound));
        }
        return bounds;
    }

    [[nodiscard]] std::vector<double> upperBounds(const core::HexacopterArchitecture& architecture) const {
        std::vector<double> bounds;
        const auto active = architecture.parameters().activeParameters();
        bounds.reserve(active.size());
        for (const auto* parameter : active) {
            bounds.push_back(parameter->normalizedAt(parameter->upper_bound));
        }
        return bounds;
    }
};

}  // namespace hexaarch::optimization
