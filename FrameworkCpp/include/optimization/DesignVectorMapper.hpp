#pragma once

#include <string>
#include <vector>

#include "core/HexacopterArchitecture.hpp"

namespace hexaarch::optimization {

class DesignVectorMapper {
public:
    [[nodiscard]] std::vector<double> pack(const core::HexacopterArchitecture& architecture) const;
    void unpackNormalized(core::HexacopterArchitecture& architecture, const std::vector<double>& normalized_values) const;
    [[nodiscard]] std::vector<std::string> parameterIds(const core::HexacopterArchitecture& architecture) const;
};

}  // namespace hexaarch::optimization
