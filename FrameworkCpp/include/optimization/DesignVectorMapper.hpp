#pragma once

#include <vector>

#include "core/HexacopterArchitecture.hpp"

namespace hexaarch::optimization {

class DesignVectorMapper {
public:
    [[nodiscard]] std::vector<double> pack(const core::HexacopterArchitecture& architecture) const;
};

}  // namespace hexaarch::optimization
