#pragma once

#include <vector>

#include "core/HexacopterArchitecture.hpp"
#include "visualization/PrimitiveInstance.hpp"

namespace hexaarch::visualization {

class ArchitectureSceneBuilder {
public:
    [[nodiscard]] std::vector<PrimitiveInstance> build(
        const core::HexacopterArchitecture& architecture) const;
};

}  // namespace hexaarch::visualization
