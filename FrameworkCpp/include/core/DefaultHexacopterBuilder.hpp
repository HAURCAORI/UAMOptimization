#pragma once

#include <vector>

#include "core/Attachment.hpp"
#include "core/Elements.hpp"

namespace hexaarch::core {

struct DefaultHexacopterParameters {
    DesignParameter* Lx = nullptr;
    DesignParameter* Lyi = nullptr;
    DesignParameter* Lyo = nullptr;
    DesignParameter* Tmax = nullptr;
    DesignParameter* cT = nullptr;
    DesignParameter* dprop = nullptr;
    DesignParameter* payload = nullptr;
};

class DefaultHexacopterBuilder {
public:
    [[nodiscard]] static std::vector<SpatialElementPtr> buildElements(const DefaultHexacopterParameters& parameters);
    [[nodiscard]] static std::vector<Attachment> buildAttachments(
        const DefaultHexacopterParameters& parameters);
};

}  // namespace hexaarch::core
