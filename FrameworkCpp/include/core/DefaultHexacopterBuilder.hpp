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
    DesignParameter* m_pax = nullptr;
    DesignParameter* m_cargo = nullptr;
    DesignParameter* m_instrument = nullptr;
    DesignParameter* r_o = nullptr;
    DesignParameter* t_wall = nullptr;
    DesignParameter* m_bat = nullptr;
    // Placement DOFs — passed directly to elements so they own their position variables.
    DesignParameter* pax_x = nullptr;
    DesignParameter* pax_y = nullptr;
    DesignParameter* pax_z = nullptr;
    DesignParameter* cargo_x = nullptr;
    DesignParameter* cargo_y = nullptr;
    DesignParameter* cargo_z = nullptr;
    DesignParameter* bat_x = nullptr;
    DesignParameter* bat_y = nullptr;
    DesignParameter* bat_z = nullptr;
};

class DefaultHexacopterBuilder {
public:
    [[nodiscard]] static std::vector<SpatialElementPtr> buildElements(const DefaultHexacopterParameters& parameters);
    [[nodiscard]] static std::vector<Attachment> buildAttachments(
        const DefaultHexacopterParameters& parameters);
};

}  // namespace hexaarch::core
