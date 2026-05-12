#include "core/AssemblyState.hpp"

#include "core/SpatialElement.hpp"

namespace hexaarch::core {

const AssembledElement* AssemblyState::find(const std::string_view element_id) const {
    for (const auto& element : elements) {
        if (element.element != nullptr && element.element->id() == element_id) {
            return &element;
        }
    }
    return nullptr;
}

}  // namespace hexaarch::core
