#include "core/HexacopterArchitecture.hpp"

namespace hexaarch::core {

HexacopterArchitecture::HexacopterArchitecture()
    : id_("default-architecture") {}

const std::string& HexacopterArchitecture::id() const {
    return id_;
}

ParameterRegistry& HexacopterArchitecture::parameters() {
    return parameters_;
}

const ParameterRegistry& HexacopterArchitecture::parameters() const {
    return parameters_;
}

ConstraintRegistry& HexacopterArchitecture::constraints() {
    return constraints_;
}

const ConstraintRegistry& HexacopterArchitecture::constraints() const {
    return constraints_;
}

}  // namespace hexaarch::core
