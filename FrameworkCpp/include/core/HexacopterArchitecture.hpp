#pragma once

#include <string>

#include "core/ConstraintRegistry.hpp"
#include "core/ParameterRegistry.hpp"

namespace hexaarch::core {

class HexacopterArchitecture {
public:
    HexacopterArchitecture();

    [[nodiscard]] const std::string& id() const;
    [[nodiscard]] ParameterRegistry& parameters();
    [[nodiscard]] const ParameterRegistry& parameters() const;
    [[nodiscard]] ConstraintRegistry& constraints();
    [[nodiscard]] const ConstraintRegistry& constraints() const;

private:
    std::string id_;
    ParameterRegistry parameters_;
    ConstraintRegistry constraints_;
};

}  // namespace hexaarch::core
