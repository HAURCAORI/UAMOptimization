#pragma once

#include <memory>
#include <string_view>
#include <vector>

#include "core/DesignParameter.hpp"

namespace hexaarch::core {

class ParameterRegistry {
public:
    ParameterRegistry() = default;
    ParameterRegistry(const ParameterRegistry& other);
    ParameterRegistry& operator=(const ParameterRegistry& other);
    ParameterRegistry(ParameterRegistry&&) noexcept = default;
    ParameterRegistry& operator=(ParameterRegistry&&) noexcept = default;

    DesignParameter& add(DesignParameter parameter);
    void clear();

    [[nodiscard]] std::vector<const DesignParameter*> parameterPointers() const;
    [[nodiscard]] std::vector<DesignParameter*> parameterPointers();
    [[nodiscard]] std::vector<const DesignParameter*> activeParameters() const;
    [[nodiscard]] std::vector<DesignParameter*> activeParameters();
    [[nodiscard]] const DesignParameter* find(std::string_view stable_id) const;
    [[nodiscard]] DesignParameter* find(std::string_view stable_id);
    [[nodiscard]] bool allWithinBounds() const;

private:
    std::vector<std::unique_ptr<DesignParameter>> parameters_;
};

}  // namespace hexaarch::core
