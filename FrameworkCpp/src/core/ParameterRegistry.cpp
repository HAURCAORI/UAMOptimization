#include "core/ParameterRegistry.hpp"

#include <algorithm>
#include <memory>
#include <utility>

namespace hexaarch::core {

ParameterRegistry::ParameterRegistry(const ParameterRegistry& other) {
    for (const auto& parameter : other.parameters_) {
        parameters_.push_back(std::make_unique<DesignParameter>(*parameter));
    }
}

ParameterRegistry& ParameterRegistry::operator=(const ParameterRegistry& other) {
    if (this == &other) {
        return *this;
    }

    parameters_.clear();
    for (const auto& parameter : other.parameters_) {
        parameters_.push_back(std::make_unique<DesignParameter>(*parameter));
    }
    return *this;
}

DesignParameter& ParameterRegistry::add(DesignParameter parameter) {
    parameters_.push_back(std::make_unique<DesignParameter>(std::move(parameter)));
    return *parameters_.back();
}

void ParameterRegistry::clear() {
    parameters_.clear();
}

std::vector<const DesignParameter*> ParameterRegistry::parameterPointers() const {
    std::vector<const DesignParameter*> pointers;
    pointers.reserve(parameters_.size());
    for (const auto& parameter : parameters_) {
        pointers.push_back(parameter.get());
    }
    return pointers;
}

std::vector<DesignParameter*> ParameterRegistry::parameterPointers() {
    std::vector<DesignParameter*> pointers;
    pointers.reserve(parameters_.size());
    for (auto& parameter : parameters_) {
        pointers.push_back(parameter.get());
    }
    return pointers;
}

std::vector<const DesignParameter*> ParameterRegistry::activeParameters() const {
    std::vector<const DesignParameter*> active;
    active.reserve(parameters_.size());

    for (const auto& parameter : parameters_) {
        if (parameter->active) {
            active.push_back(parameter.get());
        }
    }

    return active;
}

std::vector<DesignParameter*> ParameterRegistry::activeParameters() {
    std::vector<DesignParameter*> active;
    active.reserve(parameters_.size());

    for (auto& parameter : parameters_) {
        if (parameter->active) {
            active.push_back(parameter.get());
        }
    }

    return active;
}

const DesignParameter* ParameterRegistry::find(const std::string_view stable_id) const {
    const auto it = std::find_if(parameters_.begin(), parameters_.end(), [&](const auto& parameter) {
        return parameter->stable_id() == stable_id;
    });

    return it == parameters_.end() ? nullptr : it->get();
}

DesignParameter* ParameterRegistry::find(const std::string_view stable_id) {
    const auto it = std::find_if(parameters_.begin(), parameters_.end(), [&](const auto& parameter) {
        return parameter->stable_id() == stable_id;
    });

    return it == parameters_.end() ? nullptr : it->get();
}

bool ParameterRegistry::allWithinBounds() const {
    return std::all_of(parameters_.begin(), parameters_.end(), [](const auto& parameter) {
        return parameter->isWithinBounds();
    });
}

}  // namespace hexaarch::core
