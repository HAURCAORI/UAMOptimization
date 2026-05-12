#pragma once

#include <string>
#include <string_view>
#include <unordered_set>

namespace hexaarch::core {

struct DesignParameter {
    std::string name;
    std::string owner_id;
    std::string unit;
    std::string description;
    double value = 0.0;
    double lower_bound = 0.0;
    double upper_bound = 0.0;
    double default_value = 0.0;
    bool active = false;
    double scale = 1.0;
    std::unordered_set<std::string> consumer_ids;

    [[nodiscard]] std::string stable_id() const;
    [[nodiscard]] double normalized() const;
    [[nodiscard]] double normalizedAt(double raw_value) const;
    void setFromNormalized(double normalized_value);
    [[nodiscard]] bool isWithinBounds() const;
    void clamp();
    void addConsumer(std::string consumer_id);
    [[nodiscard]] bool isConsumedBy(std::string_view consumer_id) const;
};

}  // namespace hexaarch::core
