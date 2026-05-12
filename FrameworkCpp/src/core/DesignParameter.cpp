#include "core/DesignParameter.hpp"

#include <algorithm>

namespace hexaarch::core {

std::string DesignParameter::stable_id() const {
    return owner_id + "::" + name;
}

double DesignParameter::normalized() const {
    const double span = upper_bound - lower_bound;
    if (span <= 0.0 || scale == 0.0) {
        return value;
    }

    return ((value - lower_bound) / span) * scale;
}

double DesignParameter::normalizedAt(const double raw_value) const {
    const double span = upper_bound - lower_bound;
    if (span <= 0.0 || scale == 0.0) {
        return raw_value;
    }

    return ((raw_value - lower_bound) / span) * scale;
}

void DesignParameter::setFromNormalized(const double normalized_value) {
    const double span = upper_bound - lower_bound;
    if (span <= 0.0 || scale == 0.0) {
        value = normalized_value;
        return;
    }

    value = lower_bound + span * (normalized_value / scale);
}

bool DesignParameter::isWithinBounds() const {
    return value >= lower_bound && value <= upper_bound;
}

void DesignParameter::clamp() {
    if (lower_bound > upper_bound) {
        return;
    }

    value = std::clamp(value, lower_bound, upper_bound);
}

void DesignParameter::addConsumer(std::string consumer_id) {
    consumer_ids.insert(std::move(consumer_id));
}

bool DesignParameter::isConsumedBy(const std::string_view consumer_id) const {
    return consumer_ids.count(std::string(consumer_id)) > 0U;
}

}  // namespace hexaarch::core
