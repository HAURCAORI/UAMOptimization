#include "visualization/ArchitectureSceneBuilder.hpp"

#include <array>
#include <string_view>

namespace hexaarch::visualization {
namespace {

using PrimitiveKind = core::GeometryPrimitive::Kind;

std::array<float, 4> colorForElementType(const std::string_view element_type) {
    if (element_type == "BodyHullElement") {
        return {0.25f, 0.27f, 0.30f, 1.0f};
    }
    if (element_type == "BodyFrameElement") {
        return {0.18f, 0.20f, 0.22f, 1.0f};
    }
    if (element_type == "BatteryElement") {
        return {0.20f, 0.45f, 0.90f, 1.0f};
    }
    if (element_type == "PassengerElement") return {0.95f, 0.60f, 0.20f, 1.0f};  // amber
    if (element_type == "CargoElement")     return {0.55f, 0.40f, 0.20f, 1.0f};  // brown
    if (element_type == "InstrumentPanelElement") return {0.25f, 0.25f, 0.35f, 1.0f};  // dark slate
    if (element_type == "ArmElement") {
        return {0.55f, 0.58f, 0.62f, 1.0f};
    }
    if (element_type == "MotorElement") {
        return {0.55f, 0.12f, 0.12f, 1.0f};
    }
    if (element_type == "RotorElement") {
        return {0.20f, 0.70f, 0.35f, 1.0f};
    }
    if (element_type == "CabinEnvelopeElement") {
        return {0.30f, 0.70f, 0.90f, 0.35f};
    }
    return {0.75f, 0.75f, 0.75f, 1.0f};
}

bool defaultWireframeForPrimitive(const PrimitiveKind kind) {
    return kind == PrimitiveKind::disk;
}

}  // namespace

std::vector<PrimitiveInstance> ArchitectureSceneBuilder::build(
    const core::HexacopterArchitecture& architecture) const {
    std::vector<PrimitiveInstance> instances;
    const auto& assembled_elements = architecture.assemblyState().elements;

    std::size_t primitive_count = 0;
    for (const auto& assembled : assembled_elements) {
        primitive_count += assembled.local_primitives.size();
    }
    instances.reserve(primitive_count);

    for (const auto& assembled : assembled_elements) {
        if (assembled.element == nullptr) {
            continue;
        }

        const std::string element_id = assembled.element->id();
        const std::string element_type = assembled.element->type();
        const auto color = colorForElementType(element_type);

        for (const auto& primitive : assembled.local_primitives) {
            PrimitiveInstance instance;
            instance.primitive_type = primitive.kind;
            instance.world_transform = assembled.world_pose * primitive.local_pose;
            instance.dimensions = primitive.dimensions;
            instance.padding = primitive.padding;
            instance.color = color;
            instance.source_element_id = element_id;
            instance.source_element_type = element_type;
            instance.wireframe = defaultWireframeForPrimitive(primitive.kind);
            // Envelope / hull elements render as wireframe so internal components are visible.
            if (element_type == "CabinEnvelopeElement")    instance.wireframe = true;
            if (element_type == "BodyHullElement")          instance.wireframe = true;
            if (element_type == "OccupantEnvelopeElement") instance.wireframe = true;
            instances.push_back(std::move(instance));
        }
    }

    return instances;
}

}  // namespace hexaarch::visualization
