#include "core/DefaultHexacopterBuilder.hpp"

#include <array>
#include <string>

#include "core/HexacopterArchitecture.hpp"

namespace hexaarch::core {
namespace {

constexpr std::array<double, 6> kDefaultRotorYawSigns{{-1.0, -1.0, 1.0, 1.0, -1.0, 1.0}};

Eigen::Vector3d motorPosition(const DefaultHexacopterParameters& parameters, const int index) {
    if (index == 2) {
        return Eigen::Vector3d(0.0, -parameters.Lyo->value, 0.0);
    }
    if (index == 3) {
        return Eigen::Vector3d(0.0, parameters.Lyo->value, 0.0);
    }

    switch (index) {
    case 0:
        return Eigen::Vector3d(parameters.Lx->value, -parameters.Lyi->value, 0.0);
    case 1:
        return Eigen::Vector3d(parameters.Lx->value, parameters.Lyi->value, 0.0);
    case 4:
        return Eigen::Vector3d(-parameters.Lx->value, -parameters.Lyi->value, 0.0);
    case 5:
        return Eigen::Vector3d(-parameters.Lx->value, parameters.Lyi->value, 0.0);
    default:
        return Eigen::Vector3d::Zero();
    }
}

Eigen::Isometry3d poseFromDirection(const Eigen::Vector3d& origin, const Eigen::Vector3d& direction) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = origin;
    if (direction.norm() > 1e-9) {
        pose.linear() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), direction.normalized()).toRotationMatrix();
    }
    return pose;
}

}  // namespace

std::vector<SpatialElementPtr> DefaultHexacopterBuilder::buildElements(const DefaultHexacopterParameters& parameters) {
    std::vector<SpatialElementPtr> elements;
    elements.push_back(std::make_unique<BodyHullElement>("body", parameters.Lx, parameters.Lyi, parameters.Lyo));
    elements.push_back(std::make_unique<BodyFrameElement>("body_frame"));
    elements.push_back(std::make_unique<BatteryElement>(
        "battery", parameters.m_bat,
        parameters.bat_x, parameters.bat_y, parameters.bat_z));
    elements.push_back(std::make_unique<PassengerElement>(
        "passenger", parameters.m_pax,
        parameters.pax_x, parameters.pax_y, parameters.pax_z));
    elements.push_back(std::make_unique<CargoElement>(
        "cargo", parameters.m_cargo,
        parameters.cargo_x, parameters.cargo_y, parameters.cargo_z));
    elements.push_back(std::make_unique<InstrumentPanelElement>(
        "instrument_panel", parameters.m_instrument));

    for (int index = 0; index < 6; ++index) {
        elements.push_back(std::make_unique<ArmElement>(
            "arm_" + std::to_string(index + 1), index, parameters.Lx, parameters.Lyi, parameters.Lyo, parameters.r_o, parameters.t_wall));
        elements.push_back(std::make_unique<MotorElement>(
            "motor_" + std::to_string(index + 1), index, parameters.Tmax));
        elements.push_back(std::make_unique<RotorElement>(
            "rotor_" + std::to_string(index + 1), index, kDefaultRotorYawSigns.at(static_cast<std::size_t>(index)), parameters.dprop));
    }

    return elements;
}

std::vector<Attachment> DefaultHexacopterBuilder::buildAttachments(const DefaultHexacopterParameters& parameters) {
    std::vector<Attachment> attachments;
    attachments.push_back({"", "body", "", "center", AttachmentRelationship::rigidMount(), true, "", AttachmentContactPolicy::bonded_overlap, [](const HexacopterArchitecture&) {
        return Eigen::Isometry3d::Identity();
    }});
    attachments.push_back({
        "body", "body_frame", "center", "center",
        AttachmentRelationship::rigidMount(), true, "",
        AttachmentContactPolicy::bonded_overlap, nullptr
    });
    // Battery / passenger / cargo / instrument: rigid mounts to body center.
    // Each element owns its placement variables and encodes them in local_pose_ so
    // the assembled world position = body_world_pose * local_pose_ (no attachment lambda needed).
    attachments.push_back({
        "body", "battery", "center", "mount",
        AttachmentRelationship::rigidMount(), true, "",
        AttachmentContactPolicy::bonded_overlap, nullptr
    });
    attachments.push_back({
        "body", "passenger", "center", "center",
        AttachmentRelationship::rigidMount(), true, "",
        AttachmentContactPolicy::bonded_overlap, nullptr
    });
    attachments.push_back({
        "body", "cargo", "center", "center",
        AttachmentRelationship::rigidMount(), true, "",
        AttachmentContactPolicy::bonded_overlap, nullptr
    });
    // Instrument panel fixed position is owned by InstrumentPanelElement::local_pose_.
    attachments.push_back({
        "body", "instrument_panel", "center", "center",
        AttachmentRelationship::rigidMount(), true, "",
        AttachmentContactPolicy::bonded_overlap, nullptr
    });

    for (int index = 0; index < 6; ++index) {
        const std::string slot = std::to_string(index + 1);
        const Eigen::Vector3d mount_position = motorPosition(parameters, index);
        const Eigen::Isometry3d arm_mount = poseFromDirection(Eigen::Vector3d::Zero(), mount_position);

        attachments.push_back({
            "body",
            "arm_" + slot,
            "center",
            "root",
            AttachmentRelationship::rigidMount(),
            true,
            "arm_group",
            AttachmentContactPolicy::bonded_overlap,
            [arm_mount](const HexacopterArchitecture&) {
                return arm_mount;
            }
        });
        attachments.push_back({
            "arm_" + slot,
            "motor_" + slot,
            "tip",
            "mount",
            AttachmentRelationship::rigidMount(),
            true,
            "motor_group",
            AttachmentContactPolicy::bonded_overlap,
            nullptr
        });
        attachments.push_back({
            "motor_" + slot,
            "rotor_" + slot,
            "axis",
            "axis",
            AttachmentRelationship::rigidMount(),
            true,
            "rotor_group",
            AttachmentContactPolicy::bonded_overlap,
            nullptr
        });
    }

    return attachments;
}

}  // namespace hexaarch::core
