#include "core/DefaultHexacopterBuilder.hpp"

#include <array>
#include <string>

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
    elements.push_back(std::make_unique<BodyElement>("body", parameters.Lx, parameters.Lyi, parameters.Lyo));
    elements.push_back(std::make_unique<BatteryElement>("battery", parameters.Tmax, parameters.dprop));
    elements.push_back(std::make_unique<PayloadElement>("payload", parameters.payload));

    for (int index = 0; index < 6; ++index) {
        elements.push_back(std::make_unique<ArmElement>(
            "arm_" + std::to_string(index + 1), index, parameters.Lx, parameters.Lyi, parameters.Lyo, parameters.Tmax));
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
    attachments.push_back({"body", "battery", "bottom", "mount", AttachmentRelationship::localOffset({0.0, 0.0, 0.02}), true, "central", AttachmentContactPolicy::bonded_overlap, nullptr});
    attachments.push_back({"body", "payload", "bottom", "center", AttachmentRelationship::localOffset({0.0, 0.0, -0.05}), true, "central", AttachmentContactPolicy::bonded_overlap, nullptr});

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
