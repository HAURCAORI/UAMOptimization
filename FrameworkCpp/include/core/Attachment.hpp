#pragma once

#include <functional>
#include <string>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

namespace hexaarch::core {

class HexacopterArchitecture;

enum class AttachmentContactPolicy {
    enforce_clearance,
    allow_touch,
    bonded_overlap
};

enum class AttachmentRelationshipKind {
    rigid_mount,
    local_offset,
    mirrored_local_offset
};

struct AttachmentRelationship {
    AttachmentRelationshipKind kind = AttachmentRelationshipKind::rigid_mount;
    Eigen::Vector3d local_offset = Eigen::Vector3d::Zero();
    Eigen::Vector3d mirror_signs = Eigen::Vector3d::Ones();

    [[nodiscard]] static AttachmentRelationship rigidMount();
    [[nodiscard]] static AttachmentRelationship localOffset(const Eigen::Vector3d& offset);
    [[nodiscard]] static AttachmentRelationship mirroredLocalOffset(
        const Eigen::Vector3d& offset,
        const Eigen::Vector3d& mirror_signs);
    [[nodiscard]] Eigen::Isometry3d resolve() const;
};

struct Attachment {
    std::string parent_id;
    std::string child_id;
    std::string parent_anchor;
    std::string child_anchor;
    AttachmentRelationship relationship = AttachmentRelationship::rigidMount();
    bool enabled = true;
    std::string symmetry_tag;
    AttachmentContactPolicy contact_policy = AttachmentContactPolicy::enforce_clearance;
    std::function<Eigen::Isometry3d(const HexacopterArchitecture&)> relative_transform;
};

inline AttachmentRelationship AttachmentRelationship::rigidMount() {
    return {};
}

inline AttachmentRelationship AttachmentRelationship::localOffset(const Eigen::Vector3d& offset) {
    AttachmentRelationship relationship;
    relationship.kind = AttachmentRelationshipKind::local_offset;
    relationship.local_offset = offset;
    return relationship;
}

inline AttachmentRelationship AttachmentRelationship::mirroredLocalOffset(
    const Eigen::Vector3d& offset,
    const Eigen::Vector3d& mirror_signs) {
    AttachmentRelationship relationship;
    relationship.kind = AttachmentRelationshipKind::mirrored_local_offset;
    relationship.local_offset = offset;
    relationship.mirror_signs = mirror_signs;
    return relationship;
}

inline Eigen::Isometry3d AttachmentRelationship::resolve() const {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    switch (kind) {
    case AttachmentRelationshipKind::rigid_mount:
        return pose;
    case AttachmentRelationshipKind::local_offset:
        pose.translation() = local_offset;
        return pose;
    case AttachmentRelationshipKind::mirrored_local_offset:
        pose.translation() = local_offset.cwiseProduct(mirror_signs);
        return pose;
    }
    return pose;
}

}  // namespace hexaarch::core
