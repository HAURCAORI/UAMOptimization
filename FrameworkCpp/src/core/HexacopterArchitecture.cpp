#include "core/HexacopterArchitecture.hpp"

#include <algorithm>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <string>
#include <unordered_map>
#include <utility>

#include "evaluation/EvaluationContext.hpp"
#include "evaluation/EvaluationResult.hpp"

namespace hexaarch::core {
constexpr double kBaselineLx = 2.65;
constexpr double kBaselineLyi = 2.65;
constexpr double kBaselineLyo = 5.50;
constexpr double kBaselineCT = 0.03;
constexpr double kBaselinePropDiameter = 0.40;
constexpr double kBaselinePayload = 1500.0;
constexpr double kBaselineTmax = 2240.73 * 2.0 / 6.0 * 9.81;

std::string canonicalPairKey(const std::string_view lhs_id, const std::string_view rhs_id) {
    if (lhs_id <= rhs_id) {
        return std::string(lhs_id) + "||" + std::string(rhs_id);
    }
    return std::string(rhs_id) + "||" + std::string(lhs_id);
}

HexacopterArchitecture::HexacopterArchitecture()
    : id_("default-architecture") {
    registerDefaultParameters();
    bindCanonicalParameters();
    rebuildElements();
    registerDefaultConstraints();
    registerElementConstraints();
    rebuildAttachments();
    assemble();
}

HexacopterArchitecture::HexacopterArchitecture(EmptyTag)
    : id_("custom-architecture") {
    registerDefaultParameters();
    bindCanonicalParameters();
}

HexacopterArchitecture HexacopterArchitecture::createEmpty() {
    return HexacopterArchitecture{EmptyTag{}};
}

HexacopterArchitecture::HexacopterArchitecture(const HexacopterArchitecture& other)
    : id_(other.id_),
      nominal_mass_(other.nominal_mass_),
      use_vehicle_model_(other.use_vehicle_model_),
      gravity_(other.gravity_),
      parameters_(other.parameters_),
      constraints_(other.constraints_),
      attachments_(other.attachments_) {
    bindCanonicalParameters();
    elements_.reserve(other.elements_.size());
    for (const auto& element : other.elements_) {
        auto clone = element->clone();
        clone->rebindParameters(parameters_);
        clone->updateFromParameters();
        elements_.push_back(std::move(clone));
    }
    assemble();
}

HexacopterArchitecture& HexacopterArchitecture::operator=(const HexacopterArchitecture& other) {
    if (this == &other) {
        return *this;
    }

    id_ = other.id_;
    nominal_mass_ = other.nominal_mass_;
    use_vehicle_model_ = other.use_vehicle_model_;
    gravity_ = other.gravity_;
    parameters_ = other.parameters_;
    constraints_ = other.constraints_;
    attachments_ = other.attachments_;
    bindCanonicalParameters();
    elements_.clear();
    elements_.reserve(other.elements_.size());
    for (const auto& element : other.elements_) {
        auto clone = element->clone();
        clone->rebindParameters(parameters_);
        clone->updateFromParameters();
        elements_.push_back(std::move(clone));
    }
    assemble();
    return *this;
}

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

const std::vector<SpatialElementPtr>& HexacopterArchitecture::elements() const {
    return elements_;
}

const std::vector<Attachment>& HexacopterArchitecture::attachments() const {
    return attachments_;
}

const AssemblyState& HexacopterArchitecture::assemblyState() const {
    return assembly_state_;
}

const SpatialElement* HexacopterArchitecture::findElement(const std::string_view element_id) const {
    const auto it = std::find_if(elements_.begin(), elements_.end(), [&](const auto& element) {
        return element->id() == element_id;
    });
    return it == elements_.end() ? nullptr : it->get();
}

bool HexacopterArchitecture::isPackagingPairExempt(const std::string_view lhs_id, const std::string_view rhs_id) const {
    return packaging_exemptions_.contains(packagingPairKey(lhs_id, rhs_id));
}

void HexacopterArchitecture::addElement(SpatialElementPtr element) {
    if (element == nullptr) {
        throw std::invalid_argument("Cannot add null element.");
    }
    if (findElement(element->id()) != nullptr) {
        throw std::invalid_argument("Element already exists: " + element->id());
    }
    element->rebindParameters(parameters_);
    element->registerParameters(parameters_);
    element->updateFromParameters();
    constraints_.removeByOwner(element->id());
    element->registerConstraints(constraints_);
    elements_.push_back(std::move(element));
}

bool HexacopterArchitecture::removeElement(const std::string_view element_id) {
    const auto previous_size = elements_.size();
    elements_.erase(
        std::remove_if(elements_.begin(), elements_.end(), [&](const auto& element) {
            return element->id() == element_id;
        }),
        elements_.end());
    if (elements_.size() == previous_size) {
        return false;
    }
    constraints_.removeByOwner(element_id);
    attachments_.erase(
        std::remove_if(attachments_.begin(), attachments_.end(), [&](const Attachment& attachment) {
            return attachment.parent_id == element_id || attachment.child_id == element_id;
        }),
        attachments_.end());
    assemble();
    return true;
}

void HexacopterArchitecture::clearElements() {
    for (const auto& element : elements_) {
        constraints_.removeByOwner(element->id());
    }
    elements_.clear();
    attachments_.clear();
    packaging_exemptions_.clear();
    assembly_state_.elements.clear();
}

void HexacopterArchitecture::addAttachment(Attachment attachment) {
    attachments_.push_back(std::move(attachment));
}

bool HexacopterArchitecture::removeAttachment(const std::string_view child_id) {
    const auto previous_size = attachments_.size();
    attachments_.erase(
        std::remove_if(attachments_.begin(), attachments_.end(), [&](const Attachment& attachment) {
            return attachment.child_id == child_id;
        }),
        attachments_.end());
    if (attachments_.size() == previous_size) {
        return false;
    }
    assemble();
    return true;
}

void HexacopterArchitecture::clearAttachments() {
    attachments_.clear();
    packaging_exemptions_.clear();
    assembly_state_.elements.clear();
}

void HexacopterArchitecture::addConstraint(Constraint constraint) {
    constraints_.add(std::move(constraint));
}

bool HexacopterArchitecture::removeConstraint(const std::string_view stable_id) {
    return constraints_.remove(stable_id);
}

void HexacopterArchitecture::rebuildAssembly() {
    for (auto& element : elements_) {
        element->updateFromParameters();
    }
    assemble();
}

void HexacopterArchitecture::updateFromParameters() {
    for (auto& element : elements_) {
        element->updateFromParameters();
    }
    assemble();
}

double HexacopterArchitecture::Lx() const {
    return Lx_parameter_->value;
}

double HexacopterArchitecture::Lyi() const {
    return Lyi_parameter_->value;
}

double HexacopterArchitecture::Lyo() const {
    return Lyo_parameter_->value;
}

double HexacopterArchitecture::Tmax() const {
    return Tmax_parameter_->value;
}

double HexacopterArchitecture::cT() const {
    return cT_parameter_->value;
}

double HexacopterArchitecture::propellerDiameter() const {
    return dprop_parameter_->value;
}

double HexacopterArchitecture::payloadMass() const {
    return payload_parameter_->value;
}

bool HexacopterArchitecture::useVehicleModel() const {
    return use_vehicle_model_;
}

double HexacopterArchitecture::nominalMass() const {
    return nominal_mass_;
}

double HexacopterArchitecture::gravity() const {
    return gravity_;
}

std::string HexacopterArchitecture::packagingPairKey(const std::string_view lhs_id, const std::string_view rhs_id) {
    return canonicalPairKey(lhs_id, rhs_id);
}

void HexacopterArchitecture::registerDefaultParameters() {
    parameters_.clear();

    parameters_.add({"Lx", id_, "m", "Fore/aft arm length", kBaselineLx, 1.0, 5.0, kBaselineLx, true, 1.0});
    parameters_.add({"Lyi", id_, "m", "Inner lateral arm length", kBaselineLyi, 1.0, 5.0, kBaselineLyi, true, 1.0});
    parameters_.add({"Lyo", id_, "m", "Outer lateral arm length", kBaselineLyo, 2.5, 9.0, kBaselineLyo, true, 1.0});
    parameters_.add({"T_max", id_, "N", "Maximum thrust per motor", kBaselineTmax, 8000.0, 16000.0, kBaselineTmax, true, 1.0});
    parameters_.add({"cT", id_, "-", "Moment to thrust ratio", kBaselineCT, 0.01, 0.08, kBaselineCT, false, 1.0});
    parameters_.add({"d_prop", id_, "m", "Propeller diameter", kBaselinePropDiameter, 0.20, 1.20, kBaselinePropDiameter, false, 1.0});
    parameters_.add({"m_payload", id_, "kg", "Payload mass", kBaselinePayload, 1000.0, 2500.0, kBaselinePayload, false, 1.0});
    parameters_.add({"arm_outer_radius", id_, "m", "Arm tube outer radius", 0.08, 0.02, 0.15, 0.08, true, 1.0});
    parameters_.add({"arm_wall_thickness", id_, "m", "Arm tube wall thickness", 0.005, 0.001, 0.020, 0.005, true, 1.0});
}

void HexacopterArchitecture::registerDefaultConstraints() {
    constraints_.clear();

    constraints_.add({
        "parameter_bounds",
        id_,
        ConstraintSense::less_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            double worst = 0.0;
            for (const auto* parameter : context.architecture.parameters().parameterPointers()) {
                if (parameter->value < parameter->lower_bound) {
                    worst = std::max(worst, parameter->lower_bound - parameter->value);
                }
                if (parameter->value > parameter->upper_bound) {
                    worst = std::max(worst, parameter->value - parameter->upper_bound);
                }
            }
            Constraint constraint{"parameter_bounds", context.architecture.id(), ConstraintSense::less_equal, 0.0};
            return constraint.evaluate(worst);
        }
    });
    constraints_.add({
        "minimum_geometry_margin",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            const double value = std::min({
                context.architecture.Lx() - context.evaluation_context.minimum_arm_length,
                context.architecture.Lyi() - context.evaluation_context.minimum_arm_length,
                context.architecture.Lyo() - (context.architecture.Lyi() + context.evaluation_context.minimum_outer_arm_delta)
            });
            Constraint constraint{"minimum_geometry_margin", context.architecture.id(), ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(value);
        }
    });
    constraints_.add({
        "rotor_clearance",
        id_,
        ConstraintSense::greater_equal,
        0.0,
        true,
        true,
        1000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"rotor_clearance", context.architecture.id(), ConstraintSense::greater_equal, 0.0};
            return constraint.evaluate(context.physical_model.packaging.minimum_clearance);
        }
    });
    constraints_.add({
        "failed_hover_gamma",
        id_,
        ConstraintSense::greater_equal,
        1.5,
        true,
        true,
        2000.0,
        [](const ConstraintEvaluationContext& context) {
            Constraint constraint{"failed_hover_gamma", context.architecture.id(), ConstraintSense::greater_equal, 1.5};
            return constraint.evaluate(context.stage1_metrics.gamma_worst);
        }
    });
    constraints_.add({
        "fault_allocation_ratio",
        id_,
        ConstraintSense::greater_equal,
        0.05,
        true,
        true,
        1500.0,
        [](const ConstraintEvaluationContext& context) {
            const double ratio = context.stage1_metrics.sigma_worst / std::max(context.stage1_metrics.sigma_reference, 1e-9);
            Constraint constraint{"fault_allocation_ratio", context.architecture.id(), ConstraintSense::greater_equal, 0.05};
            return constraint.evaluate(ratio);
        }
    });
    constraints_.add({
        "arm_yield_failure",
        id_,
        ConstraintSense::greater_equal,
        1.5,
        true,
        true,
        2000.0,
        [](const ConstraintEvaluationContext& context) {
            const double min_sf = context.physical_model.structural.min_safety_factor;
            const double threshold = context.evaluation_context.minimum_arm_safety_factor;
            Constraint constraint{"arm_yield_failure", context.architecture.id(), ConstraintSense::greater_equal, threshold};
            return constraint.evaluate(min_sf);
        }
    });
}

void HexacopterArchitecture::registerElementConstraints() {
    for (const auto& element : elements_) {
        element->registerConstraints(constraints_);
    }
}

void HexacopterArchitecture::bindCanonicalParameters() {
    Lx_parameter_ = parameters_.find(id_ + "::Lx");
    Lyi_parameter_ = parameters_.find(id_ + "::Lyi");
    Lyo_parameter_ = parameters_.find(id_ + "::Lyo");
    Tmax_parameter_ = parameters_.find(id_ + "::T_max");
    cT_parameter_ = parameters_.find(id_ + "::cT");
    dprop_parameter_ = parameters_.find(id_ + "::d_prop");
    payload_parameter_ = parameters_.find(id_ + "::m_payload");
    r_o_parameter_ = parameters_.find(id_ + "::arm_outer_radius");
    t_wall_parameter_ = parameters_.find(id_ + "::arm_wall_thickness");
}

void HexacopterArchitecture::rebuildElements() {
    elements_ = DefaultHexacopterBuilder::buildElements({
        Lx_parameter_,
        Lyi_parameter_,
        Lyo_parameter_,
        Tmax_parameter_,
        cT_parameter_,
        dprop_parameter_,
        payload_parameter_,
        r_o_parameter_,
        t_wall_parameter_
    });

    for (auto& element : elements_) {
        element->registerParameters(parameters_);
        element->updateFromParameters();
    }
}

void HexacopterArchitecture::rebuildAttachments() {
    attachments_ = DefaultHexacopterBuilder::buildAttachments({
        Lx_parameter_,
        Lyi_parameter_,
        Lyo_parameter_,
        Tmax_parameter_,
        cT_parameter_,
        dprop_parameter_,
        payload_parameter_,
        r_o_parameter_,
        t_wall_parameter_
    });
}

void HexacopterArchitecture::assemble() {
    assembly_state_.elements.clear();
    packaging_exemptions_.clear();

    std::unordered_map<std::string, const SpatialElement*> element_by_id;
    for (const auto& element : elements_) {
        element_by_id.emplace(element->id(), element.get());
    }

    std::unordered_map<std::string, std::vector<const Attachment*>> children;
    std::unordered_map<std::string, std::size_t> parent_count;
    std::unordered_map<std::string, const Attachment*> attachment_by_child;
    std::unordered_map<std::string, std::size_t> symmetry_tag_counts;
    for (const auto& attachment : attachments_) {
        if (attachment.enabled && !attachment.symmetry_tag.empty()) {
            symmetry_tag_counts[attachment.symmetry_tag] += 1U;
        }
    }
    for (const auto& [tag, count] : symmetry_tag_counts) {
        if (count % 2U != 0U) {
            std::cerr << "[HexacopterArchitecture] symmetry_tag '" << tag
                      << "' has odd count " << count << " (expected even)\n";
        }
    }

    for (const auto& attachment : attachments_) {
        if (!attachment.enabled) {
            continue;
        }
        if (!attachment.parent_id.empty() && !element_by_id.contains(attachment.parent_id)) {
            throw std::runtime_error("Attachment parent not found: " + attachment.parent_id);
        }
        if (!element_by_id.contains(attachment.child_id)) {
            throw std::runtime_error("Attachment child not found: " + attachment.child_id);
        }
        parent_count[attachment.child_id] += 1U;
        if (parent_count[attachment.child_id] > 1U) {
            throw std::runtime_error("Attachment child has multiple parents: " + attachment.child_id);
        }
        children[attachment.parent_id].push_back(&attachment);
        attachment_by_child[attachment.child_id] = &attachment;
    }

    for (const auto& [child_id, attachment] : attachment_by_child) {
        if (attachment == nullptr || attachment->contact_policy != AttachmentContactPolicy::bonded_overlap) {
            continue;
        }

        std::string descendant = child_id;
        std::string cursor = child_id;
        while (true) {
            const auto it = attachment_by_child.find(cursor);
            if (it == attachment_by_child.end() || it->second == nullptr) {
                break;
            }
            const Attachment& bonded = *it->second;
            if (bonded.contact_policy != AttachmentContactPolicy::bonded_overlap || bonded.parent_id.empty()) {
                break;
            }
            packaging_exemptions_.insert(packagingPairKey(descendant, bonded.parent_id));
            cursor = bonded.parent_id;
        }
    }

    for (const auto& [child_id, attachment] : attachment_by_child) {
        if (attachment == nullptr || attachment->contact_policy != AttachmentContactPolicy::allow_touch) {
            continue;
        }
        if (!attachment->parent_id.empty()) {
            packaging_exemptions_.insert(packagingPairKey(child_id, attachment->parent_id));
        }
    }

    std::unordered_set<std::string> visiting;
    std::unordered_set<std::string> visited;
    std::function<void(const std::string&, const Eigen::Isometry3d&)> visit =
        [&](const std::string& parent_id, const Eigen::Isometry3d& parent_pose) {
            if (!parent_id.empty()) {
                if (visiting.contains(parent_id)) {
                    throw std::runtime_error("Attachment cycle detected at element: " + parent_id);
                }
                if (visited.contains(parent_id)) {
                    return;
                }
                visiting.insert(parent_id);
            }

            const auto child_it = children.find(parent_id);
            if (child_it == children.end()) {
                if (!parent_id.empty()) {
                    visiting.erase(parent_id);
                    visited.insert(parent_id);
                }
                return;
            }

            for (const auto* attachment : child_it->second) {
                const auto element_it = element_by_id.find(attachment->child_id);
                if (element_it == element_by_id.end()) {
                    continue;
                }

                const Eigen::Isometry3d relative_pose = attachment->relative_transform
                    ? attachment->relative_transform(*this)
                    : attachment->relationship.resolve();
                Eigen::Isometry3d parent_anchor_pose = parent_pose;
                if (!attachment->parent_id.empty() && !attachment->parent_anchor.empty()) {
                    const auto parent_element_it = element_by_id.find(attachment->parent_id);
                    if (parent_element_it == element_by_id.end()) {
                        throw std::runtime_error("Attachment parent missing during assembly: " + attachment->parent_id);
                    }
                    const auto local_anchor = parent_element_it->second->anchorPose(attachment->parent_anchor);
                    if (!local_anchor.has_value()) {
                        throw std::runtime_error(
                            "Missing parent anchor '" + attachment->parent_anchor + "' on element " + attachment->parent_id);
                    }
                    parent_anchor_pose = parent_pose * local_anchor.value();
                }

                Eigen::Isometry3d child_anchor_pose = Eigen::Isometry3d::Identity();
                if (!attachment->child_anchor.empty()) {
                    const auto local_anchor = element_it->second->anchorPose(attachment->child_anchor);
                    if (!local_anchor.has_value()) {
                        throw std::runtime_error(
                            "Missing child anchor '" + attachment->child_anchor + "' on element " + attachment->child_id);
                    }
                    child_anchor_pose = local_anchor.value();
                }

                const Eigen::Isometry3d world_pose =
                    parent_anchor_pose * relative_pose * child_anchor_pose.inverse() * element_it->second->localPose();

                AssembledElement assembled;
                assembled.element = element_it->second;
                assembled.world_pose = world_pose;
                assembled.local_primitives = element_it->second->localPrimitives();
                assembly_state_.elements.push_back(std::move(assembled));

                visit(attachment->child_id, world_pose);
            }

            if (!parent_id.empty()) {
                visiting.erase(parent_id);
                visited.insert(parent_id);
            }
        };

    visit("", Eigen::Isometry3d::Identity());

    for (const auto& [element_id, element] : element_by_id) {
        if (!visited.contains(element_id)) {
            throw std::runtime_error("Orphaned element not reachable from root: " + element_id);
        }
    }
}

}  // namespace hexaarch::core
