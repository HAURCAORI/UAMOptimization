#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "core/AssemblyState.hpp"
#include "core/Attachment.hpp"
#include "core/ConstraintRegistry.hpp"
#include "core/DefaultHexacopterBuilder.hpp"
#include "core/Elements.hpp"
#include "core/ParameterRegistry.hpp"
#include "core/SpatialElement.hpp"

namespace hexaarch::core {

class HexacopterArchitecture {
public:
    HexacopterArchitecture();
    static HexacopterArchitecture createEmpty();

    HexacopterArchitecture(const HexacopterArchitecture& other);
    HexacopterArchitecture& operator=(const HexacopterArchitecture& other);
    HexacopterArchitecture(HexacopterArchitecture&&) noexcept = default;
    HexacopterArchitecture& operator=(HexacopterArchitecture&&) noexcept = default;

    [[nodiscard]] const std::string& id() const;
    [[nodiscard]] ParameterRegistry& parameters();
    [[nodiscard]] const ParameterRegistry& parameters() const;
    [[nodiscard]] ConstraintRegistry& constraints();
    [[nodiscard]] const ConstraintRegistry& constraints() const;
    [[nodiscard]] const std::vector<SpatialElementPtr>& elements() const;
    [[nodiscard]] const std::vector<Attachment>& attachments() const;
    [[nodiscard]] const AssemblyState& assemblyState() const;
    [[nodiscard]] const SpatialElement* findElement(std::string_view element_id) const;
    [[nodiscard]] bool isPackagingPairExempt(std::string_view lhs_id, std::string_view rhs_id) const;

    void addElement(SpatialElementPtr element);
    bool removeElement(std::string_view element_id);
    void clearElements();
    void addAttachment(Attachment attachment);
    bool removeAttachment(std::string_view child_id);
    void clearAttachments();
    void addConstraint(Constraint constraint);
    bool removeConstraint(std::string_view stable_id);
    void rebuildAssembly();

    void updateFromParameters();

    [[nodiscard]] double Lx() const;
    [[nodiscard]] double Lyi() const;
    [[nodiscard]] double Lyo() const;
    [[nodiscard]] double Tmax() const;
    [[nodiscard]] double cT() const;
    [[nodiscard]] double propellerDiameter() const;
    [[nodiscard]] double payloadMass() const;
    [[nodiscard]] bool useVehicleModel() const;
    [[nodiscard]] double nominalMass() const;
    [[nodiscard]] double gravity() const;

private:
    struct EmptyTag {};
    explicit HexacopterArchitecture(EmptyTag);

    static std::string packagingPairKey(std::string_view lhs_id, std::string_view rhs_id);

    void registerDefaultParameters();
    void registerDefaultConstraints();
    void registerElementConstraints();
    void bindCanonicalParameters();
    void rebuildElements();
    void rebuildAttachments();
    void assemble();

    std::string id_;
    double nominal_mass_ = 2240.73;
    bool use_vehicle_model_ = true;
    double gravity_ = 9.81;
    ParameterRegistry parameters_;
    ConstraintRegistry constraints_;
    std::vector<SpatialElementPtr> elements_;
    std::vector<Attachment> attachments_;
    AssemblyState assembly_state_;
    std::unordered_set<std::string> packaging_exemptions_;
    DesignParameter* Lx_parameter_ = nullptr;
    DesignParameter* Lyi_parameter_ = nullptr;
    DesignParameter* Lyo_parameter_ = nullptr;
    DesignParameter* Tmax_parameter_ = nullptr;
    DesignParameter* cT_parameter_ = nullptr;
    DesignParameter* dprop_parameter_ = nullptr;
    DesignParameter* payload_parameter_ = nullptr;
};

}  // namespace hexaarch::core
