#include <algorithm>
#include <stdexcept>
#include <string_view>

#include "analysis/ComparisonReporter.hpp"
#include "analysis/CsvExporter.hpp"
#include "analysis/ParetoAnalyzer.hpp"
#include <gtest/gtest.h>

#include "physics/PrimitiveDistance.hpp"
#include "physics/AllocationMatrixBuilder.hpp"
#include "physics/VehicleScalingModel.hpp"
#include "core/HexacopterArchitecture.hpp"
#include "evaluation/ArchitectureEvaluator.hpp"
#include "optimization/DesignVectorMapper.hpp"
#include "optimization/MooRunner.hpp"
#include "optimization/PagmoProblemAdapter.hpp"

TEST(Phase0Smoke, EvaluatesDefaultArchitecture) {
    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::ArchitectureEvaluator evaluator;

    const auto result = evaluator.evaluate(architecture);

    EXPECT_TRUE(result.feasible);
    EXPECT_GT(result.combined_objective, 0.0);
    EXPECT_GT(result.stage1.mass, 0.0);
    EXPECT_GT(result.stage1.power, 0.0);
    EXPECT_GT(result.physical_model.mass_properties.mass, 0.0);
}

TEST(CoreAssembly, DefaultArchitectureBuildsAttachmentGraph) {
    hexaarch::core::HexacopterArchitecture architecture;

    EXPECT_FALSE(architecture.attachments().empty());

    const auto* body = architecture.assemblyState().find("body");
    const auto* rotor1 = architecture.assemblyState().find("rotor_1");
    const auto* rotor6 = architecture.assemblyState().find("rotor_6");

    ASSERT_NE(body, nullptr);
    ASSERT_NE(rotor1, nullptr);
    ASSERT_NE(rotor6, nullptr);

    EXPECT_NEAR(body->world_pose.translation().norm(), 0.0, 1e-9);
    EXPECT_GT(rotor1->world_pose.translation().norm(), 0.0);
    EXPECT_GT(rotor6->world_pose.translation().norm(), 0.0);
}

TEST(CoreAssembly, DefaultAttachmentsCarryBondedContactPolicy) {
    hexaarch::core::HexacopterArchitecture architecture;

    const auto has_bonded = [&](std::string_view parent_id, std::string_view child_id) {
        return std::any_of(
            architecture.attachments().begin(),
            architecture.attachments().end(),
            [&](const hexaarch::core::Attachment& attachment) {
                return attachment.parent_id == parent_id &&
                       attachment.child_id == child_id &&
                       attachment.contact_policy == hexaarch::core::AttachmentContactPolicy::bonded_overlap;
            });
    };

    EXPECT_TRUE(has_bonded("body", "arm_1"));
    EXPECT_TRUE(has_bonded("arm_1", "motor_1"));
    EXPECT_TRUE(has_bonded("motor_1", "rotor_1"));
}

TEST(Physics, PrimitiveDistanceDetectsSeparatedRotors) {
    using hexaarch::core::GeometryPrimitive;
    using hexaarch::physics::primitiveClearance;

    const auto lhs = GeometryPrimitive::makeDisk(0.2, 0.0);
    const auto rhs = GeometryPrimitive::makeDisk(0.2, 0.0);

    Eigen::Isometry3d lhs_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d rhs_pose = Eigen::Isometry3d::Identity();
    rhs_pose.translation() = Eigen::Vector3d(0.6, 0.0, 0.0);

    EXPECT_NEAR(primitiveClearance(lhs, lhs_pose, rhs, rhs_pose), 0.2, 1e-9);
}

TEST(Physics, PrimitiveDistanceHandlesSphereBoxClearance) {
    using hexaarch::core::GeometryPrimitive;
    using hexaarch::physics::primitiveClearance;

    const auto box = GeometryPrimitive::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5), 0.0);
    const auto sphere = GeometryPrimitive::makeSphere(0.2, 0.0);

    Eigen::Isometry3d box_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d sphere_pose = Eigen::Isometry3d::Identity();
    sphere_pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

    EXPECT_NEAR(primitiveClearance(box, box_pose, sphere, sphere_pose), 0.3, 1e-9);
}

TEST(Physics, PrimitiveDistanceHandlesSegmentDiskClearance) {
    using hexaarch::core::GeometryPrimitive;
    using hexaarch::physics::primitiveClearance;

    const auto segment = GeometryPrimitive::makeSegment(2.0, 0.0);
    const auto disk = GeometryPrimitive::makeDisk(0.2, 0.0);

    Eigen::Isometry3d segment_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d disk_pose = Eigen::Isometry3d::Identity();
    disk_pose.translation() = Eigen::Vector3d(1.0, 0.5, 0.0);

    EXPECT_NEAR(primitiveClearance(segment, segment_pose, disk, disk_pose), 0.3, 1e-9);
}

TEST(Physics, PrimitiveDistanceHandlesBoxBoxClearance) {
    using hexaarch::core::GeometryPrimitive;
    using hexaarch::physics::primitiveClearance;

    const auto lhs = GeometryPrimitive::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5), 0.0);
    const auto rhs = GeometryPrimitive::makeBox(Eigen::Vector3d(0.25, 0.25, 0.25), 0.0);

    Eigen::Isometry3d lhs_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d rhs_pose = Eigen::Isometry3d::Identity();
    rhs_pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

    EXPECT_NEAR(primitiveClearance(lhs, lhs_pose, rhs, rhs_pose), 0.25, 1e-9);
}

TEST(Physics, PrimitiveDistanceHandlesCylinderDiskClearance) {
    using hexaarch::core::GeometryPrimitive;
    using hexaarch::physics::primitiveClearance;

    const auto cylinder = GeometryPrimitive::makeCylinder(0.2, 2.0, 0.0);
    const auto disk = GeometryPrimitive::makeDisk(0.1, 0.0);

    Eigen::Isometry3d cylinder_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d disk_pose = Eigen::Isometry3d::Identity();
    disk_pose.translation() = Eigen::Vector3d(0.5, 1.5, 0.0);

    EXPECT_NEAR(primitiveClearance(cylinder, cylinder_pose, disk, disk_pose), 0.2, 1e-9);
}

TEST(ParameterRegistry, StableOrderingTracksActiveParameters) {
    hexaarch::core::HexacopterArchitecture architecture;

    const auto active = architecture.parameters().activeParameters();

    ASSERT_EQ(active.size(), 5U);
    EXPECT_EQ(active.at(0)->stable_id(), "default-architecture::Lx");
    EXPECT_EQ(active.at(1)->stable_id(), "default-architecture::Lyi");
    EXPECT_EQ(active.at(2)->stable_id(), "default-architecture::Lyo");
    EXPECT_EQ(active.at(3)->stable_id(), "default-architecture::T_max");
    EXPECT_EQ(active.at(4)->stable_id(), "default-architecture::cT");
}

TEST(ParameterRegistry, SharedParametersTrackMultipleConsumers) {
    hexaarch::core::HexacopterArchitecture architecture;

    const auto* thrust_max = architecture.parameters().find("default-architecture::T_max");
    ASSERT_NE(thrust_max, nullptr);

    EXPECT_TRUE(thrust_max->isConsumedBy("battery"));
    EXPECT_TRUE(thrust_max->isConsumedBy("motor_1"));
    EXPECT_TRUE(thrust_max->isConsumedBy("motor_6"));
    EXPECT_GE(thrust_max->consumer_ids.size(), 7U);
}

TEST(AllocationMatrixBuilder, MatchesBaselineLayout) {
    const auto matrix = hexaarch::physics::AllocationMatrixBuilder::build(2.65, 2.65, 5.50, 0.03);

    EXPECT_DOUBLE_EQ(matrix(0, 0), -1.0);
    EXPECT_DOUBLE_EQ(matrix(1, 2), -5.50);
    EXPECT_DOUBLE_EQ(matrix(2, 4), -2.65);
    EXPECT_DOUBLE_EQ(matrix(3, 5), 0.03);
}

TEST(AllocationMatrixBuilder, BuildsFromExplicitRotorPositions) {
    std::array<Eigen::Vector3d, 6> rotor_positions{{
        {2.0, -1.0, 0.0},
        {2.2, 1.3, 0.0},
        {0.1, -4.0, 0.0},
        {-0.2, 4.5, 0.0},
        {-2.1, -1.4, 0.0},
        {-1.8, 1.2, 0.0}
    }};
    std::array<double, 6> yaw_signs{{-1.0, -1.0, 1.0, 1.0, -1.0, 1.0}};

    const auto matrix = hexaarch::physics::AllocationMatrixBuilder::build(rotor_positions, yaw_signs, 0.03);

    EXPECT_DOUBLE_EQ(matrix(1, 0), -1.0);
    EXPECT_DOUBLE_EQ(matrix(1, 3), 4.5);
    EXPECT_DOUBLE_EQ(matrix(2, 1), 2.2);
    EXPECT_DOUBLE_EQ(matrix(2, 4), -2.1);
    EXPECT_DOUBLE_EQ(matrix(3, 2), 0.03);
    EXPECT_DOUBLE_EQ(matrix(3, 4), -0.03);
}

TEST(DesignVectorMapper, PackAndUnpackNormalizedVector) {
    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::optimization::DesignVectorMapper mapper;

    auto packed = mapper.pack(architecture);
    ASSERT_EQ(packed.size(), 5U);

    packed.at(0) = 0.25;
    packed.at(1) = 0.30;
    mapper.unpackNormalized(architecture, packed);

    EXPECT_NEAR(architecture.Lx(), 2.0, 1e-9);
    EXPECT_NEAR(architecture.Lyi(), 2.2, 1e-9);
}

TEST(PagmoProblemAdapter, ExposesProblemMetadataForSoo) {
    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::EvaluationContext context;
    hexaarch::optimization::PagmoProblemAdapter adapter{architecture, context, {"combined"}, true};

    const auto problem = adapter.problem();

    EXPECT_EQ(problem.lower_bounds.size(), 5U);
    EXPECT_EQ(problem.upper_bounds.size(), 5U);
    EXPECT_EQ(problem.parameter_scales.size(), 5U);
    EXPECT_EQ(problem.parameter_ids.size(), 5U);
    ASSERT_EQ(problem.objective_names.size(), 1U);
    EXPECT_EQ(problem.objective_names.front(), "combined");
}

TEST(Evaluation, ConstraintCallbacksPopulateResults) {
    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::ArchitectureEvaluator evaluator;

    const auto result = evaluator.evaluate(architecture);

    EXPECT_FALSE(result.constraint_results.empty());
    for (const auto& constraint : result.constraint_results) {
        EXPECT_FALSE(constraint.name.empty());
        EXPECT_FALSE(constraint.stable_id.empty());
    }
}

TEST(CoreElements, PublicElementsRegisterLocalConstraints) {
    hexaarch::core::HexacopterArchitecture architecture;

    EXPECT_NE(architecture.constraints().find("body::body_span_order"), nullptr);
    EXPECT_NE(architecture.constraints().find("battery::battery_sizing_inputs"), nullptr);
    EXPECT_NE(architecture.constraints().find("motor_1::motor_thrust_positive"), nullptr);
    EXPECT_NE(architecture.constraints().find("rotor_1::rotor_diameter_positive"), nullptr);
}

TEST(CoreComposition, PublicApiSupportsCustomGraphAndCopy) {
    hexaarch::core::HexacopterArchitecture architecture;
    architecture.clearElements();

    auto* Lx = architecture.parameters().find("default-architecture::Lx");
    auto* Lyi = architecture.parameters().find("default-architecture::Lyi");
    auto* Lyo = architecture.parameters().find("default-architecture::Lyo");
    auto* Tmax = architecture.parameters().find("default-architecture::T_max");
    auto* dprop = architecture.parameters().find("default-architecture::d_prop");
    ASSERT_NE(Lx, nullptr);
    ASSERT_NE(Lyi, nullptr);
    ASSERT_NE(Lyo, nullptr);
    ASSERT_NE(Tmax, nullptr);
    ASSERT_NE(dprop, nullptr);

    architecture.addElement(std::make_unique<hexaarch::core::BodyHullElement>("custom_body", Lx, Lyi, Lyo));
    architecture.addElement(std::make_unique<hexaarch::core::BatteryElement>("custom_battery", Tmax, dprop));
    architecture.addAttachment({
        "",
        "custom_body",
        "",
        "center",
        hexaarch::core::AttachmentRelationship::rigidMount(),
        true,
        "root",
        hexaarch::core::AttachmentContactPolicy::bonded_overlap,
        nullptr
    });
    architecture.addAttachment({
        "custom_body",
        "custom_battery",
        "bottom",
        "mount",
        hexaarch::core::AttachmentRelationship::localOffset(Eigen::Vector3d(0.0, 0.0, 0.02)),
        true,
        "battery",
        hexaarch::core::AttachmentContactPolicy::bonded_overlap,
        nullptr
    });
    architecture.rebuildAssembly();

    ASSERT_NE(architecture.assemblyState().find("custom_body"), nullptr);
    ASSERT_NE(architecture.assemblyState().find("custom_battery"), nullptr);
    EXPECT_EQ(architecture.findElement("body"), nullptr);

    hexaarch::core::HexacopterArchitecture copied = architecture;
    ASSERT_NE(copied.assemblyState().find("custom_body"), nullptr);
    ASSERT_NE(copied.assemblyState().find("custom_battery"), nullptr);
    EXPECT_EQ(copied.findElement("body"), nullptr);
}

TEST(Physics, ActiveCTParameterChangesYawAuthority) {
    hexaarch::core::HexacopterArchitecture baseline;
    hexaarch::physics::VehicleScalingModel model_builder;
    const auto baseline_model = model_builder.evaluate(baseline);

    hexaarch::core::HexacopterArchitecture candidate = baseline;
    auto* cT = candidate.parameters().find("default-architecture::cT");
    ASSERT_NE(cT, nullptr);
    cT->value = 0.06;
    candidate.updateFromParameters();

    const auto candidate_model = model_builder.evaluate(candidate);
    EXPECT_NEAR(baseline_model.allocation_matrix(3, 0), 0.03, 1e-9);
    EXPECT_NEAR(candidate_model.allocation_matrix(3, 0), 0.06, 1e-9);
}

TEST(Physics, RotorYawSignsAreElementDefined) {
    hexaarch::core::HexacopterArchitecture architecture;

    const auto* rotor1 = dynamic_cast<const hexaarch::core::RotorElement*>(architecture.findElement("rotor_1"));
    const auto* rotor3 = dynamic_cast<const hexaarch::core::RotorElement*>(architecture.findElement("rotor_3"));
    ASSERT_NE(rotor1, nullptr);
    ASSERT_NE(rotor3, nullptr);

    EXPECT_DOUBLE_EQ(rotor1->yawMomentSign(), -1.0);
    EXPECT_DOUBLE_EQ(rotor3->yawMomentSign(), 1.0);
}

TEST(Analysis, ComparisonReporterProducesCsvTable) {
    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::ArchitectureEvaluator evaluator;

    const auto baseline = evaluator.evaluate(architecture);
    auto candidate = baseline;
    candidate.combined_objective -= 10.0;

    const auto table = hexaarch::analysis::ComparisonReporter::summaryTable({
        {"baseline", baseline},
        {"candidate", candidate}
    });

    EXPECT_NE(table.find("label,feasible,objective"), std::string::npos);
    EXPECT_NE(table.find("candidate"), std::string::npos);
}

TEST(Analysis, ParetoAnalyzerFindsNondominatedPoints) {
    hexaarch::optimization::MooRunResult result;
    result.algorithm_name = "nsga2";
    result.population_size = 3U;
    result.generations = 5U;
    result.problem.objective_names = {"mass", "power"};

    hexaarch::optimization::MooPoint point_a;
    point_a.objective_vector = {1.0, 3.0};
    point_a.evaluation.stage1.mass = 1.0;
    point_a.evaluation.stage1.power = 3.0;

    hexaarch::optimization::MooPoint point_b;
    point_b.objective_vector = {3.0, 1.0};
    point_b.evaluation.stage1.mass = 3.0;
    point_b.evaluation.stage1.power = 1.0;

    hexaarch::optimization::MooPoint point_c;
    point_c.objective_vector = {4.0, 4.0};
    point_c.evaluation.stage1.mass = 4.0;
    point_c.evaluation.stage1.power = 4.0;

    result.population = {point_a, point_b, point_c};

    const auto summary = hexaarch::analysis::ParetoAnalyzer{}.analyze(result);

    EXPECT_EQ(summary.nondominated_indices.size(), 2U);
    EXPECT_TRUE(summary.knee_index == 0U || summary.knee_index == 1U);
}

TEST(Analysis, ParetoAnalyzerKeepsRawPopulationIndicesForFeasibleSubset) {
    hexaarch::optimization::MooRunResult result;
    result.algorithm_name = "nsga2";
    result.population_size = 3U;
    result.generations = 5U;
    result.problem.objective_names = {"mass", "power"};

    hexaarch::optimization::MooPoint infeasible;
    infeasible.population_index = 0U;
    infeasible.objective_vector = {0.5, 0.5};
    infeasible.evaluation.feasible = false;

    hexaarch::optimization::MooPoint point_a;
    point_a.population_index = 1U;
    point_a.objective_vector = {1.0, 3.0};
    point_a.evaluation.feasible = true;
    point_a.evaluation.stage1.mass = 1.0;
    point_a.evaluation.stage1.power = 3.0;

    hexaarch::optimization::MooPoint point_b;
    point_b.population_index = 2U;
    point_b.objective_vector = {3.0, 1.0};
    point_b.evaluation.feasible = true;
    point_b.evaluation.stage1.mass = 3.0;
    point_b.evaluation.stage1.power = 1.0;

    result.population = {infeasible, point_a, point_b};
    result.feasible_population = {point_a, point_b};
    result.has_feasible_points = true;

    const auto summary = hexaarch::analysis::ParetoAnalyzer{}.analyze(result);

    EXPECT_EQ(summary.nondominated_indices.size(), 2U);
    EXPECT_EQ(summary.nondominated_indices.at(0), 1U);
    EXPECT_EQ(summary.nondominated_indices.at(1), 2U);
    EXPECT_TRUE(summary.knee_index == 1U || summary.knee_index == 2U);
}

TEST(Optimization, MooRunnerRejectsInvalidNsgaPopulationSize) {
    hexaarch::core::HexacopterArchitecture architecture;
    hexaarch::evaluation::EvaluationContext context;
    hexaarch::optimization::MooRunner runner;
    hexaarch::optimization::MooRunConfig config;
    config.population_size = 10U;
    config.generations = 2U;

    EXPECT_THROW(runner.run(architecture, context, config), std::invalid_argument);
}
