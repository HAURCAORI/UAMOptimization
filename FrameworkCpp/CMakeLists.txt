set(HEXAARCH_SOURCES
    src/analysis/ComparisonReporter.cpp
    src/analysis/CsvExporter.cpp
    src/analysis/ParetoAnalyzer.cpp
    src/core/Constraint.cpp
    src/core/ConstraintRegistry.cpp
    src/core/DesignParameter.cpp
    src/core/AssemblyState.cpp
    src/core/DefaultHexacopterBuilder.cpp
    src/core/Elements.cpp
    src/core/HexacopterArchitecture.cpp
    src/core/ParameterRegistry.cpp
    src/evaluation/ArchitectureEvaluator.cpp
    src/evaluation/EvaluationContext.cpp
    src/evaluation/ObjectiveAggregator.cpp
    src/evaluation/Stage1Evaluator.cpp
    src/optimization/DesignVectorMapper.cpp
    src/optimization/MooRunner.cpp
    src/optimization/OptimizationProblem.cpp
    src/optimization/PagmoProblemAdapter.cpp
    src/optimization/SooRunner.cpp
    src/physics/AllocationMatrixBuilder.cpp
    src/physics/PhysicsTypes.cpp
    src/physics/PrimitiveDistance.cpp
    src/physics/VehicleScalingModel.cpp
    visualization/src/visualization/ArchitectureSceneBuilder.cpp
    visualization/src/visualization/ArchitectureViewerApp.cpp
    visualization/src/visualization/PrimitiveInstance.cpp
    visualization/src/visualization/PrimitiveMeshFactory.cpp
    visualization/src/visualization/ViewerCamera.cpp
)

set(HEXAARCH_HEADERS
    include/analysis/ComparisonReporter.hpp
    include/analysis/CsvExporter.hpp
    include/analysis/ParetoAnalyzer.hpp
    include/core/Constraint.hpp
    include/core/ConstraintRegistry.hpp
    include/core/DesignParameter.hpp
    include/core/AssemblyState.hpp
    include/core/Attachment.hpp
    include/core/DefaultHexacopterBuilder.hpp
    include/core/Elements.hpp
    include/core/GeometryPrimitive.hpp
    include/core/HexacopterArchitecture.hpp
    include/core/ParameterRegistry.hpp
    include/core/SpatialElement.hpp
    include/evaluation/ArchitectureEvaluator.hpp
    include/evaluation/EvaluationContext.hpp
    include/evaluation/EvaluationResult.hpp
    include/evaluation/ObjectiveAggregator.hpp
    include/evaluation/Stage1Evaluator.hpp
    include/optimization/DesignVectorMapper.hpp
    include/optimization/MooRunner.hpp
    include/optimization/OptimizationProblem.hpp
    include/optimization/PagmoProblemAdapter.hpp
    include/optimization/SooRunner.hpp
    include/physics/AllocationMatrixBuilder.hpp
    include/physics/PhysicsTypes.hpp
    include/physics/PrimitiveDistance.hpp
    include/physics/VehicleScalingModel.hpp
    visualization/include/visualization/ArchitectureSceneBuilder.hpp
    visualization/include/visualization/ArchitectureViewerApp.hpp
    visualization/include/visualization/PrimitiveInstance.hpp
    visualization/include/visualization/PrimitiveMeshFactory.hpp
    visualization/include/visualization/ViewerCamera.hpp
)

set(HEXAARCH_VCPKG_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg_installed/x64-windows/include")
set(HEXAARCH_VCPKG_EIGEN_INCLUDE_DIR "${HEXAARCH_VCPKG_INCLUDE_DIR}/eigen3")

add_library(hexaarch STATIC)
target_sources(hexaarch PRIVATE ${HEXAARCH_SOURCES} ${HEXAARCH_HEADERS})
target_compile_features(hexaarch PUBLIC cxx_std_20)
target_compile_definitions(hexaarch PUBLIC _ALLOW_COMPILER_AND_STL_VERSION_MISMATCH)
target_include_directories(
    hexaarch
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/visualization/include"
)

if(EXISTS "${HEXAARCH_VCPKG_INCLUDE_DIR}")
    target_include_directories(hexaarch PUBLIC "${HEXAARCH_VCPKG_INCLUDE_DIR}")
endif()
if(EXISTS "${HEXAARCH_VCPKG_EIGEN_INCLUDE_DIR}")
    target_include_directories(hexaarch PUBLIC "${HEXAARCH_VCPKG_EIGEN_INCLUDE_DIR}")
endif()

if(MSVC)
    target_compile_options(hexaarch PRIVATE /W3 /permissive- /EHsc)
endif()

add_executable(hexaarch_stage1 app/main.cpp)
target_link_libraries(hexaarch_stage1 PRIVATE hexaarch)

option(HEXAARCH_ENABLE_LSP_TEST_TARGETS "Generate test compile commands for clangd." ON)

if(HEXAARCH_ENABLE_LSP_TEST_TARGETS)
    add_executable(hexaarch_smoke_test EXCLUDE_FROM_ALL tests/test_smoke.cpp)
    target_link_libraries(hexaarch_smoke_test PRIVATE hexaarch)
endif()
