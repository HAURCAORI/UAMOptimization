#pragma once

#include <memory>
#include <optional>
#include <string>

#include "core/HexacopterArchitecture.hpp"
#include "evaluation/EvaluationResult.hpp"
#include "visualization/ViewerCamera.hpp"

namespace hexaarch::visualization {

class ArchitectureViewerApp {
public:
    struct Config {
        std::string title = "HexaArch Visualizer";
        int width = 1280;
        int height = 720;
        bool validation_layers = false;
    };
    struct Impl;

    ArchitectureViewerApp();
    explicit ArchitectureViewerApp(Config config);
    ~ArchitectureViewerApp();
    ArchitectureViewerApp(const ArchitectureViewerApp&) = delete;
    ArchitectureViewerApp& operator=(const ArchitectureViewerApp&) = delete;

    void setArchitecture(const core::HexacopterArchitecture& architecture);
    void postArchitecture(core::HexacopterArchitecture architecture, std::string title = {});
    // Set evaluation result for initial display (call before run()).
    void setResult(evaluation::EvaluationResult result);
    // Thread-safe: post a new result from the optimizer thread.
    void postResult(evaluation::EvaluationResult result);
    void requestClose();
    [[nodiscard]] int run();
    [[nodiscard]] const ViewerCamera& camera() const;

private:
    Config config_;
    const core::HexacopterArchitecture* architecture_ = nullptr;
    std::optional<evaluation::EvaluationResult> result_;
    std::unique_ptr<Impl> impl_;
};

}  // namespace hexaarch::visualization
