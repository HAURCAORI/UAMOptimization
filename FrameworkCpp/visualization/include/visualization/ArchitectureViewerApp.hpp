#pragma once

#include <memory>
#include <string>

#include "core/HexacopterArchitecture.hpp"
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
    [[nodiscard]] int run();
    [[nodiscard]] const ViewerCamera& camera() const;

private:
    Config config_;
    const core::HexacopterArchitecture* architecture_ = nullptr;
    std::unique_ptr<Impl> impl_;
};

}  // namespace hexaarch::visualization
