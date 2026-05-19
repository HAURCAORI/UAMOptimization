#pragma once

#include <string>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "physics/AttainableControlSetAnalyzer.hpp"
#include "physics/PhysicsTypes.hpp"

namespace hexaarch::analysis {

// Generates self-contained SVG figures and a CSV vertex dump for ACS visualization.
// No external runtime dependencies (no gnuplot, no matplot++).
// Output files per call to plot():
//   acs_fig1_lm.svg         — vertex scatter L vs M
//   acs_fig2_lfz.svg        — vertex scatter L vs Fz
//   acs_fig3_hover_lm.svg   — hover slice (Fz=-W) L vs M
//   acs_fig4_n0_lfz.svg     — N=0 slice L vs Fz
//   acs_fig5_hover_poly.svg — 2D hover moment polygon [L, M] at Fz=-W, N=0
//   acs_fig6_margins.svg    — directional margin bars: nominal vs worst-fault
//   acs_retention.svg       — per-motor ACS retention bar chart
//   acs_vertices.csv        — full 4D vertex cloud (nominal + worst-fault)
class AcsPlotter {
public:
    struct Config {
        std::string output_dir = ".";
        std::string label = "Design";
    };

    void plot(
        const physics::AllocationMatrix& B,
        double thrust_max,
        double vehicle_weight,
        const physics::AcsResult& acs,
        const Config& config) const;

private:
    [[nodiscard]] std::vector<Eigen::Vector4d> sliceAt(
        const std::vector<Eigen::Vector4d>& pts,
        const std::vector<std::pair<int, int>>& edges,
        int dim_cut,
        double val_cut,
        double tol) const;

    [[nodiscard]] std::vector<int> convexHull2d(
        const std::vector<Eigen::Vector2d>& pts) const;

    void scatter2dSvg(
        const std::vector<Eigen::Vector4d>& nom,
        const std::vector<Eigen::Vector4d>& fault,
        int cx, int cy,
        const std::string& xl,
        const std::string& yl,
        const std::string& title,
        const std::string& path) const;

    void hoverPolygonSvg(
        const std::vector<Eigen::Vector2d>& hull_nom,
        const std::vector<Eigen::Vector2d>& hull_fault,
        double nom_min_margin,
        double fault_min_margin,
        const std::string& xl,
        const std::string& yl,
        const std::string& title,
        const std::string& path) const;

    void retentionBarSvg(
        const physics::AcsResult& acs,
        int worst_motor,
        const std::string& title,
        const std::string& path) const;

    // Grouped bar chart: directional margins at hover for nominal vs worst-fault case.
    void marginBarsSvg(
        const physics::AcsCaseResult& nominal,
        const physics::AcsCaseResult& worst_fault,
        int fault_motor,
        const std::string& title,
        const std::string& path) const;

    void exportVerticesCsv(
        const std::vector<Eigen::Vector4d>& nom,
        const std::vector<Eigen::Vector4d>& fault,
        int worst_motor,
        const std::string& path) const;
};

}  // namespace hexaarch::analysis
