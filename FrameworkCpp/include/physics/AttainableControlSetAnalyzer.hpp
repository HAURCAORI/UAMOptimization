#pragma once

#include <array>
#include <string>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "physics/PhysicsTypes.hpp"

namespace hexaarch::physics {

struct AcsTrimSolution {
    bool feasible = false;
    double utilization = 0.0;
    std::array<double, kNumRotors> thrust{};
};

struct AcsDirectionalMargin {
    std::string label;
    double margin = 0.0;
};

struct AcsCaseResult {
    bool trim_feasible = false;
    double trim_utilization = 0.0;
    std::array<double, kNumRotors> trim_thrust{};
    double min_margin = 0.0;
    std::vector<AcsDirectionalMargin> directional_margins;
    // Named reserves at the trim point (margin in each principal axis direction)
    double yaw_reserve   = 0.0;
    double roll_reserve  = 0.0;
    double pitch_reserve = 0.0;

    // 2D hover slice: achievable (L, M) at Fz = -mg, N = 0 (BFS exact polygon, CCW).
    // Empty if fewer than 3 unique feasible vertices are found.
    std::vector<Eigen::Vector2d> hover_slice_polygon;
    // Signed distance from (L=0, M=0) to the hover slice polygon boundary.
    // Positive = origin strictly inside (hover achievable with moment margin).
    // Negative = origin outside (hover not achievable under this fault).
    double hover_slice_signed_distance = 0.0;
};

// Volume-based metrics matching MATLAB eval_acs.m
struct AcsResult {
    AcsCaseResult nominal;
    std::array<AcsCaseResult, kNumRotors> faulted{};

    // --- Directional-margin summary (Phase 1 — support-function method) ---
    double worst_fault_min_margin = 0.0;
    double overall_min_margin = 0.0;
    // Ratio of worst-fault min-margin to nominal min-margin [0,1]; degradation indicator.
    double faulted_to_nominal_ratio = 0.0;

    // --- Volume-based metrics (matching MATLAB eval_acs.m) ---
    // Analytic zonotope formula: V = Σ_{|I|=4} |det(B_I)| × Π f_max_i
    double volume_nominal = 0.0;
    std::array<double, kNumRotors> volume_faulted{};
    std::array<double, kNumRotors> retention{};

    // PFWAR: Probabilistic Fault-Weighted ACS Retention (equal weights for single faults)
    double PFWAR = 0.0;
    // FII: Fault Isotropy Index = std(retention) / mean(retention)
    double FII = 0.0;
    // WCFR: Worst-Case Fault Retention = min(retention)
    double WCFR = 0.0;

    // --- Hover margin ---
    // T_hover_thresh[i] = T_max * faulted[i].trim_utilization  (min T_max for fault-i hover)
    std::array<double, kNumRotors> T_hover_thresh{};
    double T_hover_worst = 0.0;
    // hover_margin = T_max / T_hover_worst - 1  (≥ 0 → all single faults feasible)
    double hover_margin = 0.0;

    // --- 2D hover slice metric ---
    // Minimum hover_slice_signed_distance across all single-rotor fault cases.
    // Positive = all faults can hover with (L=0,M=0) strictly inside their (L,M) envelope.
    double hover_slice_worst_fault_margin = 0.0;
};

class AttainableControlSetAnalyzer {
public:
    // Solve: find f s.t. B*f = u_req, 0 <= f <= f_max (LP via basis enumeration).
    // Returns minimum-sum feasible trim, or feasible=false if none exists.
    [[nodiscard]] AcsTrimSolution solveTrim(
        const AllocationMatrix& B,
        const std::array<double, kNumRotors>& f_max,
        const Eigen::Vector4d& u_req) const;

    // Generate all 2^n = 64 extreme thrust vertices mapped to 4D virtual-control space.
    // Row order: [Fz, L, M, N] matching B matrix row convention.
    [[nodiscard]] std::vector<Eigen::Vector4d> generateVertices(
        const AllocationMatrix& B,
        const std::array<double, kNumRotors>& f_max) const;

    // Zonotope edges: each motor varied across its range with all 2^(n-1) fixed combinations.
    // Returns 6 × 32 = 192 index pairs into the vertex list returned by generateVertices().
    [[nodiscard]] static std::vector<std::pair<int, int>> zonotopEdges();

    // Full ACS analysis: nominal + all single-rotor-fault cases.
    // Computes trim containment, directional margins, volume metrics, and hover margin.
    [[nodiscard]] AcsResult analyze(
        const AllocationMatrix& B,
        const std::array<AllocationMatrix, kNumRotors>& faulted_B,
        double thrust_max,
        double total_mass,
        double gravity) const;

private:
    // Analytic zonotope volume: V = Σ_{|I|=4} |det(B_I)| × Π f_max_i.
    // Exact equivalent of MATLAB convhulln on the 64-vertex ACS polytope.
    [[nodiscard]] double zonotopVolume(
        const AllocationMatrix& B,
        const std::array<double, kNumRotors>& f_max) const;

    // Support function h_U(d) = Σ_j max(0, d^T b_j) × f_max_j
    [[nodiscard]] double supportFunction(
        const AllocationMatrix& B,
        const std::array<double, kNumRotors>& f_max,
        const Eigen::Vector4d& d) const;

    [[nodiscard]] AcsCaseResult analyzeCase(
        const AllocationMatrix& B,
        const std::array<double, kNumRotors>& f_max,
        const Eigen::Vector4d& u_req) const;

    // BFS enumeration of the hover slice polytope {T : B.row(0)*T=u_req[0],
    // B.row(3)*T=u_req[3], 0≤T_i≤f_max_i}, projected onto (L,M)=(B.row(1)*T, B.row(2)*T).
    // Returns CCW convex hull polygon and signed distance from origin to polygon boundary.
    [[nodiscard]] std::pair<std::vector<Eigen::Vector2d>, double> computeHoverSlice(
        const AllocationMatrix& B,
        const std::array<double, kNumRotors>& f_max,
        const Eigen::Vector4d& u_req) const;
};

}  // namespace hexaarch::physics
