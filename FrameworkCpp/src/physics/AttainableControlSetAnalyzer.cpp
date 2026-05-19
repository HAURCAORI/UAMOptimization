#include "physics/AttainableControlSetAnalyzer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hexaarch::physics {
namespace {

// ---------------------------------------------------------------------------
// 2D convex hull (Graham scan) — returns CCW-ordered indices into pts.
// ---------------------------------------------------------------------------
std::vector<int> convexHull2d(const std::vector<Eigen::Vector2d>& pts) {
    const int n = static_cast<int>(pts.size());
    if (n < 3) {
        std::vector<int> r(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) r[static_cast<std::size_t>(i)] = i;
        return r;
    }
    int pivot = 0;
    for (int i = 1; i < n; ++i) {
        if (pts[i][1] < pts[pivot][1] ||
            (pts[i][1] == pts[pivot][1] && pts[i][0] < pts[pivot][0]))
            pivot = i;
    }
    std::vector<int> order;
    order.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) if (i != pivot) order.push_back(i);
    const Eigen::Vector2d& o = pts[pivot];
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        const Eigen::Vector2d da = pts[a] - o;
        const Eigen::Vector2d db = pts[b] - o;
        const double cross = da[0] * db[1] - da[1] * db[0];
        if (std::abs(cross) > 1e-12) return cross > 0.0;
        return da.squaredNorm() < db.squaredNorm();
    });
    std::vector<int> hull = {pivot, order[0]};
    for (int k = 1; k < static_cast<int>(order.size()); ++k) {
        while (hull.size() >= 2) {
            const Eigen::Vector2d a = pts[hull[hull.size()-1]] - pts[hull[hull.size()-2]];
            const Eigen::Vector2d b = pts[order[k]]            - pts[hull[hull.size()-2]];
            if (a[0]*b[1] - a[1]*b[0] <= 0.0) hull.pop_back();
            else break;
        }
        hull.push_back(order[k]);
    }
    return hull;
}

// Signed distance from the 2D origin (0,0) to a CCW convex polygon boundary.
// Returns +d if origin is strictly inside, −d if outside, 0 for degenerate input.
double signedDistFromOrigin(const std::vector<Eigen::Vector2d>& poly) {
    const int n = static_cast<int>(poly.size());
    if (n < 3) return 0.0;

    // Inside test: for CCW polygon, origin inside iff all edge cross products ≥ 0.
    // For edge (A, B) with P = origin: (B-A)×(P-A) = a[0]*b[1] - a[1]*b[0].
    bool inside = true;
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& a = poly.at(static_cast<std::size_t>(i));
        const Eigen::Vector2d& b = poly.at(static_cast<std::size_t>((i + 1) % n));
        if (a[0] * b[1] - a[1] * b[0] < -1e-10) { inside = false; break; }
    }

    // Distance from origin to nearest edge segment.
    double min_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector2d& a = poly.at(static_cast<std::size_t>(i));
        const Eigen::Vector2d& b = poly.at(static_cast<std::size_t>((i + 1) % n));
        const Eigen::Vector2d ab = b - a;
        const double len_sq = ab.squaredNorm();
        if (len_sq < 1e-20) { min_dist = std::min(min_dist, a.norm()); continue; }
        // Project origin onto segment: t = -(a·ab)/|ab|², clamped to [0,1].
        const double t = std::clamp(-a.dot(ab) / len_sq, 0.0, 1.0);
        min_dist = std::min(min_dist, (a + t * ab).norm());
    }

    if (!std::isfinite(min_dist) || min_dist < 1e-9) return 0.0;
    return inside ? min_dist : -min_dist;
}

struct DirectionSample {
    const char* label;
    Eigen::Vector4d d;
};

// 11 sampled directions in [Fz, Mx, My, Mz] space.
// Thrust row convention: B row 0 = -1 per rotor (upward thrust = negative Fz).
// The "useful thrust" direction for hover is [-1, 0, 0, 0].
std::array<DirectionSample, 11> makeDirectionSamples() {
    constexpr double s = 0.70710678118654752;  // 1/sqrt(2)
    return {{
        {"thrust",        Eigen::Vector4d{-1.0, 0.0, 0.0,  0.0}},
        {"roll_pos",      Eigen::Vector4d{ 0.0, 1.0, 0.0,  0.0}},
        {"roll_neg",      Eigen::Vector4d{ 0.0,-1.0, 0.0,  0.0}},
        {"pitch_pos",     Eigen::Vector4d{ 0.0, 0.0, 1.0,  0.0}},
        {"pitch_neg",     Eigen::Vector4d{ 0.0, 0.0,-1.0,  0.0}},
        {"yaw_pos",       Eigen::Vector4d{ 0.0, 0.0, 0.0,  1.0}},
        {"yaw_neg",       Eigen::Vector4d{ 0.0, 0.0, 0.0, -1.0}},
        {"roll_yaw_pp",   Eigen::Vector4d{ 0.0,  s,  0.0,  s  }},
        {"pitch_yaw_pp",  Eigen::Vector4d{ 0.0, 0.0,  s,   s  }},
        {"roll_yaw_pn",   Eigen::Vector4d{ 0.0,  s,  0.0, -s  }},
        {"pitch_yaw_pn",  Eigen::Vector4d{ 0.0, 0.0,  s,  -s  }},
    }};
}

}  // namespace

// ---------------------------------------------------------------------------
// Vertex generation
// ---------------------------------------------------------------------------

std::vector<Eigen::Vector4d> AttainableControlSetAnalyzer::generateVertices(
    const AllocationMatrix& B,
    const std::array<double, kNumRotors>& f_max) const {
    constexpr int n_vertices = 1 << kNumRotors;  // 2^6 = 64
    std::vector<Eigen::Vector4d> vertices;
    vertices.reserve(n_vertices);

    Eigen::Matrix<double, kNumRotors, 1> thrust;
    for (int mask = 0; mask < n_vertices; ++mask) {
        for (int k = 0; k < kNumRotors; ++k) {
            thrust(k) = (mask >> k & 1) ? f_max.at(static_cast<std::size_t>(k)) : 0.0;
        }
        vertices.push_back(B * thrust);
    }
    return vertices;
}

// ---------------------------------------------------------------------------
// Zonotope edges
// ---------------------------------------------------------------------------

std::vector<std::pair<int, int>> AttainableControlSetAnalyzer::zonotopEdges() {
    // Each edge varies motor k from 0 to T_max_k while all other motors are fixed.
    // For n=6 motors, this gives 6 × 2^5 = 192 edges.
    // Vertex index = binary mask: bit k = 0 means T_k=0, bit k = 1 means T_k=T_max_k.
    constexpr int n_vertices = 1 << kNumRotors;
    std::vector<std::pair<int, int>> edges;
    edges.reserve(kNumRotors * (n_vertices / 2));

    for (int k = 0; k < kNumRotors; ++k) {
        const int bit_k = 1 << k;
        for (int mask = 0; mask < n_vertices; ++mask) {
            if (!(mask & bit_k)) {
                // mask has bit k = 0; (mask | bit_k) has bit k = 1.
                edges.emplace_back(mask, mask | bit_k);
            }
        }
    }
    return edges;
}

// ---------------------------------------------------------------------------
// Analytic zonotope volume
// ---------------------------------------------------------------------------

double AttainableControlSetAnalyzer::zonotopVolume(
    const AllocationMatrix& B,
    const std::array<double, kNumRotors>& f_max) const {
    // Exact formula for the volume of a zonotope Z = {B*T : 0 ≤ T_i ≤ f_max_i}:
    //   V = Σ_{I ⊆ [n], |I|=d} |det(B_I)| × Π_{i∈I} f_max_i
    // where d = kNumControlDOF = 4, n = kNumRotors = 6.
    // This equals MATLAB's convhulln volume on the 64-vertex ACS polytope.
    // C(6,4) = 15 subsets → 15 4×4 determinants.
    double volume = 0.0;
    for (int a = 0; a < kNumRotors; ++a) {
        for (int b = a + 1; b < kNumRotors; ++b) {
            for (int c = b + 1; c < kNumRotors; ++c) {
                for (int d = c + 1; d < kNumRotors; ++d) {
                    Eigen::Matrix4d sub;
                    sub.col(0) = B.col(a);
                    sub.col(1) = B.col(b);
                    sub.col(2) = B.col(c);
                    sub.col(3) = B.col(d);
                    const double det_abs = std::abs(sub.determinant());
                    const double f_prod =
                        f_max.at(static_cast<std::size_t>(a)) *
                        f_max.at(static_cast<std::size_t>(b)) *
                        f_max.at(static_cast<std::size_t>(c)) *
                        f_max.at(static_cast<std::size_t>(d));
                    volume += det_abs * f_prod;
                }
            }
        }
    }
    return volume;
}

// ---------------------------------------------------------------------------
// Support function and trim solver
// ---------------------------------------------------------------------------

double AttainableControlSetAnalyzer::supportFunction(
    const AllocationMatrix& B,
    const std::array<double, kNumRotors>& f_max,
    const Eigen::Vector4d& d) const {
    double h = 0.0;
    for (int j = 0; j < kNumRotors; ++j) {
        const double proj = d.dot(B.col(j));
        h += std::max(0.0, proj) * f_max.at(static_cast<std::size_t>(j));
    }
    return h;
}

AcsTrimSolution AttainableControlSetAnalyzer::solveTrim(
    const AllocationMatrix& B,
    const std::array<double, kNumRotors>& f_max,
    const Eigen::Vector4d& u_req) const {
    AcsTrimSolution result;
    double best_sum = std::numeric_limits<double>::infinity();
    // Track best infeasible basis (lower-bound ok, upper-bound exceeded).
    // utilization > 1 means "need to scale T_max by this factor to reach feasibility."
    // This makes hover_margin continuous and differentiable for the optimizer.
    double best_infeasible_ratio = std::numeric_limits<double>::infinity();

    // Basis enumeration LP: iterate over all C(n,2) pairs of non-basic variables,
    // each pinned at their lower (0) or upper (f_max) bound. Remaining 4 variables
    // form the square basis system B_basis × x_basic = rhs.
    for (int i = 0; i < kNumRotors; ++i) {
        for (int j = i + 1; j < kNumRotors; ++j) {
            std::array<int, kNumControlDOF> basic{};
            int cursor = 0;
            for (int k = 0; k < kNumRotors; ++k) {
                if (k != i && k != j) {
                    basic.at(static_cast<std::size_t>(cursor++)) = k;
                }
            }

            for (int mask = 0; mask < 4; ++mask) {
                std::array<double, kNumRotors> candidate{};
                candidate.fill(0.0);
                Eigen::Vector4d rhs = u_req;

                candidate.at(static_cast<std::size_t>(i)) = (mask & 1) ? f_max.at(static_cast<std::size_t>(i)) : 0.0;
                candidate.at(static_cast<std::size_t>(j)) = (mask & 2) ? f_max.at(static_cast<std::size_t>(j)) : 0.0;
                rhs -= B.col(i) * candidate.at(static_cast<std::size_t>(i));
                rhs -= B.col(j) * candidate.at(static_cast<std::size_t>(j));

                Eigen::Matrix4d basis_matrix;
                for (int col = 0; col < kNumControlDOF; ++col) {
                    basis_matrix.col(col) = B.col(basic.at(static_cast<std::size_t>(col)));
                }

                const Eigen::FullPivLU<Eigen::Matrix4d> lu(basis_matrix);
                if (!lu.isInvertible()) {
                    continue;
                }

                const Eigen::Vector4d solution = lu.solve(rhs);

                // Separate lower-bound (negative thrust — physically invalid) from upper-bound
                // (exceeds T_max — T_max is too small but direction is valid).
                bool lower_ok = true;
                double max_ratio = 0.0;
                for (int col = 0; col < kNumControlDOF; ++col) {
                    const auto idx = static_cast<std::size_t>(basic.at(static_cast<std::size_t>(col)));
                    candidate.at(idx) = solution(col);
                    if (candidate.at(idx) < -1e-6) {
                        lower_ok = false;
                        break;
                    }
                    if (f_max.at(idx) > 1e-12) {
                        max_ratio = std::max(max_ratio, candidate.at(idx) / f_max.at(idx));
                    }
                }

                if (!lower_ok) {
                    continue;
                }

                if (max_ratio <= 1.0 + 1e-6) {
                    // Fully feasible: track minimum-sum solution.
                    double sum = 0.0;
                    for (int k = 0; k < kNumRotors; ++k) {
                        candidate.at(static_cast<std::size_t>(k)) =
                            std::clamp(candidate.at(static_cast<std::size_t>(k)), 0.0, f_max.at(static_cast<std::size_t>(k)));
                        sum += candidate.at(static_cast<std::size_t>(k));
                    }
                    if (sum < best_sum) {
                        result.thrust = candidate;
                        best_sum = sum;
                        result.feasible = true;
                    }
                } else if (!result.feasible && max_ratio < best_infeasible_ratio) {
                    // Upper-bound infeasible: track min max-ratio (how much to scale T_max).
                    best_infeasible_ratio = max_ratio;
                }
            }
        }
    }

    if (result.feasible) {
        result.utilization = 0.0;
        for (int k = 0; k < kNumRotors; ++k) {
            const auto idx = static_cast<std::size_t>(k);
            if (f_max.at(idx) > 1e-12) {
                result.utilization = std::max(result.utilization, result.thrust.at(idx) / f_max.at(idx));
            }
        }
    } else if (best_infeasible_ratio < std::numeric_limits<double>::infinity()) {
        // Return continuous utilization > 1: T_max / T_hover_thresh = 1 / utilization < 1.
        // hover_margin = 1/utilization - 1 < 0, giving the optimizer a gradient toward T_max ↑.
        result.utilization = best_infeasible_ratio;
    }

    return result;
}

// ---------------------------------------------------------------------------
// 2D hover slice — BFS vertex enumeration
// ---------------------------------------------------------------------------

std::pair<std::vector<Eigen::Vector2d>, double>
AttainableControlSetAnalyzer::computeHoverSlice(
    const AllocationMatrix& B,
    const std::array<double, kNumRotors>& f_max,
    const Eigen::Vector4d& u_req) const {

    // Active motors only (f_max > 0).
    std::vector<int> active;
    for (int k = 0; k < kNumRotors; ++k) {
        if (f_max.at(static_cast<std::size_t>(k)) > 1e-12) active.push_back(k);
    }
    const int na = static_cast<int>(active.size());
    if (na < 2) return {{}, 0.0};

    // n_nb non-basic variables each pinned at 0 or f_max; 2 basic variables solve
    // the 2×2 equality system (thrust row and yaw row of B).
    const int n_nb = na - 2;
    const int n_masks = 1 << n_nb;
    constexpr double kFeasTol = 1e-6;
    constexpr double kDedupTol = 1e-3;

    std::vector<Eigen::Vector2d> raw;
    raw.reserve(static_cast<std::size_t>((na * (na - 1) / 2) * n_masks));

    for (int bi = 0; bi < na; ++bi) {
        for (int bj = bi + 1; bj < na; ++bj) {
            const int gi = active.at(static_cast<std::size_t>(bi));
            const int gj = active.at(static_cast<std::size_t>(bj));

            // 2×2 basis from rows 0 (thrust) and 3 (yaw).
            Eigen::Matrix2d basis;
            basis(0, 0) = B(0, gi); basis(0, 1) = B(0, gj);
            basis(1, 0) = B(3, gi); basis(1, 1) = B(3, gj);
            const Eigen::FullPivLU<Eigen::Matrix2d> lu(basis);
            if (!lu.isInvertible()) continue;

            // Non-basic global motor indices.
            std::vector<int> nb;
            nb.reserve(static_cast<std::size_t>(n_nb));
            for (int k = 0; k < na; ++k) {
                if (k != bi && k != bj) nb.push_back(active.at(static_cast<std::size_t>(k)));
            }

            for (int mask = 0; mask < n_masks; ++mask) {
                Eigen::Vector2d rhs{u_req[0], u_req[3]};
                std::array<double, kNumRotors> T{};

                for (int nk = 0; nk < n_nb; ++nk) {
                    const int gk = nb.at(static_cast<std::size_t>(nk));
                    const double val = (mask >> nk & 1) ? f_max.at(static_cast<std::size_t>(gk)) : 0.0;
                    T.at(static_cast<std::size_t>(gk)) = val;
                    rhs[0] -= B(0, gk) * val;
                    rhs[1] -= B(3, gk) * val;
                }

                const Eigen::Vector2d sol = lu.solve(rhs);
                const double fi_max = f_max.at(static_cast<std::size_t>(gi));
                const double fj_max = f_max.at(static_cast<std::size_t>(gj));
                if (sol[0] < -kFeasTol || sol[0] > fi_max + kFeasTol) continue;
                if (sol[1] < -kFeasTol || sol[1] > fj_max + kFeasTol) continue;

                T.at(static_cast<std::size_t>(gi)) = std::clamp(sol[0], 0.0, fi_max);
                T.at(static_cast<std::size_t>(gj)) = std::clamp(sol[1], 0.0, fj_max);

                double L = 0.0, M = 0.0;
                for (int k = 0; k < kNumRotors; ++k) {
                    L += B(1, k) * T.at(static_cast<std::size_t>(k));
                    M += B(2, k) * T.at(static_cast<std::size_t>(k));
                }
                raw.push_back({L, M});
            }
        }
    }

    if (raw.size() < 3) return {raw, 0.0};

    // Deduplicate within a small absolute tolerance.
    std::vector<Eigen::Vector2d> uniq;
    uniq.reserve(raw.size());
    for (const auto& p : raw) {
        bool dup = false;
        for (const auto& u : uniq) {
            if ((p - u).squaredNorm() < kDedupTol * kDedupTol) { dup = true; break; }
        }
        if (!dup) uniq.push_back(p);
    }
    if (uniq.size() < 3) return {uniq, 0.0};

    // Build CCW convex hull and compute signed distance from origin.
    std::vector<Eigen::Vector2d> poly;
    poly.reserve(uniq.size());
    for (int idx : convexHull2d(uniq)) poly.push_back(uniq.at(static_cast<std::size_t>(idx)));

    const double sd = (poly.size() >= 3) ? signedDistFromOrigin(poly) : 0.0;
    return {std::move(poly), sd};
}

// ---------------------------------------------------------------------------
// Per-case analysis
// ---------------------------------------------------------------------------

AcsCaseResult AttainableControlSetAnalyzer::analyzeCase(
    const AllocationMatrix& B,
    const std::array<double, kNumRotors>& f_max,
    const Eigen::Vector4d& u_req) const {
    AcsCaseResult result;

    const AcsTrimSolution trim = solveTrim(B, f_max, u_req);
    result.trim_feasible = trim.feasible;
    result.trim_utilization = trim.utilization;
    result.trim_thrust = trim.thrust;

    const auto samples = makeDirectionSamples();
    result.min_margin = std::numeric_limits<double>::infinity();

    for (const auto& sample : samples) {
        const double h = supportFunction(B, f_max, sample.d);
        const double dTu = sample.d.dot(u_req);
        const double margin = h - dTu;
        result.directional_margins.push_back({sample.label, margin});
        result.min_margin = std::min(result.min_margin, margin);
    }

    // Extract named per-axis reserves for §9.8 reporting.
    for (const auto& dm : result.directional_margins) {
        if (dm.label == "yaw_pos")   result.yaw_reserve   = dm.margin;
        if (dm.label == "roll_pos")  result.roll_reserve  = dm.margin;
        if (dm.label == "pitch_pos") result.pitch_reserve = dm.margin;
    }

    return result;
}

// ---------------------------------------------------------------------------
// Full ACS analysis
// ---------------------------------------------------------------------------

AcsResult AttainableControlSetAnalyzer::analyze(
    const AllocationMatrix& B,
    const std::array<AllocationMatrix, kNumRotors>& faulted_B,
    const double thrust_max,
    const double total_mass,
    const double gravity) const {
    AcsResult result;

    // u_req convention: B row 0 = -1 for all rotors, hover needs Fz = -m*g.
    const Eigen::Vector4d u_req{-total_mass * gravity, 0.0, 0.0, 0.0};

    std::array<double, kNumRotors> f_max_nominal{};
    f_max_nominal.fill(thrust_max);

    // --- Nominal case ---
    result.nominal = analyzeCase(B, f_max_nominal, u_req);
    result.volume_nominal = zonotopVolume(B, f_max_nominal);
    {
        auto [poly, sd] = computeHoverSlice(B, f_max_nominal, u_req);
        result.nominal.hover_slice_polygon = std::move(poly);
        result.nominal.hover_slice_signed_distance = sd;
    }

    // --- Single-rotor fault cases ---
    result.worst_fault_min_margin = std::numeric_limits<double>::infinity();
    double worst_slice_margin = std::numeric_limits<double>::infinity();

    for (int i = 0; i < kNumRotors; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        // Rotor i fully failed: T_max_i = 0.
        // faulted_B[i] has column i zeroed; set f_max[i] = 0 to match.
        std::array<double, kNumRotors> f_max_fault = f_max_nominal;
        f_max_fault.at(idx) = 0.0;

        result.faulted.at(idx) = analyzeCase(faulted_B.at(idx), f_max_fault, u_req);
        {
            auto [poly, sd] = computeHoverSlice(faulted_B.at(idx), f_max_fault, u_req);
            result.faulted.at(idx).hover_slice_polygon = std::move(poly);
            result.faulted.at(idx).hover_slice_signed_distance = sd;
            worst_slice_margin = std::min(worst_slice_margin, sd);
        }
        result.volume_faulted.at(idx) = zonotopVolume(faulted_B.at(idx), f_max_fault);

        if (result.volume_nominal > 0.0) {
            result.retention.at(idx) = result.volume_faulted.at(idx) / result.volume_nominal;
        }

        result.worst_fault_min_margin =
            std::min(result.worst_fault_min_margin, result.faulted.at(idx).min_margin);

        // T_hover_thresh[i] = T_max × utilization_at_hover_under_fault_i.
        // utilization <= 1 when feasible, > 1 when infeasible (continuous — enables gradient).
        const double util = result.faulted.at(idx).trim_utilization;
        result.T_hover_thresh.at(idx) = (util > 1e-12)
            ? thrust_max * util
            : std::numeric_limits<double>::infinity();  // degenerate: no valid LP direction
    }

    result.overall_min_margin = std::min(result.nominal.min_margin, result.worst_fault_min_margin);

    // How much directional margin is retained under the worst single-motor fault.
    const double nm = result.nominal.min_margin;
    result.faulted_to_nominal_ratio = nm > 1e-9
        ? result.worst_fault_min_margin / nm
        : (result.worst_fault_min_margin >= 0.0 ? 1.0 : 0.0);

    // --- Aggregate volume metrics (matching MATLAB eval_acs.m) ---
    // PFWAR: equal-weight average over single-motor faults (p_fail normalized → 1/6 each)
    double retention_sum = 0.0;
    double retention_sq_sum = 0.0;
    double retention_min = std::numeric_limits<double>::max();
    for (int i = 0; i < kNumRotors; ++i) {
        const double r = result.retention.at(static_cast<std::size_t>(i));
        retention_sum += r;
        retention_sq_sum += r * r;
        retention_min = std::min(retention_min, r);
    }
    const double n = static_cast<double>(kNumRotors);
    const double mean_r = retention_sum / n;
    result.PFWAR = mean_r;
    result.WCFR = retention_min;

    // FII = std(retention) / mean(retention)
    if (mean_r > 0.0) {
        const double variance = retention_sq_sum / n - mean_r * mean_r;
        const double std_r = std::sqrt(std::max(0.0, variance));
        result.FII = std_r / mean_r;
    }

    // Hover margin: T_max / T_hover_worst - 1
    result.T_hover_worst = *std::max_element(result.T_hover_thresh.begin(), result.T_hover_thresh.end());
    if (result.T_hover_worst > 0.0 && result.T_hover_worst < std::numeric_limits<double>::infinity()) {
        result.hover_margin = thrust_max / result.T_hover_worst - 1.0;
    } else {
        result.hover_margin = result.T_hover_worst == 0.0
            ? std::numeric_limits<double>::infinity()
            : -std::numeric_limits<double>::infinity();
    }

    result.hover_slice_worst_fault_margin =
        std::isfinite(worst_slice_margin) ? worst_slice_margin : 0.0;

    return result;
}

}  // namespace hexaarch::physics
