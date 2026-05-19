#include "analysis/AcsPlotter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace hexaarch::analysis {
namespace {

// ---------------------------------------------------------------------------
// SVG canvas layout (all units: px)
// ---------------------------------------------------------------------------
constexpr double kW  = 800.0;
constexpr double kH  = 580.0;
constexpr double kML = 80.0;             // margin left
constexpr double kMR = 140.0;            // margin right (legend / ref-line labels)
constexpr double kMT = 50.0;             // margin top
constexpr double kMB = 65.0;             // margin bottom
constexpr double kPW = kW - kML - kMR;  // plot width
constexpr double kPH = kH - kMT - kMB;  // plot height

// ---------------------------------------------------------------------------
// Color palette (SVG hex strings)
// ---------------------------------------------------------------------------
constexpr const char* kColBlue      = "#3480E6";
constexpr const char* kColRed       = "#F24F33";
constexpr const char* kColSteelBlue = "#4D99E8";
constexpr const char* kColNavy      = "#1A4D99";
constexpr const char* kColDarkRed   = "#B21A0D";

// ---------------------------------------------------------------------------
// Data helpers
// ---------------------------------------------------------------------------
bool nearlyEqual(const Eigen::Vector4d& a, const Eigen::Vector4d& b, double tol) {
    return (a - b).cwiseAbs().maxCoeff() < tol;
}

void deduplicateVertices(std::vector<Eigen::Vector4d>& pts) {
    constexpr double kTol = 1e-9;
    std::vector<Eigen::Vector4d> unique;
    unique.reserve(pts.size());
    for (const auto& p : pts) {
        bool dup = false;
        for (const auto& u : unique)
            if (nearlyEqual(p, u, kTol)) { dup = true; break; }
        if (!dup) unique.push_back(p);
    }
    pts = std::move(unique);
}

std::vector<double> col4(const std::vector<Eigen::Vector4d>& pts, int c) {
    std::vector<double> v;
    v.reserve(pts.size());
    for (const auto& p : pts) v.push_back(p[c]);
    return v;
}

std::vector<double> col2(const std::vector<Eigen::Vector2d>& pts, int c) {
    std::vector<double> v;
    v.reserve(pts.size());
    for (const auto& p : pts) v.push_back(p[c]);
    return v;
}

// ---------------------------------------------------------------------------
// Linear scale: maps a data range to a pixel range
// ---------------------------------------------------------------------------
struct Scale {
    double lo, hi;    // data extents (after padding)
    double px_lo, px_hi;  // pixel extents

    double map(double v) const {
        if (hi == lo) return (px_lo + px_hi) * 0.5;
        return px_lo + (v - lo) / (hi - lo) * (px_hi - px_lo);
    }
};

Scale makeScale(const std::vector<double>& a, const std::vector<double>& b,
                double px_lo, double px_hi) {
    // Use (std::numeric_limits<...>::max)() — parentheses prevent Windows min/max macro expansion.
    double lo = (std::numeric_limits<double>::max)();
    double hi = -(std::numeric_limits<double>::max)();
    for (double v : a) { if (v < lo) lo = v; if (v > hi) hi = v; }
    for (double v : b) { if (v < lo) lo = v; if (v > hi) hi = v; }
    const double pad = (hi > lo) ? (hi - lo) * 0.07 : 1.0;
    return {lo - pad, hi + pad, px_lo, px_hi};
}

Scale makeScale1(const std::vector<double>& a, double px_lo, double px_hi) {
    return makeScale(a, {}, px_lo, px_hi);
}

std::string fmtd(double v, int prec = 2) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(prec) << v;
    return ss.str();
}

// ---------------------------------------------------------------------------
// Minimal SVG builder
// ---------------------------------------------------------------------------
class Svg {
    std::ostringstream _s;
public:
    Svg(double w, double h) {
        _s << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
           << "<svg xmlns=\"http://www.w3.org/2000/svg\""
           << " width=\"" << w << "\" height=\"" << h << "\""
           << " viewBox=\"0 0 " << w << " " << h << "\">\n"
           << "<style>text{font-family:sans-serif;}</style>\n";
    }

    void rect(double x, double y, double w, double h,
              const char* fill, const char* stroke = "none",
              double sw = 1.0, double fill_op = 1.0) {
        _s << "<rect x=\"" << x << "\" y=\"" << y
           << "\" width=\"" << w << "\" height=\"" << h << "\""
           << " fill=\"" << fill << "\" fill-opacity=\"" << fill_op << "\""
           << " stroke=\"" << stroke << "\" stroke-width=\"" << sw << "\"/>\n";
    }

    void circle(double cx, double cy, double r,
                const char* fill, double op = 1.0) {
        _s << "<circle cx=\"" << cx << "\" cy=\"" << cy
           << "\" r=\"" << r << "\" fill=\"" << fill
           << "\" opacity=\"" << op << "\"/>\n";
    }

    void line(double x1, double y1, double x2, double y2,
              const char* stroke, double sw = 1.0,
              const char* dash = nullptr) {
        _s << "<line x1=\"" << x1 << "\" y1=\"" << y1
           << "\" x2=\"" << x2 << "\" y2=\"" << y2 << "\""
           << " stroke=\"" << stroke << "\" stroke-width=\"" << sw << "\"";
        if (dash) _s << " stroke-dasharray=\"" << dash << "\"";
        _s << "/>\n";
    }

    void polygon(const std::vector<std::pair<double,double>>& pts,
                 const char* fill, const char* stroke,
                 double sw = 1.5, double fill_op = 0.3) {
        if (pts.empty()) return;
        _s << "<polygon points=\"";
        for (const auto& [x, y] : pts) _s << x << "," << y << " ";
        _s << "\" fill=\"" << fill << "\" fill-opacity=\"" << fill_op << "\""
           << " stroke=\"" << stroke << "\" stroke-width=\"" << sw << "\"/>\n";
    }

    void polyline(const std::vector<std::pair<double,double>>& pts,
                  const char* stroke, double sw = 1.5) {
        if (pts.empty()) return;
        _s << "<polyline points=\"";
        for (const auto& [x, y] : pts) _s << x << "," << y << " ";
        _s << "\" fill=\"none\" stroke=\"" << stroke
           << "\" stroke-width=\"" << sw << "\"/>\n";
    }

    void text(double x, double y, const std::string& s,
              double sz = 12.0, const char* anchor = "middle",
              const char* fill = "#333333") {
        _s << "<text x=\"" << x << "\" y=\"" << y << "\""
           << " font-size=\"" << sz << "\""
           << " text-anchor=\"" << anchor << "\""
           << " fill=\"" << fill << "\">" << s << "</text>\n";
    }

    // Rotated label for y-axis
    void textRotated(double x, double y, const std::string& s, double sz = 12.0) {
        _s << "<text x=\"" << x << "\" y=\"" << y << "\""
           << " font-size=\"" << sz << "\" text-anchor=\"middle\" fill=\"#333333\""
           << " transform=\"rotate(-90," << x << "," << y << ")\">"
           << s << "</text>\n";
    }

    // Angled tick label (e.g. -35° for x-axis direction labels)
    void textAngled(double x, double y, const std::string& s,
                    double angle_deg = -35.0, double sz = 9.0) {
        _s << "<text x=\"" << x << "\" y=\"" << y << "\""
           << " font-size=\"" << sz << "\" text-anchor=\"end\" fill=\"#555555\""
           << " transform=\"rotate(" << angle_deg << "," << x << "," << y << ")\">"
           << s << "</text>\n";
    }

    // Draw plot frame, title, axis labels, and numeric tick marks.
    // sx/sy: data→pixel scales. sy has px_lo at bottom, px_hi at top (y flipped).
    void axes(const Scale& sx, const Scale& sy,
              const std::string& xl, const std::string& yl,
              const std::string& title, int n_ticks = 5) {
        // Frame
        rect(kML, kMT, kPW, kPH, "#F8F8F8", "#BBBBBB");

        // Title
        text(kML + kPW * 0.5, kMT - 14, title, 13, "middle", "#111111");

        // Axis labels
        text(kML + kPW * 0.5, kH - 8, xl, 12, "middle", "#333333");
        textRotated(14, kMT + kPH * 0.5, yl, 12);

        // X ticks + gridlines
        for (int i = 0; i <= n_ticks; ++i) {
            const double t = sx.lo + (sx.hi - sx.lo) * i / n_ticks;
            const double px = sx.map(t);
            line(px, kMT + kPH, px, kMT + kPH + 5, "#666666");
            line(px, kMT, px, kMT + kPH, "#DDDDDD", 0.5);
            text(px, kMT + kPH + 18, fmtd(t), 10, "middle", "#555555");
        }

        // Y ticks + gridlines
        for (int i = 0; i <= n_ticks; ++i) {
            const double t = sy.lo + (sy.hi - sy.lo) * i / n_ticks;
            const double py = sy.map(t);
            line(kML - 5, py, kML, py, "#666666");
            line(kML, py, kML + kPW, py, "#DDDDDD", 0.5);
            text(kML - 7, py + 4, fmtd(t), 10, "end", "#555555");
        }
    }

    // Small legend swatch + label
    void legendSwatch(double x, double y, const char* color, const std::string& label) {
        rect(x, y - 9, 14, 11, color, "none", 0, 0.85);
        text(x + 18, y, label, 11, "start", "#333333");
    }

    void close() { _s << "</svg>\n"; }
    std::string str() const { return _s.str(); }
};

void writeSvg(const std::string& path, const std::string& content) {
    std::ofstream f(path);
    f << content;
}

}  // namespace

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------

void AcsPlotter::plot(
    const physics::AllocationMatrix& B,
    const double thrust_max,
    const double vehicle_weight,
    const physics::AcsResult& acs,
    const Config& config) const {

    // Worst fault = smallest 4D directional margin (matches acs::fault_directional_margin constraint).
    // hover_slice_signed_distance is intentionally not used here: motor-2 fault always yields
    // signed_distance = 0 (geometric invariant), making it an uninformative selection criterion.
    const auto worst_it = std::min_element(
        acs.faulted.begin(), acs.faulted.end(),
        [](const physics::AcsCaseResult& a, const physics::AcsCaseResult& b) {
            return a.min_margin < b.min_margin;
        });
    const int fi = static_cast<int>(std::distance(acs.faulted.begin(), worst_it));

    physics::AttainableControlSetAnalyzer analyzer;

    std::array<double, physics::kNumRotors> f_nom{};
    f_nom.fill(thrust_max);
    std::array<double, physics::kNumRotors> f_fault = f_nom;
    f_fault.at(static_cast<std::size_t>(fi)) = 0.0;

    physics::AllocationMatrix B_fault = B;
    B_fault.col(fi).setZero();

    auto verts_nom   = analyzer.generateVertices(B,       f_nom);
    auto verts_fault = analyzer.generateVertices(B_fault, f_fault);
    const auto edges = physics::AttainableControlSetAnalyzer::zonotopEdges();

    double max_abs = 0.0;
    for (const auto& v : verts_nom) max_abs = std::max(max_abs, v.cwiseAbs().maxCoeff());
    const double tol = std::max(1e-6 * max_abs, 1e-6);

    const double W = vehicle_weight;
    const std::string& dir = config.output_dir;
    const std::string lbl = config.label + " [worst: M" + std::to_string(fi + 1) + "]";

    // Fig 1: L vs M projection of full vertex cloud
    scatter2dSvg(verts_nom, verts_fault, 1, 2,
        "L [Nm]", "M [Nm]",
        lbl + " | ACS vertices [L vs M]",
        dir + "/acs_fig1_lm.svg");

    // Fig 2: L vs Fz projection of full vertex cloud
    scatter2dSvg(verts_nom, verts_fault, 1, 0,
        "L [Nm]", "Fz [N]",
        lbl + " | ACS vertices [L vs Fz]",
        dir + "/acs_fig2_lfz.svg");

    // Hover plane slice at Fz = -W
    const auto slice_Fz_nom   = sliceAt(verts_nom,   edges, 0, -W, tol);
    const auto slice_Fz_fault = sliceAt(verts_fault, edges, 0, -W, tol);

    // Fig 3: hover-plane slice L vs M
    scatter2dSvg(slice_Fz_nom, slice_Fz_fault, 1, 2,
        "L [Nm]", "M [Nm]",
        lbl + " | Hover slice [L vs M] at Fz=-W",
        dir + "/acs_fig3_hover_lm.svg");

    // Fig 4: N=0 slice L vs Fz
    const auto slice_N_nom   = sliceAt(verts_nom,   edges, 3, 0.0, tol);
    const auto slice_N_fault = sliceAt(verts_fault, edges, 3, 0.0, tol);
    scatter2dSvg(slice_N_nom, slice_N_fault, 1, 0,
        "L [Nm]", "Fz [N]",
        lbl + " | N=0 slice [L vs Fz]",
        dir + "/acs_fig4_n0_lfz.svg");

    // Fig 5: exact BFS hover slice polygon stored in AcsCaseResult.
    // Close each polygon for the SVG polyline outline (polygon primitive auto-closes).
    auto closePoly = [](std::vector<Eigen::Vector2d> poly) {
        if (!poly.empty()) poly.push_back(poly.front());
        return poly;
    };
    hoverPolygonSvg(
        closePoly(acs.nominal.hover_slice_polygon),
        closePoly(acs.faulted.at(static_cast<std::size_t>(fi)).hover_slice_polygon),
        acs.nominal.min_margin,
        acs.faulted.at(static_cast<std::size_t>(fi)).min_margin,
        "L [Nm]", "M [Nm]",
        lbl + " | Hover moment polygon (Fz=-W, N=0)",
        dir + "/acs_fig5_hover_poly.svg");

    // Fig 6: directional margin comparison (nominal vs worst-fault)
    marginBarsSvg(acs.nominal, acs.faulted.at(static_cast<std::size_t>(fi)), fi,
        lbl + " | Directional Margins at Hover",
        dir + "/acs_fig6_margins.svg");

    // Retention bar chart
    retentionBarSvg(acs, fi,
        config.label + " | Per-motor ACS Retention",
        dir + "/acs_retention.svg");

    // Full 4D vertex export for offline 3D analysis
    exportVerticesCsv(verts_nom, verts_fault, fi, dir + "/acs_vertices.csv");
}

// ---------------------------------------------------------------------------
// Slice algorithm (port of MATLAB slice_at.m)
// ---------------------------------------------------------------------------

std::vector<Eigen::Vector4d> AcsPlotter::sliceAt(
    const std::vector<Eigen::Vector4d>& pts,
    const std::vector<std::pair<int, int>>& edges,
    const int dim_cut,
    const double val_cut,
    const double tol) const {
    std::vector<Eigen::Vector4d> result;
    for (const auto& [ia, ib] : edges) {
        const Eigen::Vector4d& p1 = pts.at(static_cast<std::size_t>(ia));
        const Eigen::Vector4d& p2 = pts.at(static_cast<std::size_t>(ib));
        const double v1 = p1[dim_cut] - val_cut;
        const double v2 = p2[dim_cut] - val_cut;
        if (std::abs(v1) < tol && std::abs(v2) < tol) {
            result.push_back(p1);
            result.push_back(p2);
        } else if (std::abs(v1) < tol) {
            result.push_back(p1);
        } else if (std::abs(v2) < tol) {
            result.push_back(p2);
        } else if (v1 * v2 < 0.0) {
            const double alpha = -v1 / (v2 - v1);
            result.push_back(p1 + alpha * (p2 - p1));
        }
    }
    deduplicateVertices(result);
    return result;
}

// ---------------------------------------------------------------------------
// 2D convex hull (Graham scan)
// ---------------------------------------------------------------------------

std::vector<int> AcsPlotter::convexHull2d(const std::vector<Eigen::Vector2d>& pts) const {
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

// ---------------------------------------------------------------------------
// SVG renderers
// ---------------------------------------------------------------------------

void AcsPlotter::scatter2dSvg(
    const std::vector<Eigen::Vector4d>& nom,
    const std::vector<Eigen::Vector4d>& fault,
    const int cx, const int cy,
    const std::string& xl, const std::string& yl,
    const std::string& title_str,
    const std::string& path) const {

    const auto xn = col4(nom,   cx), yn = col4(nom,   cy);
    const auto xf = col4(fault, cx), yf = col4(fault, cy);

    std::vector<double> allx, ally;
    allx.insert(allx.end(), xn.begin(), xn.end());
    allx.insert(allx.end(), xf.begin(), xf.end());
    ally.insert(ally.end(), yn.begin(), yn.end());
    ally.insert(ally.end(), yf.begin(), yf.end());
    if (allx.empty()) return;

    const Scale sx = makeScale1(allx, kML, kML + kPW);
    const Scale sy = makeScale1(ally, kMT + kPH, kMT);  // y-axis flipped (SVG origin top-left)

    Svg svg(kW, kH);
    svg.axes(sx, sy, xl, yl, title_str);

    for (std::size_t i = 0; i < xn.size(); ++i)
        svg.circle(sx.map(xn[i]), sy.map(yn[i]), 2.5, kColBlue, 0.55);
    for (std::size_t i = 0; i < xf.size(); ++i)
        svg.circle(sx.map(xf[i]), sy.map(yf[i]), 2.5, kColRed, 0.55);

    const double lx = kML + kPW + 12;
    svg.legendSwatch(lx, kMT + 22, kColBlue, "Nominal");
    svg.legendSwatch(lx, kMT + 42, kColRed,  "Worst Fault");

    svg.close();
    writeSvg(path, svg.str());
}

void AcsPlotter::hoverPolygonSvg(
    const std::vector<Eigen::Vector2d>& hull_nom,
    const std::vector<Eigen::Vector2d>& hull_fault,
    const double nom_min_margin,
    const double fault_min_margin,
    const std::string& xl, const std::string& yl,
    const std::string& title_str,
    const std::string& path) const {

    const auto xn = col2(hull_nom,   0), yn = col2(hull_nom,   1);
    const auto xf = col2(hull_fault, 0), yf = col2(hull_fault, 1);

    // Always include origin in the scale so the hover operating point (L=0, M=0) is visible.
    std::vector<double> allx = {0.0}, ally = {0.0};
    allx.insert(allx.end(), xn.begin(), xn.end());
    allx.insert(allx.end(), xf.begin(), xf.end());
    ally.insert(ally.end(), yn.begin(), yn.end());
    ally.insert(ally.end(), yf.begin(), yf.end());
    if (allx.size() <= 1) return;  // only origin — no polygon data

    const Scale sx = makeScale1(allx, kML, kML + kPW);
    const Scale sy = makeScale1(ally, kMT + kPH, kMT);

    Svg svg(kW, kH);
    svg.axes(sx, sy, xl, yl, title_str);

    auto toPixels = [&](const std::vector<double>& xs, const std::vector<double>& ys) {
        std::vector<std::pair<double,double>> pxs;
        pxs.reserve(xs.size());
        for (std::size_t i = 0; i < xs.size(); ++i)
            pxs.push_back({sx.map(xs[i]), sy.map(ys[i])});
        return pxs;
    };

    if (!hull_nom.empty()) {
        svg.polygon(toPixels(xn, yn), kColBlue, kColBlue, 1.5, 0.22);
        svg.polyline(toPixels(xn, yn), kColBlue, 1.8);
    }
    if (!hull_fault.empty()) {
        svg.polygon(toPixels(xf, yf), kColRed, kColRed, 1.5, 0.32);
        svg.polyline(toPixels(xf, yf), kColRed, 1.8);
    }

    // Crosshairs at hover operating point (L=0, M=0) — always draw, origin is forced in-range.
    const double ox = sx.map(0.0);
    const double oy = sy.map(0.0);
    svg.line(ox, kMT, ox, kMT + kPH, "#888888", 0.8, "4,3");
    svg.line(kML, oy, kML + kPW, oy, "#888888", 0.8, "4,3");

    // Hover operating point marker
    constexpr const char* kColGreen = "#22AA44";
    svg.circle(ox, oy, 5.0, kColGreen, 0.9);

    // 4D directional margin annotations (N·m): min over 11 sampled directions at hover trim.
    // Matches the acs::fault_directional_margin hard constraint metric.
    const double ax = kML + kPW - 6;
    const double ay = kMT + kPH - 36;
    svg.text(ax, ay,      "nom 4D margin: "   + fmtd(nom_min_margin,   1) + " Nm", 10, "end", "#333333");
    svg.text(ax, ay + 16, "fault 4D margin: " + fmtd(fault_min_margin, 1) + " Nm", 10, "end", "#333333");

    const double lx = kML + kPW + 12;
    svg.legendSwatch(lx, kMT + 22, kColBlue,  "Nominal");
    svg.legendSwatch(lx, kMT + 42, kColRed,   "Worst Fault");
    svg.legendSwatch(lx, kMT + 62, kColGreen, "Hover req.");

    svg.close();
    writeSvg(path, svg.str());
}

void AcsPlotter::retentionBarSvg(
    const physics::AcsResult& acs,
    const int worst_motor,
    const std::string& title_str,
    const std::string& path) const {

    constexpr int kN = static_cast<int>(physics::kNumRotors);
    // Fixed y range 0..1.15 for retention values
    const Scale sy{0.0, 1.15, kMT + kPH, kMT};

    const double slot  = kPW / kN;
    const double bar_w = slot * 0.62;

    Svg svg(kW, kH);

    // Frame
    svg.rect(kML, kMT, kPW, kPH, "#F8F8F8", "#BBBBBB");
    svg.text(kML + kPW * 0.5, kMT - 14, title_str, 13, "middle", "#111111");
    svg.text(kML + kPW * 0.5, kH - 8,   "Motor",    12, "middle", "#333333");
    svg.textRotated(14, kMT + kPH * 0.5, "ACS Retention", 12);

    // Horizontal gridlines + y tick labels
    for (int i = 0; i <= 5; ++i) {
        const double t  = i * 0.2;
        const double py = sy.map(t);
        svg.line(kML - 5, py, kML, py, "#666666");
        svg.line(kML, py, kML + kPW, py, "#DDDDDD", 0.5);
        svg.text(kML - 7, py + 4, fmtd(t, 1), 10, "end", "#555555");
    }

    // Bars
    for (int i = 0; i < kN; ++i) {
        const double ret  = acs.retention.at(static_cast<std::size_t>(i));
        const double bx   = kML + slot * i + (slot - bar_w) * 0.5;
        const double py_t = sy.map(ret);
        const double py_b = sy.map(0.0);
        const bool worst  = (i == worst_motor);
        svg.rect(bx, py_t, bar_w, py_b - py_t,
                 worst ? kColRed       : kColSteelBlue,
                 worst ? kColDarkRed   : kColNavy, 1.0);
        svg.text(bx + bar_w * 0.5, py_b + 16,
                 "M" + std::to_string(i + 1), 11, "middle", "#333333");
    }

    // PFWAR reference line (red dashed)
    {
        const double py = sy.map(acs.PFWAR);
        svg.line(kML, py, kML + kPW, py, kColRed, 1.5, "6,4");
        svg.text(kML + kPW + 5, py + 4, "PFWAR", 10, "start", kColRed);
    }

    // WCFR reference line (navy dotted)
    {
        const double py = sy.map(acs.WCFR);
        svg.line(kML, py, kML + kPW, py, kColNavy, 1.5, "2,3");
        svg.text(kML + kPW + 5, py + 4, "WCFR", 10, "start", kColNavy);
    }

    svg.close();
    writeSvg(path, svg.str());
}

// ---------------------------------------------------------------------------
// Directional margin grouped bar chart (nominal vs worst-fault)
// ---------------------------------------------------------------------------

void AcsPlotter::marginBarsSvg(
    const physics::AcsCaseResult& nominal,
    const physics::AcsCaseResult& worst_fault,
    const int fault_motor,
    const std::string& title_str,
    const std::string& path) const {

    const int nd = static_cast<int>(nominal.directional_margins.size());
    if (nd == 0) return;
    if (static_cast<int>(worst_fault.directional_margins.size()) < nd) return;

    std::vector<double> m_nom(static_cast<std::size_t>(nd));
    std::vector<double> m_flt(static_cast<std::size_t>(nd));
    std::vector<std::string> labels(static_cast<std::size_t>(nd));

    for (int i = 0; i < nd; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        m_nom[idx]  = nominal.directional_margins[idx].margin;
        m_flt[idx]  = worst_fault.directional_margins[idx].margin;
        labels[idx] = nominal.directional_margins[idx].label;
    }

    // Y scale: covers all margins and always includes 0.
    std::vector<double> all_m;
    all_m.insert(all_m.end(), m_nom.begin(), m_nom.end());
    all_m.insert(all_m.end(), m_flt.begin(), m_flt.end());
    all_m.push_back(0.0);
    const Scale sy = makeScale1(all_m, kMT + kPH, kMT);

    const double slot   = kPW / nd;
    const double bar_w  = slot * 0.38;
    const double offset = slot * 0.22;
    const double py0    = sy.map(0.0);

    constexpr const char* kColOrange     = "#E87A20";
    constexpr const char* kColDarkOrange = "#A04000";

    Svg svg(kW, kH);
    svg.rect(kML, kMT, kPW, kPH, "#F8F8F8", "#BBBBBB");
    svg.text(kML + kPW * 0.5, kMT - 14, title_str, 13, "middle", "#111111");
    svg.text(kML + kPW * 0.5, kH - 8, "Direction", 12, "middle", "#333333");
    svg.textRotated(14, kMT + kPH * 0.5, "Margin [N or Nm]", 12);

    // Y gridlines and tick labels
    constexpr int kNT = 5;
    for (int i = 0; i <= kNT; ++i) {
        const double t  = sy.lo + (sy.hi - sy.lo) * i / kNT;
        const double py = sy.map(t);
        svg.line(kML - 5, py, kML, py, "#666666");
        svg.line(kML, py, kML + kPW, py, "#DDDDDD", 0.5);
        svg.text(kML - 7, py + 4, fmtd(t, 0), 10, "end", "#555555");
    }

    // Zero feasibility line
    if (sy.lo < 0.0 && sy.hi > 0.0) {
        svg.line(kML, py0, kML + kPW, py0, kColRed, 1.2, "6,3");
    }

    // Grouped bars per direction
    for (int i = 0; i < nd; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        const double cx = kML + slot * i + slot * 0.5;

        // Nominal bar (blue, left of center)
        {
            const double bx  = cx - offset - bar_w;
            const double py  = sy.map(m_nom[idx]);
            const double top = (std::min)(py, py0);
            const double bot = (std::max)(py, py0);
            if (bot > top) svg.rect(bx, top, bar_w, bot - top, kColBlue, kColNavy, 0.8);
        }

        // Worst-fault bar (orange, right of center)
        {
            const double bx  = cx + offset;
            const double py  = sy.map(m_flt[idx]);
            const double top = (std::min)(py, py0);
            const double bot = (std::max)(py, py0);
            if (bot > top) svg.rect(bx, top, bar_w, bot - top, kColOrange, kColDarkOrange, 0.8);
        }

        // Direction label, angled -35° from bottom axis
        svg.textAngled(cx, kMT + kPH + 14, labels[idx]);
    }

    // Legend
    const double lx = kML + kPW + 12;
    svg.legendSwatch(lx, kMT + 22, kColBlue,   "Nominal");
    svg.legendSwatch(lx, kMT + 42, kColOrange, "M" + std::to_string(fault_motor + 1) + " fault");

    svg.close();
    writeSvg(path, svg.str());
}

void AcsPlotter::exportVerticesCsv(
    const std::vector<Eigen::Vector4d>& nom,
    const std::vector<Eigen::Vector4d>& fault,
    const int worst_motor,
    const std::string& path) const {

    std::ofstream f(path);
    f << std::fixed << std::setprecision(6);
    f << "set,motor,Fz,L,M,N\n";
    for (const auto& v : nom)
        f << "nominal,-," << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "\n";
    for (const auto& v : fault)
        f << "fault,M" << (worst_motor + 1) << ","
          << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "\n";
}

}  // namespace hexaarch::analysis
