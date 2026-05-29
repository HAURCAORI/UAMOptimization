#include "calibration/FlightData.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace hexaarch::calibration {
namespace {

std::string trim(std::string s) {
    auto not_space = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
    return s;
}

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::string field;
    std::stringstream ss(line);
    while (std::getline(ss, field, ',')) {
        out.push_back(trim(field));
    }
    return out;
}

}  // namespace

std::optional<std::vector<FlightDataPoint>> loadFlightDataCsv(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        std::cerr << "FlightData: cannot open " << path.string() << "\n";
        return std::nullopt;
    }

    std::string line;
    std::vector<std::string> headers;
    while (std::getline(file, line)) {
        const std::string t = trim(line);
        if (t.empty() || t.front() == '#') {
            continue;
        }
        for (auto& h : splitCsv(t)) headers.push_back(toLower(h));
        break;
    }
    if (headers.empty()) {
        std::cerr << "FlightData: missing header row in " << path.string() << "\n";
        return std::nullopt;
    }

    std::unordered_map<std::string, std::size_t> col_index;
    for (std::size_t i = 0; i < headers.size(); ++i) {
        col_index.emplace(headers[i], i);
    }

    auto column = [&](const std::vector<std::string>& row, const std::string& name) -> std::string {
        const auto it = col_index.find(name);
        if (it == col_index.end() || it->second >= row.size()) return std::string{};
        return row[it->second];
    };

    auto parse_double = [](const std::string& s, double fallback) {
        if (s.empty()) return fallback;
        try { return std::stod(s); } catch (...) { return fallback; }
    };

    std::vector<FlightDataPoint> out;
    int row_num = 1;
    while (std::getline(file, line)) {
        ++row_num;
        const std::string t = trim(line);
        if (t.empty() || t.front() == '#') {
            continue;
        }
        const auto row = splitCsv(t);
        FlightDataPoint p;
        p.mass_kg          = parse_double(column(row, "mass_kg"), 0.0);
        p.airspeed_mps     = parse_double(column(row, "airspeed_mps"), 0.0);
        p.climb_rate_mps   = parse_double(column(row, "climb_rate_mps"), 0.0);
        p.thrust_total_n   = parse_double(column(row, "thrust_total_n"), 0.0);
        p.power_total_w    = parse_double(column(row, "power_total_w"), 0.0);
        p.air_density_kg_m3 = parse_double(column(row, "air_density_kg_m3"), 1.225);
        p.label            = column(row, "label");
        if (p.power_total_w <= 0.0 || p.thrust_total_n <= 0.0) {
            std::cerr << "FlightData: row " << row_num
                      << " has non-positive thrust or power; skipped.\n";
            continue;
        }
        out.push_back(std::move(p));
    }

    if (out.empty()) {
        std::cerr << "FlightData: no usable rows in " << path.string() << "\n";
        return std::nullopt;
    }
    return out;
}

}  // namespace hexaarch::calibration
