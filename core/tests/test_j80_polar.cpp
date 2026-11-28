#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "sail/boat_sim.hpp"
#include "sail/foil.hpp"
#include "sail/harness/autotrim.hpp"

namespace {

const std::filesystem::path kDataDir = std::filesystem::path(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";

constexpr double kKnot = 0.514444;
constexpr double kTolerance = 0.05;
// Fixed-TWA rows at or below this take the ORC certificate, those above the Python-VPP
// (tools/calib/compare_polar.py).
constexpr double kOrcUpToTwa = 75.0;

#ifdef NDEBUG
constexpr bool kSlowUnavailable = false;
#else
constexpr bool kSlowUnavailable = true;
#endif

using Table = std::vector<std::pair<double, double>>;

// The TWA/BSP rows of one wind speed from a target CSV: the fixed rows, plus the beat row when
// `with_beat`. The ORC certificate's fixed rows start at 52 deg, so a 45 deg point is only
// bracketed by its beat row.
Table read_rows(const std::filesystem::path& path, double tws, bool with_beat) {
    std::ifstream in(path);
    REQUIRE_MESSAGE(in.good(), "cannot read " << path);
    std::string line;
    std::getline(in, line);
    Table rows;
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string tws_s, twa_s, bsp_s, kind;
        std::getline(ss, tws_s, ',');
        std::getline(ss, twa_s, ',');
        std::getline(ss, bsp_s, ',');
        std::getline(ss, kind, ',');
        if (std::abs(std::stod(tws_s) - tws) > 1e-6) continue;
        if (kind == "fixed" || (with_beat && kind == "beat")) {
            rows.emplace_back(std::stod(twa_s), std::stod(bsp_s));
        }
    }
    std::sort(rows.begin(), rows.end());
    return rows;
}

double interpolate(const Table& rows, double twa) {
    REQUIRE(rows.size() >= 2);
    const bool inside = twa >= rows.front().first && twa <= rows.back().first;
    REQUIRE_MESSAGE(inside, "twa " << twa << " outside the target table");
    for (std::size_t i = 1; i < rows.size(); ++i) {
        if (twa <= rows[i].first) {
            const auto& [a, fa] = rows[i - 1];
            const auto& [b, fb] = rows[i];
            return fa + (twa - a) / (b - a) * (fb - fa);
        }
    }
    return rows.back().second;
}

double reference_bsp(double tws, double twa) {
    const bool orc = twa <= kOrcUpToTwa + 1e-9;
    return interpolate(read_rows(kBoatDir / (orc ? "orc_polar.csv" : "pyvpp_polar_nospi.csv"), tws,
                                 orc),
                       twa);
}

}  // namespace

TEST_CASE("j80_polar: TWS 10 at 45, 90 and 135 deg within 5% of the reference polar" *
          doctest::skip(kSlowUnavailable)) {
    std::ifstream fixture_in(kBoatDir / "polar_regression.json");
    REQUIRE(fixture_in.good());
    const nlohmann::json fixture = nlohmann::json::parse(fixture_in);
    const double tws_kn = fixture.at("tws_kn").get<double>();

    auto section = std::make_shared<const sail::FoilSection>(
        sail::FoilSection::load(kDataDir / "foils" / "naca0012.csv"));
    const auto boat = sail::BoatSim::load(kBoatDir, std::move(section), {});

    for (const auto& point : fixture.at("points")) {
        const double twa_deg = point.at("twa_deg").get<double>();
        CAPTURE(twa_deg);

        sail::harness::SteadyRequest request;
        request.tws = tws_kn * kKnot;
        request.twa = twa_deg * std::numbers::pi / 180.0;
        request.trim.mainsheet = point.at("mainsheet").get<double>();
        request.trim.traveler = point.at("traveler").get<double>();
        request.trim.backstay = point.at("backstay").get<double>();
        request.trim.jib_sheet_port = point.at("jib_sheet").get<double>();
        request.trim.jib_sheet_starboard = point.at("jib_sheet").get<double>();
        request.trim.jib_car = point.at("jib_car").get<double>();
        request.trim.reef = point.at("reef").get<double>();

        const sail::harness::SteadyResult result = sail::harness::evaluate_steady(*boat, request);
        REQUIRE(result.finite);
        REQUIRE_MESSAGE(result.settled, "not settled after " << result.time << " s");

        const double bsp_kn = result.bsp / kKnot;
        const double reference = reference_bsp(tws_kn, twa_deg);
        const double error = (bsp_kn - reference) / reference;
        MESSAGE("twa " << twa_deg << " deg: bsp " << bsp_kn << " kn, reference " << reference
                       << " kn, error " << 100.0 * error << " %");
        CHECK(std::abs(error) <= kTolerance);
    }
}
