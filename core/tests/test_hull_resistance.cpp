#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>

#include <nlohmann/json.hpp>

#include "sail/boat_data.hpp"
#include "sail/gravity.hpp"
#include "sail/hull_resistance.hpp"

namespace {

using sail::HullResistance;
using sail::OrcDragSurfaces;
using sail::OrcHullParameters;
using sail::OrcHullResistance;

const std::filesystem::path kDataDir = std::filesystem::path(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";
const std::filesystem::path kSurfaces = kDataDir / "hull" / "ORCi_Drag_Surfaces.csv";

constexpr double kDeg = std::numbers::pi / 180.0;

nlohmann::json read_json(const std::filesystem::path& file) {
    std::ifstream in(file);
    REQUIRE(in);
    return nlohmann::json::parse(in);
}

void write_json(const std::filesystem::path& file, const nlohmann::json& json) {
    std::ofstream out(file);
    out << json.dump(2);
}

double hughes_viscous(const OrcHullParameters& p, double v, double length, double area) {
    const double re = 0.85 * v * length / p.kinematic_viscosity;
    const double d = std::log10(re) - 2.03;
    return 0.5 * p.density * v * v * area * (0.066 / (d * d)) * 1.05;
}

// A scratch boat dir: the J/80 sailing condition plus `orc`, if given, and a hull_resistance.json
// that points at the shared surfaces by absolute path.
std::filesystem::path scratch_boat(const std::string& name, const nlohmann::json* orc) {
    const auto dir = std::filesystem::temp_directory_path() / ("sail_test_hull_resistance_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    nlohmann::json hull = read_json(kBoatDir / "hull_hydro.json");
    if (orc != nullptr) hull["conditions"]["sailing"]["orc"] = *orc;
    write_json(dir / "hull_hydro.json", hull);
    nlohmann::json res = read_json(kBoatDir / "hull_resistance.json");
    res["orc"]["drag_surfaces"]["path"] = kSurfaces.string();
    write_json(dir / "hull_resistance.json", res);
    return dir;
}

OrcHullResistance j80_model() {
    return OrcHullResistance(sail::load_orc_hull_parameters(kBoatDir),
                             OrcDragSurfaces::load(kSurfaces));
}

}  // namespace

TEST_CASE("ORC hull resistance matches the Python-VPP canoe-body reference") {
    const auto model = sail::load_hull_resistance(kBoatDir);
    REQUIRE(model->kind() == sail::HullResistanceKind::orc);
    const nlohmann::json ref = read_json(kBoatDir / "hull_resistance_reference.json");
    REQUIRE(ref.at("points").size() == 9);
    for (const auto& p : ref.at("points")) {
        const double fn = p.at("fn").get<double>();
        const double heel = p.at("heel_deg").get<double>();
        CAPTURE(fn);
        CAPTURE(heel);
        const HullResistance r = model->resistance(p.at("speed_mps").get<double>(), heel * kDeg);
        CHECK(r.fn == doctest::Approx(fn).epsilon(1e-9));
        CHECK(r.in_range);
        CHECK(r.total_N == doctest::Approx(p.at("total_N").get<double>()).epsilon(0.02));
        CHECK(r.residuary_N == doctest::Approx(p.at("residuary_N").get<double>()).epsilon(1e-6));
        CHECK(r.viscous_N == doctest::Approx(p.at("viscous_N").get<double>()).epsilon(1e-6));
    }
}

TEST_CASE("J/80 sailing condition lies inside the drag surfaces") {
    const nlohmann::json ref = read_json(kBoatDir / "hull_resistance_reference.json");
    const HullResistance r = j80_model().resistance(0.0, 0.0);
    CHECK(r.in_range);
    CHECK(r.lvr == doctest::Approx(ref["inputs"]["lvr"].get<double>()).epsilon(1e-12));
    CHECK(r.btr == doctest::Approx(ref["inputs"]["btr"].get<double>()).epsilon(1e-12));
}

TEST_CASE("hull resistance is exactly zero at zero speed and symmetric in sternway and heel") {
    const OrcHullResistance model = j80_model();
    for (const double heel : {0.0, 20.0 * kDeg, 40.0 * kDeg}) {
        const HullResistance r = model.resistance(0.0, heel);
        CHECK(r.residuary_N == 0.0);
        CHECK(r.viscous_N == 0.0);
        CHECK(r.total_N == 0.0);
        CHECK(r.fn == 0.0);
    }
    const HullResistance ahead = model.resistance(2.5, 35.0 * kDeg);
    const HullResistance astern = model.resistance(-2.5, -35.0 * kDeg);
    CHECK(astern.total_N == ahead.total_N);
    CHECK(astern.residuary_N == ahead.residuary_N);
    CHECK(model.resistance(1e-6, 0.0).total_N >= 0.0);
}

TEST_CASE("immersed-rail factor is 1 at or below 30 degrees") {
    const OrcHullResistance model = j80_model();
    for (const double heel : {0.0, 10.0, 29.9, 30.0, -30.0}) CHECK(model.rail_factor(heel) == 1.0);
    CHECK(model.rail_factor(35.0) == doctest::Approx(1.01));
    CHECK(model.rail_factor(-40.0) == doctest::Approx(1.04));
    const HullResistance upright = model.resistance(3.0, 0.0);
    CHECK(model.resistance(3.0, 30.0 * kDeg).residuary_N == upright.residuary_N);
    CHECK(model.resistance(3.0, 35.0 * kDeg).residuary_N ==
          doctest::Approx(1.01 * upright.residuary_N));
}

TEST_CASE("a hull outside the drag surfaces is clamped to the edge and flagged") {
    const OrcDragSurfaces surfaces = OrcDragSurfaces::load(kSurfaces);
    OrcHullParameters p = sail::load_orc_hull_parameters(kBoatDir);
    // BTR 12 and LVR 2.5 lie beyond the 2.5..9 and 3..9 grid.
    p.tc = p.bwl / 12.0;
    p.canoe_volume = std::pow(p.lwl / 2.5, 3.0);
    const OrcHullResistance outside(p, surfaces);
    const double v = 0.3 * std::sqrt(sail::kGravity * p.lwl);
    const HullResistance r = outside.resistance(v, 0.0);
    CHECK(r.btr_clamped);
    CHECK(r.lvr_clamped);
    CHECK_FALSE(r.fn_extrapolated);
    CHECK_FALSE(r.in_range);

    OrcHullParameters edge = p;
    edge.tc = edge.bwl / 9.0;
    edge.canoe_volume = std::pow(edge.lwl / 3.0, 3.0);
    const HullResistance e = OrcHullResistance(edge, surfaces).resistance(v, 0.0);
    CHECK(r.residuary_N == doctest::Approx(e.residuary_N).epsilon(1e-9));

    const HullResistance fast = j80_model().resistance(0.8 * std::sqrt(sail::kGravity * p.lwl), 0.0);
    CHECK(fast.fn_extrapolated);
    CHECK_FALSE(fast.in_range);
    const double last = surfaces.lookup(0.7, fast.lvr, fast.btr).value;
    const double before = surfaces.lookup(0.675, fast.lvr, fast.btr).value;
    CHECK(surfaces.lookup(0.8, fast.lvr, fast.btr).value ==
          doctest::Approx(last + 4.0 * (last - before)));
}

TEST_CASE("drag surface lookup is zero at Fn 0 and reproduces grid points") {
    const OrcDragSurfaces s = OrcDragSurfaces::load(kSurfaces);
    REQUIRE(s.fn_axis().size() == 25);
    REQUIRE(s.lvr_axis().size() == 41);
    REQUIRE(s.btr_axis().size() == 41);
    CHECK(s.fn_axis().front() == 0.0);
    CHECK(s.fn_axis()[1] == doctest::Approx(0.125));
    CHECK(s.fn_axis().back() == doctest::Approx(0.7));
    CHECK(s.lookup(0.0, 6.0, 6.0).value == 0.0);
    // First value of the file: Fn 0.125, LVR 3, BTR 2.5.
    CHECK(s.lookup(0.125, 3.0, 2.5).value == doctest::Approx(0.0487));
    const double mid = s.lookup(0.0625, 3.0, 2.5).value;
    CHECK(mid == doctest::Approx(0.5 * 0.0487));
}

TEST_CASE("dossier ORC quantities replace the stand-ins when supplied") {
    const OrcDragSurfaces surfaces = OrcDragSurfaces::load(kSurfaces);
    const OrcHullParameters base = sail::load_orc_hull_parameters(kBoatDir);
    const double v = 3.0;

    SUBCASE("LSM1 and effective BTR") {
        const nlohmann::json orc = {{"lsm1", {{"value", 6.8}, {"unit", "m"}}},
                                    {"btr_effective", {{"value", 5.5}}}};
        const auto dir = scratch_boat("lsm1", &orc);
        const auto model = sail::load_hull_resistance(dir);
        const OrcHullParameters p = sail::load_orc_hull_parameters(dir);
        REQUIRE(p.overrides.lsm1.has_value());
        const HullResistance r = model->resistance(v, 0.0);
        CHECK(r.fn == doctest::Approx(v / std::sqrt(sail::kGravity * 6.8)));
        CHECK(r.btr == doctest::Approx(5.5));
        CHECK(r.lvr == doctest::Approx(6.8 / std::cbrt(base.canoe_volume)));
        CHECK(r.viscous_N == doctest::Approx(hughes_viscous(p, v, 6.8, base.wetted_area)));
        const HullResistance plain = OrcHullResistance(base, surfaces).resistance(v, 0.0);
        CHECK(r.viscous_N != doctest::Approx(plain.viscous_N));
    }

    SUBCASE("composite dynamic length from LSM1, LSM4 and LSM6") {
        const nlohmann::json orc = {{"lsm1", {{"value", 6.8}}},
                                    {"lsm4", {{"value", 7.6}}},
                                    {"lsm6", {{"value", 7.2}}}};
        const auto dir = scratch_boat("composite", &orc);
        const OrcHullResistance model(sail::load_orc_hull_parameters(dir), surfaces);
        CHECK(model.dynamic_length(0.0) == doctest::Approx(6.8));
        // At Fn 0.3 the ORC mixture is all LSM6.
        CHECK(model.dynamic_length(0.3 * std::sqrt(sail::kGravity * 7.2)) == doctest::Approx(7.2));
        // Well above Fn 0.5 L is 70% LSM4 and 30% LSM1.
        CHECK(model.dynamic_length(0.6 * std::sqrt(sail::kGravity * 7.36)) ==
              doctest::Approx(0.7 * 7.6 + 0.3 * 6.8));
        const double vq = 0.35 * std::sqrt(sail::kGravity * 7.4);
        const double l = model.dynamic_length(vq);
        const HullResistance r = model.resistance(vq, 0.0);
        CHECK(r.fn == doctest::Approx(vq / std::sqrt(sail::kGravity * l)));
        CHECK(l == doctest::Approx(7.2 + (r.fn - 0.3) / 0.1 * (7.6 - 7.2)));
        CHECK(r.viscous_N == doctest::Approx(hughes_viscous(model.params(), vq, 6.8,
                                                            base.wetted_area)));
    }

    SUBCASE("heeled tables") {
        const nlohmann::json orc = {{"heeled",
                                     {{"heel_deg", {0.0, 20.0, 40.0}},
                                      {"wetted_area", {10.0, 11.0, 12.0}},
                                      {"btr", {6.0, 5.0, 4.0}},
                                      {"lvr", {6.0, 6.2, 6.4}}}}};
        const auto dir = scratch_boat("heeled", &orc);
        const OrcHullResistance model(sail::load_orc_hull_parameters(dir), surfaces);
        const HullResistance r = model.resistance(v, 10.0 * kDeg);
        CHECK(r.btr == doctest::Approx(5.5));
        CHECK(r.lvr == doctest::Approx(6.1));
        CHECK(r.viscous_N == doctest::Approx(hughes_viscous(model.params(), v, base.lwl, 10.5)));
        const HullResistance beyond = model.resistance(v, 60.0 * kDeg);
        CHECK(beyond.btr == doctest::Approx(4.0));
        CHECK(beyond.viscous_N ==
              doctest::Approx(hughes_viscous(model.params(), v, base.lwl, 12.0)));
    }

    SUBCASE("no orc block keeps the stand-ins") {
        const auto dir = scratch_boat("none", nullptr);
        const HullResistance r = sail::load_hull_resistance(dir)->resistance(v, 0.0);
        CHECK(r.btr == doctest::Approx(base.bwl / base.tc));
        CHECK(r.fn == doctest::Approx(v / std::sqrt(sail::kGravity * base.lwl)));
    }

    SUBCASE("LSM4 without LSM6 is rejected") {
        const nlohmann::json orc = {{"lsm1", {{"value", 6.8}}}, {"lsm4", {{"value", 7.6}}}};
        const auto dir = scratch_boat("partial", &orc);
        CHECK_THROWS_AS(sail::load_orc_hull_parameters(dir), sail::BoatDataError);
    }
}

TEST_CASE("hull resistance model selector") {
    CHECK(sail::load_hull_resistance(kBoatDir, sail::HullResistanceKind::orc)->kind() ==
          sail::HullResistanceKind::orc);
    CHECK_THROWS_AS(sail::load_hull_resistance(kBoatDir, sail::HullResistanceKind::dsyhs),
                    sail::BoatDataError);
}
