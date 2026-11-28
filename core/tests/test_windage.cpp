#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>

#include <nlohmann/json.hpp>

#include "sail/mass_model.hpp"
#include "sail/sails.hpp"
#include "sail/windage.hpp"

namespace {

using namespace sail;

constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kDeg = std::numbers::pi / 180.0;

const std::filesystem::path kBoatDir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";

using Json = nlohmann::json;

Json read(const char* file) {
    std::ifstream in(kBoatDir / file);
    REQUIRE(in);
    return Json::parse(in);
}

double val(const Json& j, const char* key) { return j.at(key).at("value").get<double>(); }

Vec3 vec(const Json& j, const char* key) {
    const auto& a = j.at(key).at("value");
    return Vec3(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
}

// Wind from `from_deg` (the boat's bow points east, so 90 is head to wind and 180 is from
// starboard), speed in knots.
std::shared_ptr<const WindField> wind(double knots, double from_deg) {
    return std::make_shared<UniformWind>(knots * kKnot, from_deg);
}

struct Boat {
    MassModel model = MassModel::load(kBoatDir, MassConfiguration::sailing);
    WindageModule module;
    Boat(double knots, double from_deg)
        : module(load_windage_params(model), wind(knots, from_deg)) {}

    State upright() const {
        State s;
        s.position.z() = model.cg().z();
        return s;
    }

    Wrench run(const State& s) {
        Wrench w;
        module.accumulate(s, 0.0, w);
        return w;
    }
};

// The head-wind drag area, element by element, from the data files.
double hand_drag_area() {
    const Json rig = read("rig.json");
    const Json part = read("particulars.json");
    const Json w = read("windage.json");

    const double mast_base = vec(rig, "mast_axis_at_sheer").z();
    const double mast = val(w.at("mast"), "cd_front") * val(rig, "mast_section_transverse") *
                        (val(rig, "mast_top_above_water") - mast_base);
    const double depth = val(rig, "boom_section_depth");
    const double boom = val(w.at("boom"), "cd_end") * depth * depth;

    const Json& rg = w.at("rigging");
    const double wire_cd = val(rg, "cd") * (1.0 + val(rg, "spreader_factor"));
    const Vec3 axis = vec(rig, "mast_axis_at_sheer");
    const Vec3 chain = vec(rig, "shroud_chainplate_base");
    const auto across = [](const Vec3& a, const Vec3& b) { return std::hypot(b.y() - a.y(), b.z() - a.z()); };
    double wires = val(rg, "forestay_diameter") *
                   across(vec(rig, "forestay_deck_point"), vec(rig, "hounds_forestay_on_mast"));
    const double up = axis.z();
    const struct {
        double z;
        const char* diameter;
    } shrouds[] = {{vec(rig, "hounds_forestay_on_mast").z(), "upper_shroud_diameter"},
                   {up + val(rig, "spreader_upper_height_above_sheer"), "intermediate_shroud_diameter"},
                   {up + val(rig, "spreader_lower_height_above_sheer"), "lower_shroud_diameter"}};
    for (const auto& s : shrouds) {
        wires += 2.0 * val(rg, s.diameter) * across(Vec3(axis.x(), 0.0, s.z), chain);
    }
    wires += val(rg, "backstay_diameter") *
             across(Vec3(axis.x(), 0.0, val(rig, "mast_top_above_water")),
                    Vec3(-val(part, "safp"), 0.0, val(part, "freeboard_aft_fam")));
    wires *= wire_cd;

    const double ffm = val(part, "freeboard_fwd_ffm");
    const double fam = val(part, "freeboard_aft_fam");
    const double fmast = val(part, "freeboard_at_mast");
    const double mast_x = -axis.x();
    const double safp = val(part, "safp");
    const double loa = val(part, "loa");
    const double fb_mean = (0.5 * (ffm + fmast) * mast_x + 0.5 * (fmast + fam) * (safp - mast_x) +
                            fam * (loa - safp)) /
                           loa;
    const double hull = val(w.at("hull"), "cd") * val(part, "beam") * fb_mean;

    const double crew = val(w.at("crew"), "cd") * val(w.at("crew"), "sitting_front_area_per_person") *
                        read("mass.json").at("crew_count").at("value").get<double>();

    return mast + boom + wires + hull + crew;
}

}  // namespace

TEST_CASE("windage head to wind at 20 knots matches the element-by-element hand total") {
    Boat boat(20.0, 90.0);
    const Wrench w = boat.run(boat.upright());
    const double q = 0.5 * SailSetParams{}.air_density * std::pow(20.0 * kKnot, 2);
    const double hand = q * hand_drag_area();

    CHECK(-w.force.x() == doctest::Approx(hand).epsilon(0.05));
    CHECK(std::abs(w.force.y()) < 1e-9 * hand);
    CHECK(std::abs(w.force.z()) < 1e-9 * hand);
    MESSAGE("hand total " << hand << " N, module total " << -w.force.x() << " N");
}

TEST_CASE("windage is nonzero with no sail set and with the sails furled") {
    Boat boat(10.0, 90.0);
    CHECK(boat.run(boat.upright()).force.norm() > 1.0);

    auto sails = load_sail_set_params(kBoatDir, std::filesystem::path(SAIL_TEST_DATA_DIR) / "sails",
                                      boat.model.cg());
    SailSet set(std::move(sails), wind(10.0, 90.0));
    for (std::size_t i = 0; i < set.size(); ++i) {
        SailTrim trim = set.trim(i);
        trim.reef = 0.0;
        set.set_trim(i, trim);
    }
    Wrench sail_wrench;
    set.accumulate(boat.upright(), 0.0, sail_wrench);
    Wrench both = sail_wrench;
    boat.module.accumulate(boat.upright(), 0.0, both);
    CHECK((both.force - sail_wrench.force).norm() == doctest::Approx(boat.run(boat.upright()).force.norm()));
}

TEST_CASE("accumulate adds to the wrench") {
    Boat boat(10.0, 90.0);
    Wrench w;
    w.force = Vec3(1.0, 2.0, 3.0);
    boat.module.accumulate(boat.upright(), 0.0, w);
    const Wrench alone = boat.run(boat.upright());
    CHECK((w.force - Vec3(1.0, 2.0, 3.0) - alone.force).norm() < 1e-9);
}

TEST_CASE("windage force lies along the apparent wind") {
    Boat boat(15.0, 140.0);
    State s = boat.upright();
    s.orientation = Quat(Eigen::AngleAxisd(0.4, Vec3::UnitZ())) * Quat(Eigen::AngleAxisd(15.0 * kDeg, Vec3::UnitX()));
    s.linear_velocity = Vec3(3.0, 0.2, 0.0);
    const Wrench w = boat.run(s);
    const Vec3 apparent = apparent_wind(s, Vec3::Zero(), *std::make_shared<UniformWind>(15.0 * kKnot, 140.0), 0.0).body;
    REQUIRE(w.force.norm() > 1.0);
    CHECK(w.force.normalized().dot(apparent.normalized()) == doctest::Approx(1.0).epsilon(1e-9));
}

TEST_CASE("windage is zero in zero apparent wind") {
    Boat boat(0.0, 90.0);
    const Wrench w = boat.run(boat.upright());
    CHECK(w.force.norm() == 0.0);
    CHECK(w.moment.norm() == 0.0);
}

TEST_CASE("a beam wind heels the boat away from the wind") {
    Boat from_starboard(15.0, 180.0);
    const Wrench w = from_starboard.run(from_starboard.upright());
    CHECK(w.force.y() > 0.0);
    CHECK(w.moment.x() < 0.0);
    for (const WindageGroup g : {WindageGroup::mast, WindageGroup::rigging, WindageGroup::hull}) {
        CHECK(from_starboard.module.last_result().groups[static_cast<std::size_t>(g)].moment.x() < 0.0);
    }

    Boat from_port(15.0, 0.0);
    const Wrench mirror = from_port.run(from_port.upright());
    CHECK(mirror.force.y() < 0.0);
    CHECK(mirror.moment.x() == doctest::Approx(-w.moment.x()).epsilon(1e-6));
}

TEST_CASE("a hiking crew shows a different area from a sitting one") {
    Boat boat(15.0, 180.0);
    boat.module.set_crew(4, boat.model.crew_position(), CrewPosture::sitting);
    const double sitting = boat.run(boat.upright()).force.y();
    const double crew_sitting = boat.module.last_result().groups[static_cast<std::size_t>(WindageGroup::crew)].force.y();
    boat.module.set_crew(4, boat.model.crew_position(), CrewPosture::hiking);
    boat.run(boat.upright());
    const double crew_hiking = boat.module.last_result().groups[static_cast<std::size_t>(WindageGroup::crew)].force.y();
    CHECK(crew_sitting > 0.0);
    CHECK(crew_hiking == doctest::Approx(crew_sitting * 0.4 / 0.5));
    CHECK(sitting > crew_sitting);
    boat.module.set_crew(0, Vec3::Zero(), CrewPosture::sitting);
    boat.run(boat.upright());
    CHECK(boat.module.last_result().groups[static_cast<std::size_t>(WindageGroup::crew)].force.norm() == 0.0);
}

TEST_CASE("heel exposes more topsides on the rising side and less on the sinking side") {
    Boat boat(10.0, 180.0);
    boat.run(boat.upright());
    const WindageResult level = boat.module.last_result();

    State heeled = boat.upright();
    heeled.orientation = Quat(Eigen::AngleAxisd(20.0 * kDeg, Vec3::UnitX()));
    boat.run(heeled);
    const WindageResult& r = boat.module.last_result();

    CHECK(level.topsides_port == doctest::Approx(level.topsides_starboard));
    CHECK(r.topsides_port > level.topsides_port);
    CHECK(r.topsides_starboard < level.topsides_starboard);
    MESSAGE("upright port/starboard " << level.topsides_port << "/" << level.topsides_starboard
                                      << " m^2, 20 deg starboard down " << r.topsides_port << "/"
                                      << r.topsides_starboard);
}

TEST_CASE("moving the reference point keeps the force and shifts the moment by r x F") {
    Boat boat(15.0, 120.0);
    const State s = boat.upright();
    const Wrench before = boat.run(s);
    const Vec3 shift(0.3, 0.0, -0.2);
    boat.module.set_reference_point(boat.module.params().reference_point + shift);
    State moved = s;
    moved.position += shift;
    const Wrench after = boat.run(moved);
    CHECK((after.force - before.force).norm() < 1e-9);
    CHECK((after.moment - (before.moment - shift.cross(before.force))).norm() < 1e-6);
}
