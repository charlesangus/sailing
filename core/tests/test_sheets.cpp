#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <numbers>
#include <utility>
#include <vector>

#include "sail/controls.hpp"
#include "sail/mass_model.hpp"
#include "sail/sails.hpp"
#include "sail/sheets.hpp"
#include "sail/simulation.hpp"
#include "sail/wind.hpp"

namespace {

using sail::Controls;
using sail::MassModel;
using sail::SailSet;
using sail::SheetController;
using sail::State;
using sail::Tack;
using sail::Vec3;
using sail::Wrench;

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kKnot = 1852.0 / 3600.0;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";
const std::filesystem::path kSailDir = kDataDir / "sails";

std::shared_ptr<const sail::WindField> wind(double knots, double twa_deg) {
    return std::make_shared<sail::UniformWind>(knots * kKnot, 90.0 + twa_deg);
}

State sailing(double boat_knots) {
    State s;
    s.linear_velocity = Vec3(boat_knots * kKnot, 0.0, 0.0);
    return s;
}

struct Rig {
    explicit Rig(double twa_deg)
        : sails(sail::load_sail_set_params(
                    kBoatDir, kSailDir,
                    MassModel::load(kBoatDir, sail::MassConfiguration::sailing).cg()),
                wind(10.0, twa_deg)),
          sheets(sail::load_sheet_params(kBoatDir), sails, wind(10.0, twa_deg)),
          main(sails.index("main")),
          jib(sails.index("jib")) {}

    void set_wind(double twa_deg) {
        sails.set_wind(wind(10.0, twa_deg));
        sheets.set_wind(wind(10.0, twa_deg));
    }

    // Twice at the same time, so the controller reads the centre of effort its own trim placed;
    // a repeated time does not step the reef.
    Wrench evaluate(const Controls& c, double t = 0.0) {
        sheets.set_controls(c);
        Wrench w;
        for (int i = 0; i < 2; ++i) {
            w = Wrench{};
            sheets.accumulate(state, t, w);
            sails.accumulate(state, t, w);
        }
        return w;
    }

    const sail::SailState& main_state() const { return sails.last_result().sails[main]; }
    const sail::SailState& jib_state() const { return sails.last_result().sails[jib]; }

    SailSet sails;
    SheetController sheets;
    std::size_t main;
    std::size_t jib;
    State state = sailing(5.0);
};

Controls trimmed() {
    Controls c;
    c.mainsheet = 0.95;
    c.jib_sheet_port = 0.95;
    c.jib_sheet_starboard = 0.95;
    return c;
}

}  // namespace

TEST_CASE("controls clamp to their ranges") {
    Controls c;
    c.mainsheet = 2.0;
    c.traveler = -3.0;
    c.backstay = std::nan("");
    c.jib_car = -1.0;
    c.reef = 0.7;
    const Controls k = sail::clamped(c);
    CHECK(k.mainsheet == 1.0);
    CHECK(k.traveler == -1.0);
    CHECK(k.backstay == 0.0);
    CHECK(k.jib_car == 0.0);
    CHECK(k.reef == 1.0);
    c.reef = 0.3;
    CHECK(sail::clamped(c).reef == 0.0);
}

TEST_CASE("sheet geometry of the J/80") {
    const sail::SheetParams p = sail::load_sheet_params(kBoatDir);
    const sail::SheetGeometry main(p.main.tack, p.main.head, p.main.clew, p.main.sheet_point);
    // The mast is vertical, so the boom's swing is its chord angle.
    CHECK(main.chord_angle(0.3) == doctest::Approx(0.3));
    CHECK(main.clew(0.3).y() > 0.0);
    CHECK(main.nearest_swing(p.traveler_centre) == doctest::Approx(0.0).epsilon(1e-12));

    const sail::SheetGeometry jib(p.jib.tack, p.jib.head, p.jib.clew, p.jib.sheet_point);
    const Vec3 mid = 0.5 * (p.jib_track_forward + p.jib_track_aft);
    const double nearest = jib.chord_angle(jib.nearest_swing(mid));
    CHECK(nearest > 6.0 * kDeg);
    CHECK(nearest < 11.0 * kDeg);
    CHECK(jib.chord_angle(jib.swing_for_chord_angle(40.0 * kDeg)) ==
          doctest::Approx(40.0 * kDeg));

    const double length = jib.sheet_length(0.5, mid);
    CHECK(jib.limit_swing(mid, length) == doctest::Approx(0.5));
    CHECK(jib.limit_swing(mid, 0.0) == doctest::Approx(jib.nearest_swing(mid)));
}

TEST_CASE("sheet range ends: two-blocked and at the stop") {
    Rig rig(150.0);
    Controls c;
    c.mainsheet = 1.0;
    rig.evaluate(c);
    CHECK(rig.sheets.main().angle == doctest::Approx(0.0).epsilon(1e-9));
    c.mainsheet = 0.0;
    rig.evaluate(c);
    CHECK(rig.sheets.main().angle == doctest::Approx(rig.sheets.params().main.max_angle));
    CHECK(rig.sheets.main().tack == Tack::Starboard);
}

TEST_CASE("easing the mainsheet past luffing drops the main's lift") {
    Rig rig(45.0);
    Controls c = trimmed();
    c.mainsheet = 0.0;
    rig.evaluate(c);
    CHECK(rig.sheets.main().luffing);
    CHECK(rig.main_state().luffing);
    CHECK(std::abs(rig.main_state().cl) < 0.1);
    CHECK(rig.sheets.main().angle == doctest::Approx(rig.sheets.main().wind.awa_sail));
}

TEST_CASE("overtrimming the main past stall lowers lift and raises drag") {
    Rig rig(70.0);
    Controls c = trimmed();
    double best_cl = -1.0;
    double best_cd = 0.0;
    double best_sheet = 0.0;
    for (int i = 0; i <= 200; ++i) {
        c.mainsheet = i / 200.0;
        rig.evaluate(c);
        if (rig.main_state().cl > best_cl) {
            best_cl = rig.main_state().cl;
            best_cd = rig.main_state().cd;
            best_sheet = c.mainsheet;
        }
    }
    CHECK(best_sheet < 1.0);
    c.mainsheet = 1.0;
    rig.evaluate(c);
    CHECK(rig.main_state().stalled);
    CHECK(rig.main_state().cl < best_cl - 0.1);
    CHECK(rig.main_state().cd > best_cd + 0.1);
}

TEST_CASE("the traveler to windward raises the main's angle of attack") {
    Rig rig(70.0);
    Controls c = trimmed();
    c.mainsheet = 0.8;
    std::vector<double> aoa;
    for (const double traveler : {1.0, 0.0, -1.0}) {
        c.traveler = traveler;
        rig.evaluate(c);
        CHECK(std::abs(rig.sheets.main().angle) < std::abs(rig.sheets.main().free_angle));
        aoa.push_back(rig.main_state().aoa);
    }
    CHECK(aoa[1] > aoa[0] + 1.0 * kDeg);
    CHECK(aoa[2] > aoa[1] + 1.0 * kDeg);
}

TEST_CASE("the reef takes its transition time") {
    Rig rig(45.0);
    Controls c = trimmed();
    rig.evaluate(c);
    const double full = rig.main_state().area;
    const double reefed = full * rig.sheets.params().main.reefed_area_fraction;
    const double jib_area = rig.jib_state().area;

    c.reef = 1.0;
    rig.sheets.set_controls(c);
    const double dt = sail::Simulation::kFixedDt;
    double previous = full;
    for (int k = 0; k <= 2400; ++k) {
        const double t = k * dt;
        Wrench w;
        rig.sheets.accumulate(rig.state, t, w);
        rig.sails.accumulate(rig.state, t, w);
        const double area = rig.main_state().area;
        CHECK(area <= previous + 1e-12);
        previous = area;
        if (k == 1000 || k == 1990) {
            CHECK(area < full - 1e-6);
            CHECK(area > reefed + 1e-6);
        }
        if (k >= 2000) CHECK(area == doctest::Approx(reefed).epsilon(1e-12));
    }
    CHECK(rig.jib_state().area == doctest::Approx(jib_area));

    rig.sheets.reset(0.0);
    rig.evaluate(Controls{}, 0.0);
    CHECK(rig.main_state().area == doctest::Approx(full));
}

TEST_CASE("backstay lowers lift monotonically through flat") {
    Rig rig(50.0);
    Controls c = trimmed();
    double main_cl = 1e9;
    double jib_cl = 1e9;
    double main_flat = 2.0;
    for (int i = 0; i <= 10; ++i) {
        c.backstay = i / 10.0;
        rig.evaluate(c);
        CHECK(rig.sheets.main().trim.flat < main_flat);
        CHECK(rig.main_state().cl < main_cl);
        CHECK(rig.jib_state().cl < jib_cl);
        main_flat = rig.sheets.main().trim.flat;
        main_cl = rig.main_state().cl;
        jib_cl = rig.jib_state().cl;
    }
    CHECK(main_flat == doctest::Approx(rig.sheets.params().main.flat_at_full_backstay));
}

TEST_CASE("the windward jib sheet has no effect") {
    Rig rig(50.0);
    Controls c = trimmed();
    c.jib_sheet_port = 0.9;
    c.jib_sheet_starboard = 0.0;
    const Wrench eased = rig.evaluate(c);
    const sail::SailState jib = rig.jib_state();
    c.jib_sheet_starboard = 1.0;
    const Wrench hard = rig.evaluate(c);
    CHECK(rig.sheets.jib().tack == Tack::Starboard);
    CHECK(rig.jib_state().aoa == jib.aoa);
    CHECK(rig.jib_state().cl == jib.cl);
    CHECK((hard.force - eased.force).norm() == 0.0);
    CHECK((hard.moment - eased.moment).norm() == 0.0);

    c.jib_sheet_port = 0.5;
    rig.evaluate(c);
    CHECK(rig.jib_state().aoa != jib.aoa);
}

TEST_CASE("the two tacks mirror each other") {
    Rig rig(55.0);
    Controls c = trimmed();
    c.mainsheet = 0.85;
    c.traveler = -0.4;
    c.jib_sheet_port = 0.9;
    c.jib_sheet_starboard = 0.2;
    c.jib_car = 0.3;
    c.backstay = 0.4;
    const Wrench stbd = rig.evaluate(c);
    const sail::SailSetResult s = rig.sails.last_result();
    const sail::SheetedSailState sm = rig.sheets.main();
    const sail::SheetedSailState sj = rig.sheets.jib();

    rig.set_wind(-55.0);
    std::swap(c.jib_sheet_port, c.jib_sheet_starboard);
    const Wrench port = rig.evaluate(c);
    const sail::SailSetResult p = rig.sails.last_result();
    CHECK(rig.sheets.main().tack == Tack::Port);
    CHECK(rig.sheets.jib().tack == Tack::Port);

    CHECK(rig.sheets.main().angle == doctest::Approx(-sm.angle));
    CHECK(rig.sheets.jib().angle == doctest::Approx(-sj.angle));
    CHECK(rig.sheets.main().lead.y() == doctest::Approx(-sm.lead.y()));
    CHECK(rig.sheets.jib().lead.y() == doctest::Approx(-sj.lead.y()));
    for (const std::size_t i : {rig.main, rig.jib}) {
        CHECK(p.sails[i].aoa == doctest::Approx(-s.sails[i].aoa));
        CHECK(p.sails[i].cl == doctest::Approx(-s.sails[i].cl));
        CHECK(p.sails[i].cd == doctest::Approx(s.sails[i].cd));
    }
    CHECK(port.force.x() == doctest::Approx(stbd.force.x()));
    CHECK(port.force.y() == doctest::Approx(-stbd.force.y()));
    CHECK(port.moment.x() == doctest::Approx(-stbd.moment.x()));
    CHECK(port.moment.z() == doctest::Approx(-stbd.moment.z()));
}

TEST_CASE("J/80 sheet mapping at TWA 50, TWS 10 kn" * doctest::description("report")) {
    Rig rig(50.0);
    MESSAGE("main sheet " << rig.sheets.main_sheet_range_min() << " to "
                          << rig.sheets.main_sheet_range_max() << " m, jib sheet "
                          << rig.sheets.jib_sheet_range_min() << " to "
                          << rig.sheets.jib_sheet_range_max() << " m");
    Controls c = trimmed();
    for (const double sheet : {0.0, 0.5, 0.8, 0.9, 0.95, 1.0}) {
        c.mainsheet = sheet;
        c.jib_sheet_port = sheet;
        rig.evaluate(c);
        MESSAGE("sheet " << sheet << ": main angle " << rig.sheets.main().angle / kDeg
                         << " aoa " << rig.main_state().aoa / kDeg << " cl "
                         << rig.main_state().cl << "; jib angle "
                         << rig.sheets.jib().angle / kDeg << " aoa "
                         << rig.jib_state().aoa / kDeg << " cl " << rig.jib_state().cl);
        CHECK(std::isfinite(rig.main_state().cl));
    }
}
