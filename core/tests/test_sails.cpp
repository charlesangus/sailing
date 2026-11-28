#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <numbers>
#include <vector>

#include "sail/mass_model.hpp"
#include "sail/sails.hpp"
#include "sail/wind.hpp"

namespace {

using sail::MassModel;
using sail::SailSet;
using sail::SailSetParams;
using sail::SailTrim;
using sail::State;
using sail::Vec3;
using sail::Wrench;

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kKnot = 1852.0 / 3600.0;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";
const std::filesystem::path kSailDir = kDataDir / "sails";

SailSetParams j80_params() {
    const MassModel mass = MassModel::load(kBoatDir, sail::MassConfiguration::sailing);
    return sail::load_sail_set_params(kBoatDir, kSailDir, mass.cg());
}

// True wind angle `twa_deg` off the bow of a boat in sailing(), positive from starboard.
std::shared_ptr<const sail::WindField> wind(double knots, double twa_deg) {
    return std::make_shared<sail::UniformWind>(knots * kKnot, 90.0 + twa_deg);
}

// Upright with the body axes on the world's, so heading east, at `boat_knots`.
State sailing(double boat_knots) {
    State s;
    s.linear_velocity = Vec3(boat_knots * kKnot, 0.0, 0.0);
    return s;
}

double relative(double value, double reference) {
    return std::abs(value - reference) / std::abs(reference);
}

}  // namespace

TEST_CASE("sail spline reproduces the ORC table nodes") {
    const auto main = sail::SailCoefficients::load(kSailDir / "orc_main.csv");
    const auto jib = sail::SailCoefficients::load(kSailDir / "orc_jib.csv");
    CHECK(main.envelope(60.0 * kDeg, 1.0).cl == doctest::Approx(1.38319).epsilon(1e-9));
    CHECK(main.envelope(60.0 * kDeg, 1.0).cd == doctest::Approx(0.11302).epsilon(1e-9));
    CHECK(main.envelope(28.0 * kDeg, 0.0).cl == doctest::Approx(1.34698).epsilon(1e-9));
    CHECK(jib.envelope(150.0 * kDeg, 1.0).cl == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(jib.envelope(150.0 * kDeg, 1.0).cd == doctest::Approx(0.95).epsilon(1e-9));
    CHECK(jib.envelope(2.0 * kDeg, 1.0).cd == doctest::Approx(0.05));
    CHECK(main.envelope(-60.0 * kDeg, 1.0).cl == main.envelope(60.0 * kDeg, 1.0).cl);
    CHECK(main.optimal(-60.0 * kDeg, 1.0).cl == doctest::Approx(-1.38319));
}

// Reference: ORC VPP Documentation 2023 (Offshore Racing Congress), the high coefficient sets of
// Table 5.1 (mainsail, kpm 0.01379) and Table 5.4 (genoa, kpj 0.016) read by not-a-knot cubic
// splines, combined by eqs. 5.35 to 5.48 with flat = 1: CL and CD0 area-weighted, KPP by
// eq. 5.41, heff = eff_span_corr (eq. 5.42) * kheff (Figure 5.14, 2023) * (b + HBI) (eq. 5.45),
// CD by eq. 5.47 with fcdmult(1) = 1.06 and fcdj, and the headsail blanketing of 5.6.2. J/80
// dimensions from data/boats/j80. Computed independently of this code.
TEST_CASE("J/80 combined sail coefficients match the ORC method") {
    const SailSet sails(j80_params(), wind(10.0, 0.0));
    struct Row {
        double awa_deg, cl, cd;
    };
    const std::vector<Row> rows = {
        {30, 1.45175, 0.18271},  {45, 1.45271, 0.34449},  {60, 1.32982, 0.44914},
        {90, 0.99016, 0.64575},  {120, 0.62311, 1.00463}, {150, 0.23249, 1.19904},
        {180, -0.10684, 1.18612},
    };
    for (const Row& row : rows) {
        CAPTURE(row.awa_deg);
        const auto plan = sails.coefficients(row.awa_deg * kDeg);
        CHECK(relative(plan.cl, row.cl) < 0.02);
        CHECK(relative(plan.cd, row.cd) < 0.02);
    }
    const auto plan = sails.coefficients(30.0 * kDeg);
    CHECK(plan.fractionality == doctest::Approx(0.91565).epsilon(1e-3));
    CHECK(plan.effective_span == doctest::Approx(1.07016).epsilon(1e-3));
    CHECK(plan.effective_height == doctest::Approx(14.9115).epsilon(5e-3));
}

TEST_CASE("flattening lowers lift monotonically") {
    SailSet sails(j80_params(), wind(10.0, 0.0));
    for (double awa : {30.0, 60.0, 90.0}) {
        double previous = 1e9;
        for (double flat = 1.0; flat >= -1e-9; flat -= 0.1) {
            for (std::size_t i = 0; i < sails.size(); ++i) {
                sails.set_trim(i, SailTrim{std::max(flat, 0.0), 1.0, {}});
            }
            const double cl = sails.coefficients(awa * kDeg).cl;
            CHECK(cl < previous);
            previous = cl;
        }
        CHECK(previous == doctest::Approx(0.0));
    }
}

TEST_CASE("no apparent wind gives no sail force") {
    SailSet sails(j80_params(), wind(0.0, 0.0));
    Wrench w;
    sails.accumulate(State{}, 0.0, w);
    CHECK(w.force.norm() == 0.0);
    CHECK(w.moment.norm() == 0.0);
    for (const auto& st : sails.last_result().sails) {
        CHECK(std::isfinite(st.cl));
        CHECK(std::isfinite(st.cd));
        CHECK(std::isfinite(st.force.norm()));
    }
    CHECK(std::isfinite(sails.last_result().plan.cd));
}

TEST_CASE("starboard tack upwind drives forward and heels to port") {
    SailSet sails(j80_params(), wind(10.0, 45.0));
    Wrench w;
    sails.accumulate(sailing(5.0), 0.0, w);
    CHECK(w.force.x() > 0.0);
    CHECK(w.force.y() > 0.0);
    CHECK(w.moment.x() < 0.0);
    CHECK(sails.last_result().sails[0].wind.tack == sail::Tack::Starboard);

    SailSet port(j80_params(), wind(10.0, -45.0));
    Wrench m;
    port.accumulate(sailing(5.0), 0.0, m);
    CHECK(m.force.x() == doctest::Approx(w.force.x()));
    CHECK(m.force.y() == doctest::Approx(-w.force.y()));
    CHECK(m.moment.x() == doctest::Approx(-w.moment.x()));
}

TEST_CASE("forces in one apparent wind sum to the plan coefficients") {
    SailSet sails(j80_params(), wind(10.0, 90.0));
    Wrench w;
    sails.accumulate(State{}, 0.0, w);
    const auto& r = sails.last_result();
    const double beta = 90.0 * kDeg;
    const double q = 0.5 * 1.225 * std::pow(10.0 * kKnot, 2) * r.plan.area;
    CHECK(w.force.x() == doctest::Approx(q * (r.plan.cl * std::sin(beta) - r.plan.cd * std::cos(beta))));
    CHECK(w.force.y() == doctest::Approx(q * (r.plan.cl * std::cos(beta) + r.plan.cd * std::sin(beta))));
}

// ORC VPP Documentation 2023, 5.6.2: bk = 1 - f_j (AWA - 135) / 45 above 135 degrees, f_j the
// headsail's fraction outside the foretriangle.
TEST_CASE("headsail blanketing follows ORC") {
    SailSetParams params = j80_params();
    const std::size_t jib = 1;
    REQUIRE(params.sails[jib].name == "jib");
    {
        std::vector<double> bk;
        for (double awa : {120.0, 135.0, 180.0}) {
            SailSet s(params, wind(10.0, awa));
            Wrench w;
            s.accumulate(State{}, 0.0, w);
            bk.push_back(s.last_result().sails[jib].blanketing);
        }
        CHECK(bk[0] == 1.0);
        CHECK(bk[1] == doctest::Approx(1.0));
        const double a = params.sails[jib].area;
        const double fore = 0.5 * params.rig.foretriangle_height * params.rig.foretriangle_base;
        CHECK(bk[2] == doctest::Approx(1.0 - (a - fore) / a));
    }
    params.sails[jib].area = 2.0 * params.sails[jib].area;
    SailSet big(params, wind(10.0, 180.0));
    Wrench w;
    big.accumulate(State{}, 0.0, w);
    const double fore = 0.5 * params.rig.foretriangle_height * params.rig.foretriangle_base;
    CHECK(big.last_result().sails[jib].blanketing ==
          doctest::Approx(fore / params.sails[jib].area));
}

TEST_CASE("the jib carries less of the load downwind than upwind") {
    const auto share = [&](double true_from) {
        SailSet s(j80_params(), wind(10.0, true_from));
        Wrench w;
        s.accumulate(State{}, 0.0, w);
        const auto& r = s.last_result();
        return r.sails[1].force.norm() / (r.sails[0].force.norm() + r.sails[1].force.norm());
    };
    const double up = share(30.0);
    const double down = share(180.0);
    CHECK(down < up);
    CHECK(down < 0.35);
}

TEST_CASE("reefing lowers area and centre of effort") {
    SailSet sails(j80_params(), wind(10.0, 60.0));
    Wrench w;
    sails.accumulate(State{}, 0.0, w);
    const auto full = sails.last_result().sails;
    const double full_force = w.force.norm();
    sails.set_trim(0, SailTrim{1.0, 0.8, {}});
    Wrench r;
    sails.accumulate(State{}, 0.0, r);
    const auto& reefed = sails.last_result().sails;
    CHECK(reefed[0].area == doctest::Approx(full[0].area * 0.64));
    CHECK(reefed[0].centre_of_effort.z() < full[0].centre_of_effort.z());
    CHECK(reefed[1].area == doctest::Approx(full[1].area));
    CHECK(r.force.norm() < full_force);
    CHECK_THROWS(sails.set_trim(0, SailTrim{1.1, 1.0, {}}));
    CHECK_THROWS(sails.set_trim(0, SailTrim{1.0, -0.1, {}}));
}

TEST_CASE("angle of attack override") {
    const auto main = sail::SailCoefficients::load(kSailDir / "orc_main.csv");
    for (double awa : {20.0, 45.0, 90.0, 150.0, -60.0}) {
        CAPTURE(awa);
        const double b = awa * kDeg;
        const auto opt = main.optimal(b, 1.0);
        const auto same = main.section(b, opt.aoa, 1.0);
        CHECK(same.cl == doctest::Approx(opt.cl));
        CHECK(same.cd == doctest::Approx(opt.cd));
        CHECK_FALSE(opt.backed);
        const auto eased = main.section(b, 0.3 * opt.aoa, 1.0);
        CHECK(std::abs(eased.cl) < std::abs(opt.cl));
        const auto backed = main.section(b, -opt.aoa, 1.0);
        CHECK(backed.backed);
        CHECK(backed.cl == doctest::Approx(-opt.cl));
    }
    CHECK(main.section(45.0 * kDeg, 1.0 * kDeg, 1.0).luffing);
    CHECK(main.section(45.0 * kDeg, 60.0 * kDeg, 1.0).stalled);
    const auto plate = main.section(90.0 * kDeg, 90.0 * kDeg, 1.0);
    CHECK(std::abs(plate.cl) < main.envelope(90.0 * kDeg, 1.0).cl);

    // Nearly head to wind on starboard, both sails sheeted to port of the wind: backed.
    const double awa = 2.0 * kDeg;
    SailSet sails(j80_params(), wind(10.0, awa / kDeg));
    const auto& p = sails.params();
    for (std::size_t i = 0; i < sails.size(); ++i) {
        const double chord =
            std::atan2(p.sails[i].clew.y() - p.sails[i].tack.y(),
                       -(p.sails[i].clew.x() - p.sails[i].tack.x()));
        sails.set_trim(i, SailTrim{1.0, 1.0, awa - chord - 20.0 * kDeg});
    }
    Wrench w;
    sails.accumulate(sailing(0.0), 0.0, w);
    CHECK(w.force.x() < 0.0);
    for (const auto& st : sails.last_result().sails) CHECK(st.backed);
}

TEST_CASE("J/80 sail forces at TWS 10 kn" * doctest::description("report")) {
    for (const auto& [twa, boat] : {std::pair{45.0, 5.92}, std::pair{90.0, 6.27}}) {
        SailSet sails(j80_params(), wind(10.0, twa));
        Wrench w;
        sails.accumulate(sailing(boat), 0.0, w);
        MESSAGE("TWA " << twa << " boat " << boat << " kn: drive " << w.force.x() << " N, side "
                       << w.force.y() << " N, Mx " << w.moment.x() << " N m, AWA "
                       << sails.last_result().sails[0].wind.awa_sail / kDeg);
        CHECK(w.force.x() > 0.0);
    }
}
