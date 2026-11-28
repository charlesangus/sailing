#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <numbers>

#include "sail/boat_sim.hpp"
#include "sail/foil.hpp"
#include "sail/harness/autopilot.hpp"
#include "sail/harness/autotrim.hpp"

namespace {

using namespace sail;
using namespace sail::harness;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";

constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kDeg = std::numbers::pi / 180.0;

std::shared_ptr<const FoilSection> naca0012() {
    static const auto section = std::make_shared<const FoilSection>(
        FoilSection::load(kDataDir / "foils" / "naca0012.csv"));
    return section;
}

SettleDetector one_channel(std::size_t window) {
    SettleChannel c;
    c.mean_change = 0.01;
    c.spread = 0.02;
    return SettleDetector({c}, window);
}

}  // namespace

TEST_CASE("golden-section search finds the maximum of a known 1-D function") {
    int calls = 0;
    const auto f = [&](double x) {
        ++calls;
        return 2.0 - (x - 0.3) * (x - 0.3);
    };
    const GoldenResult g = golden_section_max(f, -1.0, 2.0, 1e-5, 100);
    CHECK(g.x == doctest::Approx(0.3).epsilon(1e-4));
    CHECK(g.f == doctest::Approx(2.0));
    CHECK(g.evaluations == calls);
    CHECK(g.evaluations < 40);

    const GoldenResult edge = golden_section_max([](double x) { return x; }, 0.0, 1.0, 1e-3, 100);
    CHECK(edge.x > 0.998);

    const GoldenResult capped = golden_section_max(f, -1.0, 2.0, 1e-9, 5);
    CHECK(capped.evaluations == 5);
}

TEST_CASE("parabolic peak search resolves a flat-topped maximum within 0.3 degrees") {
    constexpr double kDeg = std::numbers::pi / 180.0;
    const double peak = 38.3 * kDeg;
    int calls = 0;
    const auto f = [&](double twa) {
        ++calls;
        const double d = (twa - peak) / kDeg;
        return 5.0 - 0.002 * d * d - 0.00002 * d * d * d * d;
    };
    const GoldenResult plain = golden_section_max(f, 30.0 * kDeg, 60.0 * kDeg, 0.035, 7);
    calls = 0;
    const GoldenResult g = parabolic_peak_max(f, 30.0 * kDeg, 60.0 * kDeg, 0.035, 7, 0.0035);
    CHECK(g.evaluations == calls);
    CHECK(g.evaluations <= 7);
    CHECK(std::abs(g.x - peak) < 0.3 * kDeg);
    CHECK(g.f >= plain.f);
}

TEST_CASE("settle detection flags an oscillation as not settled and a convergence as settled") {
    constexpr double dt = 0.05;
    constexpr std::size_t kWindow = 120;

    SettleDetector converging = one_channel(kWindow);
    SettleDetector oscillating = one_channel(kWindow);
    SettleDetector drifting = one_channel(kWindow);
    bool converged_ever = false;
    bool oscillation_ever = false;
    bool drift_ever = false;
    for (int i = 0; i < 4000; ++i) {
        const double t = i * dt;
        const std::array<double, 1> a{3.0 + 0.8 * std::exp(-t / 4.0)};
        const std::array<double, 1> b{3.0 + 0.05 * std::sin(2.0 * std::numbers::pi * t / 4.0)};
        // Slow enough to pass the spread test, so only the change of the mean catches it.
        const std::array<double, 1> c{3.0 + 0.003 * t};
        converging.push(a);
        oscillating.push(b);
        drifting.push(c);
        converged_ever = converged_ever || converging.settled();
        oscillation_ever = oscillation_ever || oscillating.settled();
        drift_ever = drift_ever || drifting.settled();
    }
    CHECK(converged_ever);
    CHECK(converging.settled());
    CHECK(converging.mean(0) == doctest::Approx(3.0).epsilon(1e-6));
    CHECK_FALSE(oscillation_ever);
    CHECK(oscillating.spread(0) > 0.09);
    CHECK_FALSE(drift_ever);
    CHECK(drifting.spread(0) < 0.02);

    SettleDetector early = one_channel(kWindow);
    const std::array<double, 1> flat{1.0};
    for (std::size_t i = 0; i + 1 < 2 * kWindow; ++i) early.push(flat);
    CHECK_FALSE(early.settled());
    early.push(flat);
    CHECK(early.settled());
}

TEST_CASE("settle detection waits out a slow exponential tail") {
    constexpr double dt = 0.05;
    constexpr std::size_t kWindow = 120;
    constexpr double kTau = 30.0;
    SettleChannel loose;
    loose.mean_change = 0.01;
    loose.spread = 0.03;
    SettleChannel tight = loose;
    tight.residual = 0.01;
    SettleDetector without_tail({loose}, kWindow);
    SettleDetector with_tail({tight}, kWindow);
    double loose_offset = -1.0;
    double tight_offset = -1.0;
    double tight_time = -1.0;
    for (int i = 0; i < 6000; ++i) {
        const double t = i * dt;
        const std::array<double, 1> v{3.0 + 0.3 * std::exp(-t / kTau)};
        without_tail.push(v);
        with_tail.push(v);
        if (loose_offset < 0.0 && without_tail.settled()) {
            loose_offset = without_tail.mean(0) - 3.0;
        }
        if (tight_offset < 0.0 && with_tail.settled()) {
            tight_offset = with_tail.mean(0) - 3.0;
            tight_time = t;
        }
    }
    MESSAGE("first settled " << loose_offset << " off without the tail test, " << tight_offset
                             << " off at " << tight_time << " s with it");
    REQUIRE(loose_offset > 0.0);
    REQUIRE(tight_offset > 0.0);
    CHECK(loose_offset > 0.04);
    CHECK(tight_offset < 0.012);
    CHECK(with_tail.tail(0) == doctest::Approx(3.0 - with_tail.mean(0)).epsilon(0.05));

    SettleDetector noisy({tight}, kWindow);
    for (int i = 0; i < 1000; ++i) {
        const std::array<double, 1> v{3.0 + 0.004 * std::sin(1.3 * i)};
        noisy.push(v);
    }
    CHECK(noisy.settled());
}

TEST_CASE("true wind angle is signed by the tack") {
    State s;
    CHECK(true_wind_angle(s, UniformWind(5.0, 180.0), 0.0) == doctest::Approx(90.0 * kDeg));
    CHECK(true_wind_angle(s, UniformWind(5.0, 0.0), 0.0) == doctest::Approx(-90.0 * kDeg));
    CHECK(true_wind_angle(s, UniformWind(5.0, 90.0), 0.0) == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(true_wind_angle(s, UniformWind(5.0, 270.0), 0.0) == doctest::Approx(std::numbers::pi));
}

TEST_CASE("the course true wind angle is the heading's plus the leeway") {
    const UniformWind wind(5.0, 90.0 + 45.0);
    State s;
    const double leeway = 4.0 * kDeg;
    s.linear_velocity = 3.0 * Vec3(std::cos(leeway), std::sin(leeway), 0.0);
    CHECK(true_wind_angle(s, wind, 0.0) == doctest::Approx(45.0 * kDeg));
    CHECK(course_true_wind_angle(s, wind, 0.0) == doctest::Approx(49.0 * kDeg));
    s.linear_velocity = Vec3(0.5 * kMinCourseSpeed, 0.05, 0.0);
    CHECK(course_true_wind_angle(s, wind, 0.0) == doctest::Approx(45.0 * kDeg));
}

TEST_CASE("the autopilot holds a course true wind angle on either tack") {
    for (const double side : {1.0, -1.0}) {
        CAPTURE(side);
        const auto boat = BoatSim::load(kBoatDir, naca0012());
        const double kTwa = side * 90.0 * kDeg;
        const auto wind = std::make_shared<const UniformWind>(10.0 * kKnot, 90.0 + side * 90.0);
        boat->set_wind(wind);
        Controls trim;
        trim.mainsheet = 0.4;
        trim.jib_sheet_port = trim.jib_sheet_starboard = 0.4;
        boat->set_controls(trim);
        State initial = boat->equilibrium().pose;
        initial.linear_velocity = Vec3(3.0, 0.0, 0.0);
        // Started 10 degrees high of the target, so the pilot has to bear away.
        initial.orientation =
            Quat(Eigen::AngleAxisd(-side * 10.0 * kDeg, Vec3::UnitZ())) * initial.orientation;
        boat->appendages().rudder().reset(0.0);
        Simulation& sim = boat->start(initial);

        Autopilot pilot;
        pilot.set_target(kTwa);
        constexpr double dt = Simulation::kFixedDt;
        constexpr double kRun = 45.0;
        constexpr double kTail = 15.0;
        double max_error = 0.0;
        double speed_sum = 0.0;
        int tail = 0;
        const int steps = static_cast<int>(std::lround(kRun / dt));
        for (int i = 0; i < steps; ++i) {
            const State& s = sim.state();
            const double twa = course_true_wind_angle(s, *wind, sim.time());
            trim.tiller = pilot.update(dt, twa, (s.rotation() * s.angular_velocity).z());
            boat->appendages().set_controls(trim);
            boat->step();
            if (sim.time() >= kRun - kTail) {
                const State& n = sim.state();
                const double twa = course_true_wind_angle(n, *wind, sim.time());
                max_error = std::max(max_error, std::abs(twa - kTwa));
                const Vec3 v = n.world_linear_velocity();
                speed_sum += std::hypot(v.x(), v.y());
                ++tail;
            }
        }
        const double speed_kn = speed_sum / tail / kKnot;
        MESSAGE("TWA error over the last " << kTail << " s within " << max_error / kDeg
                                           << " deg at " << speed_kn << " kn, tiller "
                                           << trim.tiller);
        CHECK(max_error < 3.0 * kDeg);
        CHECK(speed_kn > 3.0);
    }
}

TEST_CASE("an unsettled evaluation ranks below every settled one") {
    SteadyResult settled;
    settled.settled = true;
    settled.bsp = 1.0;
    settled.vmg = -0.5;
    SteadyResult unsettled;
    unsettled.window_bsp = 5.0;
    unsettled.window_vmg = -4.0;
    SteadyResult worse = unsettled;
    worse.window_bsp = 4.0;
    worse.window_vmg = -3.0;
    for (const TrimObjective o :
         {TrimObjective::bsp, TrimObjective::vmg_up, TrimObjective::vmg_down}) {
        CAPTURE(static_cast<int>(o));
        CHECK(trim_objective(settled, o) > trim_objective(unsettled, o));
    }
    CHECK(trim_objective(unsettled, TrimObjective::bsp) >
          trim_objective(worse, TrimObjective::bsp));
    CHECK(trim_objective(unsettled, TrimObjective::vmg_down) >
          trim_objective(worse, TrimObjective::vmg_down));
    SteadyResult broken = settled;
    broken.finite = false;
    CHECK(trim_objective(broken, TrimObjective::bsp) == -std::numeric_limits<double>::infinity());
}

TEST_CASE("a polar point solved after another matches the same point solved alone") {
    PolarPointRequest a;
    a.tws = 10.0 * kKnot;
    a.kind = PolarKind::fixed;
    a.twa = 45.0 * kDeg;
    a.trim.max_evaluations = 2;
    a.trim.search_reef = false;
    a.settle.max_time = 20.0;
    PolarPointRequest b = a;
    b.twa = 120.0 * kDeg;

    const auto reused = BoatSim::load(kBoatDir, naca0012());
    const PolarPointResult first = solve_polar_point(*reused, a);
    const PolarPointResult after = solve_polar_point(*reused, b);
    const auto fresh = BoatSim::load(kBoatDir, naca0012());
    const PolarPointResult alone = solve_polar_point(*fresh, b);

    CHECK(first.evaluations == 2);
    CHECK(after.evaluations == alone.evaluations);
    CHECK(after.sim_time == alone.sim_time);
    CHECK(after.best.objective == alone.best.objective);
    CHECK(after.best.trim.mainsheet == alone.best.trim.mainsheet);
    CHECK(after.best.trim.jib_sheet_port == alone.best.trim.jib_sheet_port);
    const SteadyResult& x = after.best.steady;
    const SteadyResult& y = alone.best.steady;
    CHECK(x.window_bsp == y.window_bsp);
    CHECK(x.window_vmg == y.window_vmg);
    CHECK(x.final.crew_y == y.final.crew_y);
    CHECK(x.final.rudder == y.final.rudder);
    CHECK(x.final.state.position == y.final.state.position);
    CHECK(x.final.state.linear_velocity == y.final.state.linear_velocity);
    CHECK(x.final.state.angular_velocity == y.final.state.angular_velocity);
    CHECK(x.final.state.orientation.coeffs() == y.final.state.orientation.coeffs());
    CHECK(reused->mass_model().cg() == fresh->mass_model().cg());
}
