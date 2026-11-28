#include <doctest/doctest.h>

#include <array>
#include <cmath>
#include <filesystem>
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

TEST_CASE("true wind angle is signed by the tack") {
    State s;
    CHECK(true_wind_angle(s, UniformWind(5.0, 180.0), 0.0) == doctest::Approx(90.0 * kDeg));
    CHECK(true_wind_angle(s, UniformWind(5.0, 0.0), 0.0) == doctest::Approx(-90.0 * kDeg));
    CHECK(true_wind_angle(s, UniformWind(5.0, 90.0), 0.0) == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(true_wind_angle(s, UniformWind(5.0, 270.0), 0.0) == doctest::Approx(std::numbers::pi));
}

TEST_CASE("the autopilot holds a true wind angle on a short sailing run") {
    const auto boat = BoatSim::load(kBoatDir, naca0012());
    constexpr double kTwa = 90.0 * kDeg;
    const auto wind = std::make_shared<const UniformWind>(10.0 * kKnot, 90.0 + 90.0);
    boat->set_wind(wind);
    Controls trim;
    trim.mainsheet = 0.4;
    trim.jib_sheet_port = trim.jib_sheet_starboard = 0.4;
    boat->set_controls(trim);
    State initial = boat->equilibrium().pose;
    initial.linear_velocity = Vec3(3.0, 0.0, 0.0);
    // Started 10 degrees high of the target, so the pilot has to bear away.
    initial.orientation = Quat(Eigen::AngleAxisd(-10.0 * kDeg, Vec3::UnitZ())) * initial.orientation;
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
        const double twa = true_wind_angle(s, *wind, sim.time());
        trim.tiller = pilot.update(dt, twa, (s.rotation() * s.angular_velocity).z());
        boat->appendages().set_controls(trim);
        boat->step();
        if (sim.time() >= kRun - kTail) {
            const State& n = sim.state();
            max_error = std::max(max_error, std::abs(true_wind_angle(n, *wind, sim.time()) - kTwa));
            const Vec3 v = n.world_linear_velocity();
            speed_sum += std::hypot(v.x(), v.y());
            ++tail;
        }
    }
    const double speed_kn = speed_sum / tail / kKnot;
    MESSAGE("TWA error over the last " << kTail << " s within " << max_error / kDeg
                                       << " deg at " << speed_kn << " kn, tiller " << trim.tiller);
    CHECK(max_error < 3.0 * kDeg);
    CHECK(speed_kn > 3.0);
}
