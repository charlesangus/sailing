#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <numbers>
#include <vector>

#include "sail/appendages.hpp"
#include "sail/controls.hpp"
#include "sail/damping.hpp"
#include "sail/equilibrium.hpp"
#include "sail/foil.hpp"
#include "sail/gravity.hpp"
#include "sail/hull_forces.hpp"
#include "sail/hull_resistance.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/mass_model.hpp"
#include "sail/simulation.hpp"

namespace {

using sail::Appendages;
using sail::Controls;
using sail::FoilForces;
using sail::FoilSection;
using sail::MassModel;
using sail::RudderActuator;
using sail::RudderForceModule;
using sail::Simulation;
using sail::State;
using sail::Vec3;
using sail::Wrench;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kRho = 1025.0;

std::shared_ptr<const FoilSection> naca0012() {
    static const auto section =
        std::make_shared<const FoilSection>(FoilSection::load(kDataDir / "foils" / "naca0012.csv"));
    return section;
}

struct Foils {
    MassModel model = MassModel::load(kBoatDir);
    Appendages app{naca0012(), naca0012(), kBoatDir, model.cg()};

    Wrench apply(const State& state, double t = 0.0) {
        Wrench w;
        app.keel().accumulate(state, t, w);
        app.rudder().accumulate(state, t, w);
        return w;
    }

    void hold_rudder(double angle) { app.rudder().actuator().reset(angle); }
};

State moving(const Vec3& v, const Vec3& omega = Vec3::Zero()) {
    State s;
    s.linear_velocity = v;
    s.angular_velocity = omega;
    return s;
}

// Holds the surge speed with a stiff body-x force at the CG, standing in for the sails.
class SpeedHold final : public sail::ForceModule {
public:
    SpeedHold(double mass, double speed) : mass_(mass), speed_(speed) {}
    void accumulate(const State& state, double, Wrench& wrench) override {
        wrench.force.x() += mass_ * (speed_ - state.linear_velocity.x()) / 0.2;
    }

private:
    double mass_;
    double speed_;
};

double heel_of(const State& state) {
    const Vec3 up = state.orientation * Vec3(0.0, 0.0, 1.0);
    return std::atan2(-up.y(), up.z());
}

// Roll decay from 5 degrees at forward speed `speed`, as a damping ratio from the log decrement
// of the first two heel extremes.
double roll_decay_zeta(double speed, bool with_foils) {
    const MassModel model = MassModel::load(kBoatDir);
    const sail::BodyProperties body = model.body_properties();
    const sail::BoatHydrostatics hydro = sail::load_boat_hydrostatics(kBoatDir);
    const sail::HydrostaticParams params = hydro.params(model, sail::kGravity);
    sail::EquilibriumRequest request;
    request.heel = 5.0 * kDeg;
    const sail::Equilibrium start = sail::solve_equilibrium(hydro.mesh, params, body.mass, request);
    REQUIRE(start.converged);
    State initial = start.pose;
    initial.linear_velocity = Vec3(speed, 0.0, 0.0);

    sail::GravityModule gravity(body.mass);
    sail::HydrostaticsModule hydrostatics(hydro.mesh, params);
    sail::DampingModule damping(sail::load_damping(model));
    sail::HullForceModule hull(sail::load_hull_resistance(kBoatDir),
                               sail::load_hull_force_params(kBoatDir, model.cg()));
    Appendages app(naca0012(), naca0012(), kBoatDir, model.cg());
    SpeedHold hold(body.mass, speed);

    Simulation sim(body, initial);
    sim.add_module(gravity);
    sim.add_module(hydrostatics);
    sim.add_module(damping);
    sim.add_module(hull);
    if (with_foils) app.add_to(sim);
    if (speed > 0.0) sim.add_module(hold);

    std::vector<double> peaks;
    double prev2 = heel_of(sim.state());
    sim.step();
    double prev = heel_of(sim.state());
    peaks.push_back(prev2);
    for (int i = 0; i < 10 * 200 && peaks.size() < 3; ++i) {
        sim.step();
        const double h = heel_of(sim.state());
        if ((prev - prev2) * (h - prev) < 0.0) peaks.push_back(prev);
        prev2 = prev;
        prev = h;
    }
    REQUIRE(peaks.size() >= 2);
    REQUIRE(peaks[0] * peaks[1] < 0.0);
    const double delta = std::log(std::abs(peaks[0] / peaks[1]));
    return delta / std::sqrt(std::numbers::pi * std::numbers::pi + delta * delta);
}

}  // namespace

TEST_CASE("J/80 rudder parameters and its pose about the stock") {
    Foils f;
    const sail::RudderParams& p = f.app.rudder().params();
    CHECK(p.max_angle == doctest::Approx(35.0 * kDeg));
    CHECK(p.rate == doctest::Approx(60.0 * kDeg));
    CHECK(p.downwash_factor == doctest::Approx(0.5));
    CHECK(p.stock_axis.isApprox(Vec3(0.0, 0.0, 1.0)));
    CHECK(p.stock_position.x() == doctest::Approx(-4.2949));

    const Vec3 neutral = p.foil.foil.geometry.quarter_chord_centre;
    const Vec3 offset = neutral - p.stock_position;
    const double radius = std::hypot(offset.x(), offset.y());
    CHECK(radius == doctest::Approx(0.0799).epsilon(1e-3));

    f.hold_rudder(20.0 * kDeg);
    f.apply(moving(Vec3(3.0, 0.0, 0.0)));
    CHECK(f.app.rudder_angle() == doctest::Approx(20.0 * kDeg));
    const sail::FoilGeometry& g = f.app.rudder().foil().params().geometry;
    const Vec3 moved = g.quarter_chord_centre - p.stock_position;
    CHECK(moved.x() == doctest::Approx(-radius * std::cos(20.0 * kDeg)));
    CHECK(moved.y() == doctest::Approx(-radius * std::sin(20.0 * kDeg)));
    CHECK(g.quarter_chord_centre.z() == doctest::Approx(neutral.z()));
    CHECK(g.chord_direction.y() < 0.0);
    CHECK(g.span_direction.isApprox(Vec3(0.0, 0.0, -1.0)));

    CHECK_THROWS_AS(sail::load_rudder_params(kBoatDir / "no_such_dir"), sail::BoatDataError);
}

TEST_CASE("J/80 keel at 5 kn and 3 degrees leeway matches the Helmbold hand computation") {
    Foils f;
    const double v = 5.0 * kKnot;
    const double leeway = 3.0 * kDeg;
    // Drifting to port.
    f.apply(moving(Vec3(v * std::cos(leeway), v * std::sin(leeway), 0.0)));
    const FoilForces& r = f.app.keel().last_result();

    const double span = 1.263;
    const double area = 1.1071;
    const double ar = (2.8817 / 1.4408) * span * span / area;
    const double tan_half = std::tan(16.04 * kDeg) + (1.23 - 0.96) / (4.0 * span);
    const double slope = 2.0 * std::numbers::pi * ar /
                         (2.0 + std::sqrt(ar * ar * (1.0 + tan_half * tan_half) + 4.0));
    const double lift = 0.5 * kRho * v * v * area * slope * leeway;

    CHECK(r.alpha == doctest::Approx(leeway));
    CHECK(r.force.y() < 0.0);
    CHECK(r.lift.norm() == doctest::Approx(lift).epsilon(0.02));
    CHECK(-r.force.y() == doctest::Approx(lift).epsilon(0.02));
    MESSAGE("keel at 5 kn, 3 deg: Helmbold a " << slope << " /rad (AR_eff " << ar << "), hand lift "
                                               << lift << " N, module lift " << r.lift.norm()
                                               << " N, side force " << r.force.y() << " N");
}

TEST_CASE("rudder angle follows a step tiller command at the rate limit") {
    Foils f;
    RudderForceModule& rudder = f.app.rudder();
    const double dt = Simulation::kFixedDt;
    const double rate = rudder.params().rate;
    const State s = moving(Vec3(3.0, 0.0, 0.0));

    const auto run_to = [&](double tiller, double from_t) {
        f.app.set_controls(Controls{tiller});
        const double target = tiller * rudder.params().max_angle;
        const double start = rudder.angle();
        double t = from_t;
        for (int i = 0; i < 1000; ++i) {
            Wrench w;
            rudder.accumulate(s, t, w);
            CHECK(std::abs(rudder.angle() - start) <= rate * (t - from_t) + 1e-12);
            if (rudder.angle() == target) return t;
            t += dt;
        }
        return -1.0;
    };

    Wrench w;
    rudder.accumulate(s, 0.0, w);
    REQUIRE(rudder.angle() == 0.0);

    const double reached = run_to(1.0, 0.0);
    CHECK(rudder.angle() == doctest::Approx(35.0 * kDeg));
    CHECK(std::abs(reached - 35.0 / 60.0) <= dt);

    const double back = run_to(-1.0, reached);
    CHECK(std::abs((back - reached) - 70.0 / 60.0) <= dt);

    SUBCASE("out-of-range and non-finite tiller") {
        f.app.set_controls(Controls{4.0});
        CHECK(rudder.actuator().command() == doctest::Approx(35.0 * kDeg));
        f.app.set_controls(Controls{std::nan("")});
        CHECK(rudder.actuator().command() == 0.0);
    }

    SUBCASE("through a Simulation") {
        Foils g;
        const sail::BodyProperties body = g.model.body_properties();
        Simulation sim(body, moving(Vec3(3.0, 0.0, 0.0)));
        g.app.add_to(sim);
        g.app.set_controls(Controls{0.5});
        for (int i = 0; i < 10; ++i) sim.step();
        // The tenth step's accumulate runs at t = 9 dt, nine steps after the first.
        CHECK(g.app.rudder_angle() == doctest::Approx(9.0 * dt * rate));
    }
}

TEST_CASE("positive tiller turns the boat to starboard: negative yaw moment at speed") {
    for (const double tiller : {0.3, -0.3}) {
        Foils f;
        f.hold_rudder(tiller * f.app.rudder().params().max_angle);
        f.apply(moving(Vec3(3.0, 0.0, 0.0)));
        const Wrench& w = f.app.rudder().last_wrench();
        if (tiller > 0.0) {
            CHECK(w.force.y() > 0.0);
            CHECK(w.moment.z() < 0.0);
            CHECK(f.app.rudder().last_result().alpha < 0.0);
        } else {
            CHECK(w.force.y() < 0.0);
            CHECK(w.moment.z() > 0.0);
        }
        MESSAGE("tiller " << tiller << ": rudder " << f.app.rudder_angle() / kDeg
                          << " deg, side force " << w.force.y() << " N, yaw moment "
                          << w.moment.z() << " N*m");
    }

    SUBCASE("the boat yaws to starboard") {
        Foils f;
        const sail::BodyProperties body = f.model.body_properties();
        sail::HullForceModule hull(sail::load_hull_resistance(kBoatDir),
                                   sail::load_hull_force_params(kBoatDir, f.model.cg()));
        SpeedHold hold(body.mass, 3.0);
        Simulation sim(body, moving(Vec3(3.0, 0.0, 0.0)));
        sim.add_module(hull);
        sim.add_module(hold);
        f.app.add_to(sim);
        f.app.set_controls(Controls{0.3});
        for (int i = 0; i < 400; ++i) sim.step();
        CHECK(sim.state().angular_velocity.z() < 0.0);
    }
}

TEST_CASE("keel downwash lowers the rudder's angle of attack by k * alpha_keel") {
    for (const double rudder_angle : {0.0, 5.0 * kDeg}) {
        Foils f;
        f.hold_rudder(rudder_angle);
        const State s = moving(Vec3(3.0, 0.15, 0.0), Vec3(0.0, 0.0, 0.0));

        f.app.rudder().set_downwash_factor(0.0);
        f.apply(s);
        const double alpha_free = f.app.rudder().last_result().alpha;
        CHECK(f.app.rudder().downwash() == 0.0);

        f.app.rudder().set_downwash_factor(0.5);
        f.apply(s);
        const double keel_alpha = f.app.keel().last_result().alpha;
        CHECK(f.app.rudder().keel_alpha() == doctest::Approx(keel_alpha));
        CHECK(f.app.rudder().downwash() == doctest::Approx(0.5 * keel_alpha));
        CHECK(alpha_free - f.app.rudder().last_result().alpha ==
              doctest::Approx(0.5 * keel_alpha));
        if (rudder_angle == 0.0) {
            CHECK(std::abs(f.app.rudder().last_result().alpha) < std::abs(alpha_free));
        }
    }

    CHECK_THROWS_AS(Foils().app.rudder().set_downwash_factor(-0.1), std::invalid_argument);

    SUBCASE("no downwash in sternway") {
        Foils f;
        f.apply(moving(Vec3(-2.0, 0.1, 0.0)));
        CHECK(f.app.rudder().downwash() == 0.0);
    }
}

TEST_CASE("J/80 roll damping: foil share against the lumped coefficient") {
    Foils f;
    const sail::BodyProperties body = f.model.body_properties();
    const sail::DampingStiffness k = sail::dossier_stiffness(f.model.data());
    const double b_crit = sail::critical_damping(k.roll, body.generalised_mass()(3, 3));
    const sail::DampingCoefficients lumped = sail::load_damping(f.model);
    const double zeta_lumped = lumped.roll.linear / b_crit;

    constexpr double kRate = 0.02;
    const auto roll_moment = [&](double speed, double p, bool keel) {
        const State s = moving(Vec3(speed, 0.0, 0.0), Vec3(p, 0.0, 0.0));
        Wrench w;
        if (keel) {
            f.app.keel().accumulate(s, 0.0, w);
        } else {
            f.app.rudder().accumulate(s, 0.0, w);
        }
        return w.moment.x();
    };
    const auto b_lin = [&](double speed, bool keel) {
        return -(roll_moment(speed, kRate, keel) - roll_moment(speed, -kRate, keel)) /
               (2.0 * kRate);
    };

    const double keel3 = b_lin(3.0, true);
    const double rudder3 = b_lin(3.0, false);
    CHECK(keel3 > 0.0);
    CHECK(rudder3 > 0.0);
    CHECK(std::abs(b_lin(0.0, true)) < 1e-3 * keel3);

    const double ref_rate = sail::load_damping_spec(kBoatDir).roll.reference_velocity;
    const double foil_quad_rest =
        -(roll_moment(0.0, ref_rate, true) + roll_moment(0.0, ref_rate, false));
    const double lumped_quad_rest = lumped.roll.quadratic * ref_rate * ref_rate;
    CHECK(foil_quad_rest > 0.0);

    MESSAGE("roll b_crit " << b_crit << " N*m*s/rad; lumped linear " << lumped.roll.linear
                           << " (zeta " << zeta_lumped << "); at 3 m/s keel " << keel3
                           << " (zeta " << keel3 / b_crit << "), rudder " << rudder3 << " (zeta "
                           << rudder3 / b_crit << "); at rest and " << ref_rate
                           << " rad/s, foil cross-flow moment " << foil_quad_rest
                           << " N*m vs lumped quadratic " << lumped_quad_rest << " N*m");
}

TEST_CASE("J/80 roll decay stays plausible at rest and at 3 m/s") {
    const double rest = roll_decay_zeta(0.0, true);
    const double rest_lumped = roll_decay_zeta(0.0, false);
    const double fast = roll_decay_zeta(3.0, true);
    MESSAGE("roll decay zeta: at rest " << rest << " (lumped and hull only " << rest_lumped
                                        << "), at 3 m/s " << fast);
    CHECK(rest >= 0.05);
    CHECK(rest <= 0.15);
    CHECK(rest - rest_lumped >= 0.0);
    CHECK(rest - rest_lumped < 0.01);
    CHECK(fast > rest + 0.02);
    CHECK(fast <= 0.16);
}
