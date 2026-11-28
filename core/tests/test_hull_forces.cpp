#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

#include "sail/damping.hpp"
#include "sail/equilibrium.hpp"
#include "sail/gravity.hpp"
#include "sail/hull_forces.hpp"
#include "sail/hull_resistance.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/mass_model.hpp"
#include "sail/simulation.hpp"

namespace {

using sail::HullForceModule;
using sail::HullForceParams;
using sail::HullForceResult;
using sail::MassModel;
using sail::Quat;
using sail::State;
using sail::Vec3;
using sail::Wrench;

const std::filesystem::path kBoatDir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";

constexpr double kDeg = std::numbers::pi / 180.0;

struct Hull {
    MassModel model = MassModel::load(kBoatDir);
    std::shared_ptr<const sail::HullResistanceModel> resistance =
        sail::load_hull_resistance(kBoatDir);
    HullForceParams params = sail::load_hull_force_params(kBoatDir, model.cg());
    HullForceModule module{resistance, params};

    Wrench apply(const State& state) {
        Wrench w;
        module.accumulate(state, 0.0, w);
        return w;
    }
};

State moving(const Vec3& body_velocity, const Quat& orientation = Quat::Identity()) {
    State s;
    s.orientation = orientation;
    s.linear_velocity = body_velocity;
    return s;
}

bool finite(const Wrench& w) { return w.force.allFinite() && w.moment.allFinite(); }

class UniformCurrent final : public sail::WaterCurrent {
public:
    explicit UniformCurrent(const Vec3& v) : v_(v) {}
    Vec3 velocity(const Vec3&, double) const override { return v_; }

private:
    Vec3 v_;
};

}  // namespace

TEST_CASE("hull force parameters derive from the J/80 dossier") {
    const Hull hull;
    const HullForceParams& p = hull.params;
    CHECK(p.density == doctest::Approx(1025.0));
    CHECK(p.clr_point.x() == doctest::Approx(-0.35739).epsilon(1e-4));
    CHECK(p.clr_point.y() == 0.0);
    CHECK(p.clr_point.z() == doctest::Approx(0.040884 - 0.424 * 0.283534).epsilon(1e-4));
    CHECK(p.reference_point == hull.model.cg());
    const double tc = 0.283534;
    CHECK(p.side_force_coefficient ==
          doctest::Approx(0.5 * 1025.0 * std::numbers::pi * tc * tc).epsilon(1e-4));
    CHECK(p.sway.quadratic > 0.0);
    CHECK(p.sway.linear == doctest::Approx(0.1 * p.sway.quadratic));
    CHECK(p.yaw.quadratic > 0.0);
    CHECK(p.yaw.linear > 0.0);
    MESSAGE("CLR " << p.clr_point.transpose() << ", CG " << p.reference_point.transpose()
                   << ", k " << p.side_force_coefficient << ", sway " << p.sway.linear << " / "
                   << p.sway.quadratic << ", yaw " << p.yaw.linear << " / " << p.yaw.quadratic);

    CHECK_THROWS_AS(sail::load_hull_force_params(kBoatDir / "no_such_dir"), sail::BoatDataError);
}

TEST_CASE("towed upright at zero leeway the drag is the model's total, opposite the velocity") {
    Hull hull;
    for (const double speed : {0.5, 1.5, 3.0, 4.5}) {
        const double total = hull.resistance->resistance(speed, 0.0).total_N;
        REQUIRE(total > 0.0);

        const Wrench w = hull.apply(moving(Vec3(speed, 0.0, 0.0)));
        CHECK(w.force.x() == doctest::Approx(-total).epsilon(1e-12));
        CHECK(std::abs(w.force.y()) < 1e-12 * total);
        CHECK(std::abs(w.force.z()) < 1e-12 * total);
        const HullForceResult& r = hull.module.last_result();
        CHECK(r.leeway == 0.0);
        CHECK(r.side_force_world.norm() == 0.0);
        CHECK(r.induced_drag_world.norm() == 0.0);
        const Vec3 arm = hull.params.clr_point - hull.params.reference_point;
        CHECK((w.moment - arm.cross(w.force)).norm() < 1e-9 * total);
    }

    SUBCASE("on any heading the drag is horizontal and opposite the world velocity") {
        const Quat yawed(Eigen::AngleAxisd(130.0 * kDeg, Vec3::UnitZ()));
        const State s = moving(Vec3(3.0, 0.0, 0.0), yawed);
        const Wrench w = hull.apply(s);
        const Vec3 force_world = s.orientation * w.force;
        const Vec3 v_world = s.world_linear_velocity();
        CHECK(force_world.norm() ==
              doctest::Approx(hull.resistance->resistance(3.0, 0.0).total_N).epsilon(1e-12));
        CHECK(force_world.normalized().dot(v_world.normalized()) == doctest::Approx(-1.0));
    }

    SUBCASE("heel comes from the state") {
        const State s = moving(Vec3(3.0, 0.0, 0.0), sail::heel_trim_orientation(20.0 * kDeg, 0.0));
        const Wrench w = hull.apply(s);
        CHECK(hull.module.last_result().heel == doctest::Approx(20.0 * kDeg));
        const double total = hull.resistance->resistance(3.0, 20.0 * kDeg).total_N;
        CHECK((s.orientation * w.force).x() == doctest::Approx(-total).epsilon(1e-12));
    }
}

TEST_CASE("sternway drag opposes the velocity") {
    Hull hull;
    const Wrench w = hull.apply(moving(Vec3(-2.0, 0.0, 0.0)));
    CHECK(w.force.x() == doctest::Approx(hull.resistance->resistance(2.0, 0.0).total_N));
    CHECK(std::abs(w.force.y()) < 1e-9);

    const Vec3 v(-2.0, 0.3, 0.0);
    const Wrench wl = hull.apply(moving(v));
    const HullForceResult& r = hull.module.last_result();
    CHECK(wl.force.dot(v) < 0.0);
    CHECK((r.resistance_world + r.induced_drag_world).dot(v) < 0.0);
    CHECK(r.side_force_world.y() < 0.0);
    CHECK(std::abs(r.side_force_world.dot(v)) < 1e-9);
}

TEST_CASE("side force opposes leeway, is zero at zero leeway and follows k u_ax u_lat") {
    Hull hull;
    const double k = hull.params.side_force_coefficient;

    hull.apply(moving(Vec3(3.0, 0.0, 0.0)));
    CHECK(hull.module.last_result().side_force_world.norm() == 0.0);

    for (const double lateral : {0.05, 0.26, -0.26}) {
        const Vec3 v(3.0, lateral, 0.0);
        hull.apply(moving(v));
        const HullForceResult& r = hull.module.last_result();
        CHECK(r.side_force_world.y() * lateral < 0.0);
        CHECK(r.side_force_world.norm() == doctest::Approx(k * 3.0 * std::abs(lateral)));
        CHECK(std::abs(r.side_force_world.dot(v)) < 1e-9);
        CHECK(r.induced_drag_world.dot(v) < 0.0);
        const double leeway = std::atan2(lateral, 3.0);
        CHECK(r.induced_drag_world.norm() ==
              doctest::Approx(0.5 * r.side_force_world.norm() * std::abs(std::sin(leeway))));
    }

    hull.apply(moving(Vec3(0.0, 1.0, 0.0)));
    CHECK(hull.module.last_result().side_force_world.norm() < 1e-12);
}

TEST_CASE("hull forces are finite and vanish smoothly at and near zero speed") {
    Hull hull;
    const Wrench zero = hull.apply(State{});
    REQUIRE(finite(zero));
    CHECK(zero.force.norm() == 0.0);
    CHECK(zero.moment.norm() == 0.0);

    const double denormal = std::numeric_limits<double>::denorm_min();
    for (const double s : {denormal, 1e-300, 1e-12, 1e-6, 1e-3, 0.05}) {
        for (const Vec3& dir : {Vec3(1, 0, 0), Vec3(-1, 0, 0), Vec3(0, 1, 0), Vec3(1, -1, 0),
                                Vec3(-1, 1, 0.5), Vec3(0, 0, 1)}) {
            State st = moving(s * dir);
            st.angular_velocity = Vec3(0.0, 0.0, s);
            const Wrench w = hull.apply(st);
            REQUIRE(finite(w));
            CHECK(w.force.norm() <= 200.0 * s + 1e-300);
        }
    }
}

TEST_CASE("water moving with the hull gives no force") {
    MassModel model = MassModel::load(kBoatDir);
    const Vec3 v_world(1.2, -0.4, 0.0);
    HullForceModule module(sail::load_hull_resistance(kBoatDir),
                           sail::load_hull_force_params(kBoatDir, model.cg()),
                           std::make_shared<UniformCurrent>(v_world));
    State s = moving(Vec3::Zero(), Quat(Eigen::AngleAxisd(0.7, Vec3::UnitZ())));
    s.linear_velocity = s.orientation.conjugate() * v_world;
    Wrench w;
    module.accumulate(s, 0.0, w);
    CHECK(w.force.norm() < 1e-9);
    CHECK(w.moment.norm() < 1e-9);
}

TEST_CASE("sway and yaw damping oppose their velocities; moving the CG moves the moment") {
    Hull hull;
    State yawing;
    yawing.angular_velocity = Vec3(0.0, 0.0, 0.3);
    const Wrench w = hull.apply(yawing);
    CHECK(w.moment.z() < 0.0);
    CHECK(hull.module.last_result().damping_moment.z() ==
          doctest::Approx(-(hull.params.yaw.linear + hull.params.yaw.quadratic * 0.3) * 0.3));

    State rolling;
    rolling.angular_velocity = Vec3(0.4, 0.2, 0.0);
    CHECK(hull.apply(rolling).force.norm() == 0.0);

    const Vec3 v(0.0, 0.5, 0.0);
    const Wrench before = hull.apply(moving(v));
    CHECK(before.force.y() ==
          doctest::Approx(-(hull.params.sway.linear + hull.params.sway.quadratic * 0.5) * 0.5));
    hull.module.set_reference_point(hull.params.reference_point + Vec3(0.5, 0.0, 0.0));
    const Wrench after = hull.apply(moving(v));
    CHECK(after.force.y() == doctest::Approx(before.force.y()));
    CHECK(after.moment.z() - before.moment.z() == doctest::Approx(-0.5 * before.force.y()));
}

TEST_CASE("J/80 drifting sideways at 1 m/s decelerates monotonically") {
    const MassModel model = MassModel::load(kBoatDir);
    const sail::BodyProperties body = model.body_properties();
    const sail::BoatHydrostatics hydro = sail::load_boat_hydrostatics(kBoatDir);
    const sail::HydrostaticParams hparams = hydro.params(model);
    const sail::Equilibrium eq = sail::solve_equilibrium(hydro.mesh, hparams, body.mass);
    REQUIRE(eq.converged);

    State start = eq.pose;
    start.linear_velocity = Vec3(0.0, 1.0, 0.0);
    start.angular_velocity.setZero();

    sail::Simulation sim(body, start);
    sim.add_module(std::make_unique<sail::GravityModule>(body.mass));
    sim.add_module(std::make_unique<sail::HydrostaticsModule>(hydro.mesh, hparams));
    sim.add_module(std::make_unique<sail::DampingModule>(sail::load_damping(model)));
    sim.add_module(std::make_unique<HullForceModule>(
        sail::load_hull_resistance(kBoatDir), sail::load_hull_force_params(kBoatDir, model.cg())));

    const auto horizontal_speed = [](const State& s) {
        const Vec3 v = s.world_linear_velocity();
        return std::hypot(v.x(), v.y());
    };

    double previous = horizontal_speed(sim.state());
    bool monotonic = true;
    bool all_finite = true;
    constexpr int kSteps = 10 * 200;
    for (int i = 0; i < kSteps; ++i) {
        sim.step();
        const double speed = horizontal_speed(sim.state());
        all_finite = all_finite && std::isfinite(speed);
        if (speed > previous + 1e-9) monotonic = false;
        previous = speed;
    }
    CHECK(all_finite);
    CHECK(monotonic);
    CHECK(previous < 0.6);
    CHECK(previous > 0.0);
    MESSAGE("horizontal speed after 10 s of beam-on drift: " << previous << " m/s");
}
