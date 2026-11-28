#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "sail/gravity.hpp"
#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace {

using sail::BodyProperties;
using sail::ForceModule;
using sail::Simulation;
using sail::State;
using sail::Vec3;
using sail::Wrench;

constexpr double kG = sail::kGravity;


Vec3 world_angular_momentum(const BodyProperties& body, const State& s) {
    return s.rotation() * (body.inertia * s.angular_velocity);
}

void run_seconds(Simulation& sim, double seconds) {
    const auto steps = static_cast<int>(std::llround(seconds / Simulation::kFixedDt));
    for (int i = 0; i < steps; ++i) sim.step();
}

}  // namespace

TEST_CASE("free fall under a gravity module matches g t^2 / 2") {
    BodyProperties body;
    body.mass = 1200.0;
    State initial;
    initial.position = Vec3(3.0, -2.0, 100.0);
    Simulation sim(body, initial);
    sim.add_module(std::make_unique<sail::GravityModule>(body.mass));

    // Semi-implicit Euler overshoots the drop by a factor (1 + 1/n) after n steps, so the run
    // must exceed 1000 steps for the 0.1% bound to hold.
    const double t = 10.0;
    run_seconds(sim, t);

    REQUIRE(sim.time() == doctest::Approx(t));
    const double expected_drop = 0.5 * kG * t * t;
    const double drop = initial.position.z() - sim.state().position.z();
    CHECK(std::abs(drop - expected_drop) / expected_drop < 1e-3);
    CHECK(sim.state().position.x() == doctest::Approx(3.0));
    CHECK(sim.state().position.y() == doctest::Approx(-2.0));
}

TEST_CASE("torque-free spin about a principal axis conserves angular momentum") {
    BodyProperties body;
    body.mass = 50.0;
    body.inertia = Vec3(2.0, 3.0, 5.0).asDiagonal();

    for (int axis = 0; axis < 3; ++axis) {
        CAPTURE(axis);
        State initial;
        initial.orientation = sail::Quat(Eigen::AngleAxisd(0.3, Vec3(1.0, 2.0, 3.0).normalized()));
        initial.angular_velocity = Vec3::Zero();
        initial.angular_velocity[axis] = 4.0;
        Simulation sim(body, initial);

        const Vec3 l0 = world_angular_momentum(body, sim.state());
        double worst = 0.0;
        const int steps = static_cast<int>(std::llround(60.0 / Simulation::kFixedDt));
        for (int i = 0; i < steps; ++i) {
            sim.step();
            const double err = (world_angular_momentum(body, sim.state()) - l0).norm() / l0.norm();
            if (err > worst) worst = err;
        }
        CHECK(worst < 1e-3);
        CHECK(std::abs(sim.state().orientation.norm() - 1.0) < 1e-12);
    }
}

TEST_CASE("torque-free spin about a non-principal axis conserves |L| and kinetic energy") {
    BodyProperties body;
    body.mass = 10.0;
    body.inertia = Vec3(1.0, 2.0, 3.0).asDiagonal();

    State initial;
    initial.orientation = sail::Quat(Eigen::AngleAxisd(0.4, Vec3(-1.0, 0.5, 2.0).normalized()));
    initial.angular_velocity = Vec3(1.0, 1.0, 1.0);
    Simulation sim(body, initial);

    const auto energy = [&] {
        const Vec3& w = sim.state().angular_velocity;
        return 0.5 * w.dot(body.inertia * w);
    };
    const Vec3 l0 = world_angular_momentum(body, sim.state());
    const double e0 = energy();
    double worst_norm = 0.0;
    double worst_vector = 0.0;
    double worst_energy = 0.0;
    const int steps = static_cast<int>(std::llround(60.0 / Simulation::kFixedDt));
    for (int i = 0; i < steps; ++i) {
        sim.step();
        const Vec3 l = world_angular_momentum(body, sim.state());
        worst_norm = std::max(worst_norm, std::abs(l.norm() / l0.norm() - 1.0));
        worst_vector = std::max(worst_vector, (l - l0).norm() / l0.norm());
        worst_energy = std::max(worst_energy, std::abs(energy() / e0 - 1.0));
    }
    MESSAGE("non-principal spin over 60 s: |L| drift " << worst_norm << ", L direction drift "
                                                       << worst_vector << ", energy drift "
                                                       << worst_energy);
    CHECK(worst_norm < 5e-3);
    CHECK(worst_energy < 5e-3);
    CHECK(worst_vector < 5e-3);
}

TEST_CASE("torque-free motion with added mass conserves kinetic energy and momentum norms") {
    BodyProperties body;
    body.mass = 1500.0;
    body.inertia << 2500.0, 0.0, -120.0, 0.0, 6000.0, 0.0, -120.0, 0.0, 7000.0;
    sail::Mat6 added = sail::Mat6::Zero();
    added.diagonal() << 75.0, 900.0, 2600.0, 400.0, 9000.0, 3000.0;
    added(1, 5) = added(5, 1) = 150.0;
    added(2, 4) = added(4, 2) = -300.0;
    body.added_mass = added;
    const sail::Mat6 m = body.generalised_mass();

    State initial;
    initial.linear_velocity = Vec3(3.0, 0.4, -0.2);
    initial.angular_velocity = Vec3(0.6, -0.3, 0.5);
    Simulation sim(body, initial);

    const auto momentum = [&] {
        sail::Vec6 nu;
        nu << sim.state().linear_velocity, sim.state().angular_velocity;
        return sail::Vec6(m * nu);
    };
    const auto energy = [&] {
        sail::Vec6 nu;
        nu << sim.state().linear_velocity, sim.state().angular_velocity;
        return 0.5 * nu.dot(m * nu);
    };
    const double e0 = energy();
    const sail::Vec6 p0 = momentum();
    const double lin0 = p0.head<3>().norm();
    const double cross0 = p0.head<3>().dot(p0.tail<3>());
    double worst_energy = 0.0;
    double worst_linear = 0.0;
    double worst_cross = 0.0;
    const int steps = static_cast<int>(std::llround(60.0 / Simulation::kFixedDt));
    for (int i = 0; i < steps; ++i) {
        sim.step();
        const sail::Vec6 p = momentum();
        worst_energy = std::max(worst_energy, std::abs(energy() / e0 - 1.0));
        worst_linear = std::max(worst_linear, std::abs(p.head<3>().norm() / lin0 - 1.0));
        worst_cross = std::max(worst_cross, std::abs(p.head<3>().dot(p.tail<3>()) - cross0) /
                                                (lin0 * p0.tail<3>().norm()));
    }
    MESSAGE("added-mass motion over 60 s: energy drift " << worst_energy << ", |p_lin| drift "
                                                         << worst_linear << ", p_lin.p_ang drift "
                                                         << worst_cross);
    CHECK(worst_energy < 5e-3);
    CHECK(worst_linear < 5e-3);
    CHECK(worst_cross < 5e-3);
}

TEST_CASE("intermediate-axis spin tumbles (Dzhanibekov effect)") {
    BodyProperties body;
    body.mass = 10.0;
    body.inertia = Vec3(1.0, 2.0, 3.0).asDiagonal();

    State initial;
    initial.angular_velocity = Vec3(1e-3, 5.0, 1e-3);
    Simulation sim(body, initial);

    const Vec3 initial_axis_world = sim.state().rotation().col(1);
    bool body_rate_flipped = false;
    bool axis_flipped = false;
    const int steps = static_cast<int>(std::llround(20.0 / Simulation::kFixedDt));
    for (int i = 0; i < steps; ++i) {
        sim.step();
        if (sim.state().angular_velocity.y() < 0.0) body_rate_flipped = true;
        if (sim.state().rotation().col(1).dot(initial_axis_world) < -0.9) axis_flipped = true;
    }
    CHECK(body_rate_flipped);
    CHECK(axis_flipped);
}

TEST_CASE("accumulator gives the same state regardless of how real time is sliced") {
    BodyProperties body;
    body.mass = 80.0;
    body.inertia = Vec3(4.0, 6.0, 9.0).asDiagonal();
    State initial;
    initial.linear_velocity = Vec3(2.0, 0.5, 3.0);
    initial.angular_velocity = Vec3(0.2, 1.5, -0.4);

    Simulation coarse(body, initial);
    Simulation fine(body, initial);
    coarse.add_module(std::make_unique<sail::GravityModule>(body.mass));
    fine.add_module(std::make_unique<sail::GravityModule>(body.mass));

    int coarse_steps = 0;
    for (int i = 0; i < 100; ++i) coarse_steps += coarse.advance(0.01);
    int fine_steps = 0;
    for (int i = 0; i < 1000; ++i) fine_steps += fine.advance(0.001);

    CHECK(coarse_steps == 200);
    CHECK(fine_steps == 200);
    CHECK(coarse.step_count() == fine.step_count());
    CHECK(coarse.alpha() == fine.alpha());
    CHECK((coarse.state().position - fine.state().position).norm() < 1e-12);
    CHECK(coarse.state().orientation.angularDistance(fine.state().orientation) < 1e-12);
    CHECK((coarse.state().linear_velocity - fine.state().linear_velocity).norm() < 1e-12);
    CHECK((coarse.state().angular_velocity - fine.state().angular_velocity).norm() < 1e-12);
}

TEST_CASE("a large frame hitch runs at most 20 substeps and drops the rest") {
    BodyProperties body;
    Simulation sim(body, State{});
    sim.add_module(std::make_unique<sail::GravityModule>(body.mass));

    CHECK(sim.advance(1.0) == Simulation::kMaxSubsteps);
    CHECK(sim.step_count() == 20);
    CHECK(sim.time() == doctest::Approx(0.1));
    CHECK(sim.alpha() == 0.0);

    CHECK(sim.advance(0.0025) == 0);
    CHECK(sim.alpha() == doctest::Approx(0.5));
    CHECK(sim.advance(0.0025) == 1);
    CHECK(sim.step_count() == 21);
    CHECK(sim.alpha() == doctest::Approx(0.0));
}

TEST_CASE("previous and current states bracket the latest step") {
    BodyProperties body;
    State initial;
    initial.position = Vec3(0.0, 0.0, 10.0);
    Simulation sim(body, initial);
    sim.add_module(std::make_unique<sail::GravityModule>(body.mass));

    sim.advance(Simulation::kFixedDt * 3.5);
    CHECK(sim.step_count() == 3);
    CHECK(sim.alpha() >= 0.0);
    CHECK(sim.alpha() < 1.0);
    CHECK(sim.previous_state().position.z() > sim.state().position.z());
}
