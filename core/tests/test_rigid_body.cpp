#include <doctest/doctest.h>

#include <cmath>
#include <memory>

#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace {

using sail::BodyProperties;
using sail::ForceModule;
using sail::Simulation;
using sail::State;
using sail::Vec3;
using sail::Wrench;

constexpr double kG = 9.80665;

class Gravity : public ForceModule {
public:
    explicit Gravity(double mass) : mass_(mass) {}
    void accumulate(const State& state, double, Wrench& wrench) override {
        wrench.force += state.orientation.conjugate() * Vec3(0.0, 0.0, -mass_ * kG);
    }

private:
    double mass_;
};

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
    sim.add_module(std::make_unique<Gravity>(body.mass));

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
    coarse.add_module(std::make_unique<Gravity>(body.mass));
    fine.add_module(std::make_unique<Gravity>(body.mass));

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
    sim.add_module(std::make_unique<Gravity>(body.mass));

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
    sim.add_module(std::make_unique<Gravity>(body.mass));

    sim.advance(Simulation::kFixedDt * 3.5);
    CHECK(sim.step_count() == 3);
    CHECK(sim.alpha() >= 0.0);
    CHECK(sim.alpha() < 1.0);
    CHECK(sim.previous_state().position.z() > sim.state().position.z());
}
