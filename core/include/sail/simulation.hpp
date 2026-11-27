#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "sail/rigid_body.hpp"

namespace sail {

// A source of force and moment on the body, e.g. gravity, hydrostatics, aerodynamics.
class ForceModule {
public:
    virtual ~ForceModule() = default;

    // Adds this module's contribution to `wrench` (body frame, about the CG) for the given state
    // at simulation time `t` seconds. Must add, never overwrite, and must be deterministic.
    virtual void accumulate(const State& state, double t, Wrench& wrench) = 0;
};

// Fixed-step simulation of one rigid body driven by a set of force modules.
//
// Real elapsed time is fed to advance(), which runs as many fixed steps of kFixedDt as fit,
// up to kMaxSubsteps per call. Time beyond that cap is discarded, so a long frame hitch slows
// the simulation down instead of making it spiral trying to catch up.
class Simulation {
public:
    static constexpr double kFixedDt = 1.0 / 200.0;
    static constexpr int kMaxSubsteps = 20;

    Simulation(const BodyProperties& body, const State& initial);

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) = default;
    Simulation& operator=(Simulation&&) = default;

    // Takes ownership of `module`.
    void add_module(std::unique_ptr<ForceModule> module);
    // Registers `module` without taking ownership; it must outlive this Simulation.
    void add_module(ForceModule& module);

    // Consumes `real_dt` seconds of wall time and returns the number of fixed steps run.
    // Non-positive or non-finite input runs nothing.
    int advance(double real_dt);

    // Runs exactly one fixed step, bypassing the accumulator.
    void step();

    // State after the latest fixed step, and the one before it.
    const State& state() const { return current_; }
    const State& previous_state() const { return previous_; }

    // Fraction in [0, 1) of a fixed step left in the accumulator. A renderer shows
    // previous_state() blended toward state() by this amount.
    double alpha() const;

    // Simulation time in seconds, equal to step_count() * kFixedDt.
    double time() const { return static_cast<double>(step_count_) * kFixedDt; }
    std::uint64_t step_count() const { return step_count_; }

    const BodyProperties& body() const { return body_; }

private:
    BodyProperties body_;
    State current_;
    State previous_;
    std::vector<ForceModule*> modules_;
    std::vector<std::unique_ptr<ForceModule>> owned_;
    std::int64_t accumulator_ns_ = 0;
    std::uint64_t step_count_ = 0;
};

}  // namespace sail
