#include "sail/simulation.hpp"

#include <cmath>
#include <initializer_list>
#include <utility>

namespace sail {

namespace {

// The accumulator counts integer nanoseconds so that splitting the same elapsed time into
// different call sizes yields exactly the same number of steps; a floating-point accumulator
// can land a hair under kFixedDt and defer a step to the next call.
constexpr std::int64_t kStepNs = 5'000'000;
static_assert(kStepNs * 200 == 1'000'000'000, "kStepNs must match Simulation::kFixedDt");

}  // namespace

Simulation::Simulation(const BodyProperties& body, const State& initial)
    : body_(body), current_(initial), previous_(initial) {}

void Simulation::add_module(std::unique_ptr<ForceModule> module) {
    if (!module) return;
    modules_.push_back(module.get());
    owned_.push_back(std::move(module));
}

void Simulation::add_module(ForceModule& module) { modules_.push_back(&module); }

int Simulation::advance(double real_dt) { return advance(real_dt, {}); }

int Simulation::advance(double real_dt, const std::function<void()>& before_step) {
    if (!std::isfinite(real_dt) || real_dt <= 0.0) return 0;

    // Clamping before conversion keeps an absurd input from overflowing the integer cast; the
    // excess would be discarded by the substep cap anyway.
    constexpr double kMaxCallSeconds = 2.0 * kMaxSubsteps * kFixedDt;
    const double clamped = real_dt < kMaxCallSeconds ? real_dt : kMaxCallSeconds;
    accumulator_ns_ += std::llround(clamped * 1e9);

    int steps = 0;
    while (accumulator_ns_ >= kStepNs && steps < kMaxSubsteps) {
        if (before_step) before_step();
        step();
        accumulator_ns_ -= kStepNs;
        ++steps;
    }
    if (accumulator_ns_ >= kStepNs) accumulator_ns_ = 0;
    return steps;
}

void Simulation::step() {
    Wrench wrench;
    const double t = time();
    for (ForceModule* module : modules_) module->accumulate(current_, t, wrench);
    previous_ = current_;
    integrate_step(body_, wrench, kFixedDt, current_);
    ++step_count_;
}

void Simulation::set_body(const BodyProperties& body, const Vec3& cg_shift) {
    body_ = body;
    for (State* s : {&current_, &previous_}) {
        s->position += s->orientation * cg_shift;
        s->linear_velocity += s->angular_velocity.cross(cg_shift);
    }
}

double Simulation::alpha() const {
    return static_cast<double>(accumulator_ns_) / static_cast<double>(kStepNs);
}

}  // namespace sail
