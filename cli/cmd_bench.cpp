#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "common.hpp"
#include "sail/damping.hpp"
#include "sail/gravity.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/simulation.hpp"

namespace sailcli {

namespace {

using Json = nlohmann::ordered_json;
using Clock = std::chrono::steady_clock;

constexpr const char* kUsage =
    "usage: sailsim bench --boat DIR [--steps N] [--heel DEG]\n"
    "\n"
    "Times the J/80-style hull simulation (gravity, hydrostatics and damping modules, measurement\n"
    "trim) for N fixed steps and prints JSON: mean and p99 microseconds per step, per force\n"
    "module and for the integrator, and the build type.\n"
    "\n"
    "The boat starts at the heel's equilibrium (trim held level). Each step the attitude is reset\n"
    "to that heel and trim with zero angular velocity, so the heel is held for the whole run and\n"
    "the waterline keeps cutting the same triangles; heave stays free.\n"
    "\n"
    "  --boat DIR   boat dossier directory, e.g. data/boats/j80\n"
    "  --steps N    fixed steps to time, default 10000\n"
    "  --heel DEG   held heel, default 30\n";

// Times each accumulate() call into a preallocated per-step array.
class Timed final : public sail::ForceModule {
public:
    Timed(std::string name, std::unique_ptr<sail::ForceModule> inner, std::size_t steps)
        : name_(std::move(name)), inner_(std::move(inner)) {
        micros.reserve(steps);
    }
    void accumulate(const sail::State& state, double t, sail::Wrench& wrench) override {
        const auto begin = Clock::now();
        inner_->accumulate(state, t, wrench);
        micros.push_back(std::chrono::duration<double, std::micro>(Clock::now() - begin).count());
    }
    const std::string& name() const { return name_; }

    std::vector<double> micros;

private:
    std::string name_;
    std::unique_ptr<sail::ForceModule> inner_;
};

struct Stats {
    double mean = 0.0;
    double p99 = 0.0;
};

Stats summarise(std::vector<double> samples) {
    if (samples.empty()) return {};
    double sum = 0.0;
    for (const double s : samples) sum += s;
    const std::size_t rank = static_cast<std::size_t>(0.99 * static_cast<double>(samples.size() - 1));
    std::nth_element(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(rank), samples.end());
    return {sum / static_cast<double>(samples.size()), samples[rank]};
}

Json stats_json(const Stats& s) {
    Json j;
    j["mean_us"] = s.mean;
    j["p99_us"] = s.p99;
    return j;
}

int bench(const Options& options) {
    const double steps_value = options.number("--steps", 10000.0);
    if (steps_value < 1.0 || steps_value > 1e8) throw UsageError("--steps must be in [1, 1e8]");
    const std::size_t steps = static_cast<std::size_t>(steps_value);
    const double heel = rad(options.number("--heel", 30.0));

    const LoadedBoat boat = load_boat(options, sail::MassConfiguration::measurement_trim);
    const sail::BodyProperties body = boat.model.body_properties();

    sail::EquilibriumRequest request;
    request.heel = heel;
    request.free_trim = false;
    const sail::Equilibrium start = sail::solve_equilibrium(boat.hydro.mesh, boat.params, body.mass, request);
    if (!start.converged) throw std::runtime_error("equilibrium at the requested heel did not converge");

    std::vector<std::unique_ptr<Timed>> owned;
    owned.push_back(std::make_unique<Timed>("gravity", std::make_unique<sail::GravityModule>(body.mass), steps));
    owned.push_back(std::make_unique<Timed>(
        "hydrostatics", std::make_unique<sail::HydrostaticsModule>(boat.hydro.mesh, boat.params), steps));
    owned.push_back(std::make_unique<Timed>(
        "damping", std::make_unique<sail::DampingModule>(sail::load_damping(boat.model)), steps));

    sail::State state = start.pose;
    const sail::Quat held = state.orientation;
    std::vector<double> integrator_us;
    std::vector<double> total_us;
    integrator_us.reserve(steps);
    total_us.reserve(steps);

    for (std::size_t i = 0; i < steps; ++i) {
        const auto begin = Clock::now();
        sail::Wrench wrench;
        const double t = static_cast<double>(i) * sail::Simulation::kFixedDt;
        for (auto& module : owned) module->accumulate(state, t, wrench);
        const auto mid = Clock::now();
        sail::integrate_step(body, wrench, sail::Simulation::kFixedDt, state);
        const auto end = Clock::now();
        integrator_us.push_back(std::chrono::duration<double, std::micro>(end - mid).count());
        total_us.push_back(std::chrono::duration<double, std::micro>(end - begin).count());
        state.orientation = held;
        state.angular_velocity.setZero();
    }

    Json modules = Json::object();
    for (const auto& module : owned) modules[module->name()] = stats_json(summarise(module->micros));

    Json out;
    out["build_type"] = SAIL_BUILD_TYPE;
    out["steps"] = steps;
    out["heel_deg"] = deg(heel);
    out["heel_held"] = "attitude reset to the held heel and trim, angular velocity zeroed, every step";
    out["total_mass_kg"] = body.mass;
    out["step"] = stats_json(summarise(total_us));
    out["modules"] = modules;
    out["integrator"] = stats_json(summarise(integrator_us));
    std::puts(out.dump(2).c_str());
    return 0;
}

}  // namespace

int run_bench(const Args& args) { return guarded("bench", kUsage, bench, args, {"--boat", "--steps", "--heel"}); }

}  // namespace sailcli
