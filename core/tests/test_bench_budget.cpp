#include <doctest/doctest.h>

#include <chrono>
#include <filesystem>
#include <vector>

#include "sail/damping.hpp"
#include "sail/equilibrium.hpp"
#include "sail/gravity.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/mass_model.hpp"
#include "sail/simulation.hpp"

namespace {

constexpr double kHeel = 30.0 * 3.14159265358979323846 / 180.0;
constexpr int kSteps = 2000;
constexpr double kBudgetMicros = 1000.0;

#ifdef NDEBUG
constexpr bool kTimingUnreliable = false;
#else
constexpr bool kTimingUnreliable = true;
#endif

}  // namespace

TEST_CASE("bench_budget: hydrostatics under 1 ms per step at 30 degrees heel" *
          doctest::skip(kTimingUnreliable)) {
    const std::filesystem::path dir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";
    const sail::MassModel model = sail::MassModel::load(dir, sail::MassConfiguration::measurement_trim);
    const sail::BoatHydrostatics hydro = sail::load_boat_hydrostatics(dir);
    const sail::HydrostaticParams params = hydro.params(model);
    const sail::BodyProperties body = model.body_properties();

    sail::EquilibriumRequest request;
    request.heel = kHeel;
    request.free_trim = false;
    const sail::Equilibrium start = sail::solve_equilibrium(hydro.mesh, params, body.mass, request);
    REQUIRE(start.converged);

    sail::HydrostaticsModule hydrostatics(hydro.mesh, params);
    sail::GravityModule gravity(body.mass);
    sail::State state = start.pose;
    const sail::Quat held = state.orientation;

    using Clock = std::chrono::steady_clock;
    double hydro_ns = 0.0;
    for (int i = 0; i < kSteps; ++i) {
        sail::Wrench wrench;
        const auto begin = Clock::now();
        hydrostatics.accumulate(state, i * sail::Simulation::kFixedDt, wrench);
        hydro_ns += std::chrono::duration<double, std::nano>(Clock::now() - begin).count();
        gravity.accumulate(state, i * sail::Simulation::kFixedDt, wrench);
        sail::integrate_step(body, wrench, sail::Simulation::kFixedDt, state);
        state.orientation = held;
        state.angular_velocity.setZero();
    }
    const double mean_us = hydro_ns / kSteps / 1000.0;
    MESSAGE("hydrostatics mean " << mean_us << " us per step over " << kSteps << " steps");
    CHECK(mean_us < kBudgetMicros);
}
