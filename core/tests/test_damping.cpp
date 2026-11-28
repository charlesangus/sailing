#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <numbers>
#include <vector>

#include "sail/damping.hpp"
#include "sail/equilibrium.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/mass_model.hpp"
#include "sail/mesh.hpp"
#include "sail/gravity.hpp"
#include "sail/simulation.hpp"

namespace {

using sail::DampingCoefficients;
using sail::DampingDof;
using sail::DampingModule;
using sail::MassModel;
using sail::Simulation;
using sail::State;
using sail::TriMesh;
using sail::Vec3;
using sail::Wrench;

const std::filesystem::path kBoatDir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";

constexpr double kG = sail::kGravity;


TEST_CASE("damping force law is odd, linear plus quadratic, on heave roll and pitch only") {
    DampingCoefficients c;
    c.heave = {10.0, 3.0};
    c.roll = {20.0, 5.0};
    c.pitch = {30.0, 7.0};
    DampingModule module(c);

    State s;
    s.linear_velocity = Vec3(4.0, 5.0, 2.0);
    s.angular_velocity = Vec3(1.0, -2.0, 6.0);
    Wrench w;
    module.accumulate(s, 0.0, w);

    CHECK(w.force.x() == 0.0);
    CHECK(w.force.y() == 0.0);
    CHECK(w.force.z() == doctest::Approx(-(10.0 * 2.0 + 3.0 * 2.0 * 2.0)));
    CHECK(w.moment.x() == doctest::Approx(-(20.0 * 1.0 + 5.0 * 1.0 * 1.0)));
    CHECK(w.moment.y() == doctest::Approx(30.0 * 2.0 + 7.0 * 2.0 * 2.0));
    CHECK(w.moment.z() == 0.0);

    State negated = s;
    negated.linear_velocity.z() = -2.0;
    Wrench opposite;
    module.accumulate(negated, 0.0, opposite);
    CHECK(opposite.force.z() == doctest::Approx(-w.force.z()));

    Wrench accumulated = w;
    module.accumulate(s, 0.0, accumulated);
    CHECK(accumulated.force.z() == doctest::Approx(2.0 * w.force.z()));
}

TEST_CASE("damping linear and quadratic parts separate") {
    DampingCoefficients linear_only;
    linear_only.heave = {10.0, 0.0};
    DampingCoefficients quadratic_only;
    quadratic_only.heave = {0.0, 3.0};
    DampingModule lin(linear_only);
    DampingModule quad(quadratic_only);

    State slow;
    slow.linear_velocity.z() = 1.0;
    State fast;
    fast.linear_velocity.z() = 2.0;
    Wrench a, b, c, d;
    lin.accumulate(slow, 0.0, a);
    lin.accumulate(fast, 0.0, b);
    quad.accumulate(slow, 0.0, c);
    quad.accumulate(fast, 0.0, d);

    CHECK(b.force.z() == doctest::Approx(2.0 * a.force.z()));
    CHECK(d.force.z() == doctest::Approx(4.0 * c.force.z()));
}

TEST_CASE("damping multipliers scale one DOF at a time and reject bad values") {
    DampingCoefficients c;
    c.heave = {10.0, 2.0};
    c.roll = {20.0, 4.0};
    DampingModule module(c);
    module.set_multiplier(DampingDof::roll, 5.0);
    CHECK(module.multiplier(DampingDof::heave) == 1.0);
    CHECK(module.multiplier(DampingDof::roll) == 5.0);

    State s;
    s.linear_velocity.z() = 1.0;
    s.angular_velocity.x() = 1.0;
    Wrench w;
    module.accumulate(s, 0.0, w);
    CHECK(w.force.z() == doctest::Approx(-12.0));
    CHECK(w.moment.x() == doctest::Approx(-5.0 * 24.0));

    module.set_multipliers(0.0);
    Wrench off;
    module.accumulate(s, 0.0, off);
    CHECK(off.force.z() == 0.0);
    CHECK(off.moment.x() == 0.0);

    CHECK_THROWS_AS(module.set_multiplier(DampingDof::pitch, -1.0), std::invalid_argument);
    CHECK_THROWS_AS(module.set_multipliers(std::nan("")), std::invalid_argument);
}

TEST_CASE("J/80 damping coefficients derive from the dossier") {
    const MassModel model = MassModel::load(kBoatDir);
    const sail::DampingSpec spec = sail::load_damping_spec(kBoatDir);
    const sail::DampingStiffness k = sail::dossier_stiffness(model.data());

    CHECK(k.heave == doctest::Approx(1025.0 * kG * 8.99 * 1000.0 / 1025.0).epsilon(1e-9));
    CHECK(k.roll == doctest::Approx(264.8 / std::sin(std::numbers::pi / 180.0)).epsilon(1e-9));
    // A 26 ft keelboat is roughly ten to twenty times stiffer in pitch than in roll.
    CHECK(k.pitch > 5.0 * k.roll);
    CHECK(k.pitch < 50.0 * k.roll);

    const sail::BodyProperties body = model.body_properties();
    const DampingCoefficients c = sail::derive_damping(spec, k, body);
    const double heave_crit = 2.0 * std::sqrt(k.heave * (body.mass + body.added_mass(2, 2)));
    CHECK(c.heave.linear == doctest::Approx(spec.heave.zeta_linear * heave_crit));
    CHECK(c.heave.quadratic * spec.heave.reference_velocity ==
          doctest::Approx(spec.heave.zeta_quadratic * heave_crit));
    CHECK(c.roll.linear > 0.0);
    CHECK(c.pitch.linear > 0.0);

    CHECK(spec.roll.zeta_linear >= 0.05);
    CHECK(spec.roll.zeta_linear <= 0.1);
}

TEST_CASE("damping spec errors name the file") {
    const std::filesystem::path missing = kBoatDir / "no_such_dir";
    CHECK_THROWS_AS(sail::load_damping_spec(missing), sail::BoatDataError);
}

TEST_CASE("J/80 dropped into flat water oscillates at the heave period and decays") {
    const MassModel model = MassModel::load(kBoatDir);
    const sail::BodyProperties body = model.body_properties();

    const sail::BoatHydrostatics hydro = sail::load_boat_hydrostatics(kBoatDir);
    const TriMesh& boat = hydro.mesh;
    const sail::HydrostaticParams params = hydro.params(model, kG);

    const DampingCoefficients coefficients = sail::load_damping(model);

    const sail::Equilibrium solved = sail::solve_equilibrium(boat, params, body.mass);
    REQUIRE(solved.converged);
    const State equilibrium = solved.pose;

    const sail::HydrostaticResult at_equilibrium =
        sail::compute_hydrostatics(boat, equilibrium, 0.0, params);
    REQUIRE(at_equilibrium.waterplane_area > 0.0);
    REQUIRE(at_equilibrium.force_world.z() ==
            doctest::Approx(body.mass * kG).epsilon(1e-6));
    REQUIRE(std::abs(at_equilibrium.moment_world.y()) < 1e-3 * body.mass * kG);
    const double heave_stiffness = params.density * kG * at_equilibrium.waterplane_area;
    const double period =
        2.0 * std::numbers::pi * std::sqrt((body.mass + body.added_mass(2, 2)) / heave_stiffness);

    constexpr double kDrop = 0.1;
    State dropped = equilibrium;
    dropped.position.z() += kDrop;
    dropped.linear_velocity.setZero();
    dropped.angular_velocity.setZero();

    Simulation sim(body, dropped);
    sim.add_module(std::make_unique<sail::GravityModule>(body.mass));
    sim.add_module(std::make_unique<sail::HydrostaticsModule>(boat, params));
    sim.add_module(std::make_unique<DampingModule>(coefficients));

    constexpr int kSteps = 30 * 200;
    std::vector<double> z;
    z.reserve(kSteps + 1);
    z.push_back(sim.state().position.z());
    for (int i = 0; i < kSteps; ++i) {
        sim.step();
        z.push_back(sim.state().position.z());
    }

    std::vector<double> x(z.size());
    for (std::size_t i = 0; i < z.size(); ++i) x[i] = z[i] - equilibrium.position.z();
    const double dt = Simulation::kFixedDt;

    {
        std::vector<double> up_crossings;
        const std::size_t limit = static_cast<std::size_t>(5.0 * period / dt);
        for (std::size_t i = 1; i < std::min(limit, x.size()); ++i) {
            if (x[i - 1] < 0.0 && x[i] >= 0.0) {
                const double frac = -x[i - 1] / (x[i] - x[i - 1]);
                up_crossings.push_back((static_cast<double>(i - 1) + frac) * dt);
            }
        }
        REQUIRE(up_crossings.size() >= 3);
        const double measured =
            (up_crossings.back() - up_crossings.front()) /
            static_cast<double>(up_crossings.size() - 1);
        CHECK(std::abs(measured - period) / period < 0.10);
    }

    {
        const std::size_t start = static_cast<std::size_t>(5.0 * period / dt);
        REQUIRE(start < x.size());
        double worst = 0.0;
        for (std::size_t i = start; i < x.size(); ++i) worst = std::max(worst, std::abs(x[i]));
        CHECK(worst < 0.05 * kDrop);
    }

    {
        double previous = kDrop * 2.0;
        int peaks = 0;
        for (std::size_t i = 1; i + 1 < x.size(); ++i) {
            if (x[i] > x[i - 1] && x[i] >= x[i + 1] && x[i] > 0.001) {
                CHECK(x[i] <= previous * 1.01);
                previous = x[i];
                ++peaks;
            }
        }
        CHECK(peaks >= 2);
    }
}

}  // namespace
