#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <numbers>
#include <vector>

#include "sail/boat_sim.hpp"
#include "sail/foil.hpp"
#include "sail/grounding.hpp"

namespace {

using namespace sail;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";

constexpr double kKnot = 0.514444;
constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kBoatLength = 8.0;

#ifdef NDEBUG
constexpr bool kTimingUnreliable = false;
#else
constexpr bool kTimingUnreliable = true;
#endif

std::shared_ptr<const FoilSection> naca0012() {
    static const auto section = std::make_shared<const FoilSection>(
        FoilSection::load(kDataDir / "foils" / "naca0012.csv"));
    return section;
}

// A plane rising along world +x at `slope_deg`, at height `z0` on the line x = x0.
std::shared_ptr<const Bathymetry> slope(double z0, double x0, double slope_deg) {
    constexpr std::size_t kCols = 161;
    constexpr std::size_t kRows = 61;
    constexpr double kPixel = 1.0;
    const double ox = x0 - 100.0;
    const double oy = 30.0;
    std::vector<float> h(kCols * kRows);
    for (std::size_t row = 0; row < kRows; ++row) {
        for (std::size_t col = 0; col < kCols; ++col) {
            const double x = ox + static_cast<double>(col) * kPixel;
            h[row * kCols + col] =
                static_cast<float>(z0 + std::tan(slope_deg * kDeg) * (x - x0));
        }
    }
    return std::make_shared<const Bathymetry>(kCols, kRows, kPixel, ox, oy, std::move(h));
}

std::shared_ptr<const Bathymetry> constant(double z) {
    return std::make_shared<const Bathymetry>(1, 1, 1.0, 0.0, 0.0,
                                              std::vector<float>{static_cast<float>(z)});
}

bool finite(const State& s) {
    return s.position.allFinite() && s.orientation.coeffs().allFinite() &&
           s.linear_velocity.allFinite() && s.angular_velocity.allFinite();
}

double kinetic_energy(const BodyProperties& body, const State& s) {
    return 0.5 * body.mass * s.linear_velocity.squaredNorm() +
           0.5 * s.angular_velocity.dot(body.inertia * s.angular_velocity);
}

double horizontal_speed(const State& s) {
    const Vec3 v = s.world_linear_velocity();
    return std::hypot(v.x(), v.y());
}

// The lowest contact point in the world at `s`.
Vec3 lowest_point(const BoatSim& boat, const State& s) {
    Vec3 best = Vec3::Constant(std::numeric_limits<double>::infinity());
    const Vec3& ref = boat.grounding().params().reference_point;
    for (const Vec3& p : boat.grounding().points()) {
        const Vec3 w = s.position + s.orientation * (p - ref);
        if (w.z() < best.z()) best = w;
    }
    return best;
}

struct Grounding {
    double thrust = 0.0;
    double first_contact_x = std::numeric_limits<double>::quiet_NaN();
    double rest_x = 0.0;
    double furthest_x = -std::numeric_limits<double>::infinity();
    double max_vertical_speed = 0.0;
    double max_kinetic_ratio = 0.0;
    double max_penetration = 0.0;
    bool finite = true;
};

// Drives the J/80 from its 5 kn tow balance, under the thrust that holds it there, onto a 10
// degree slope whose seabed meets the keel tip 10 m ahead, for `seconds`.
Grounding run_aground(BoatSim& boat, double seconds) {
    TowRequest request;
    request.speed = 5.0 * kKnot;
    const TowResult tow = solve_tow(boat, request);
    REQUIRE(tow.converged);
    Grounding g;
    g.thrust = tow.tow_force_world.x();
    const Vec3 tip = lowest_point(boat, tow.state);
    boat.set_bathymetry(slope(tip.z(), tip.x() + 10.0, 10.0));
    boat.set_aero(false);
    boat.thrust().set_thrust(g.thrust);
    Simulation& sim = boat.start(tow.state);
    const double ke0 = kinetic_energy(boat.body(), sim.state());
    const auto steps = static_cast<int>(std::lround(seconds / Simulation::kFixedDt));
    for (int i = 0; i < steps; ++i) {
        boat.step();
        const State& s = sim.state();
        if (!finite(s)) {
            g.finite = false;
            break;
        }
        const GroundingResult& r = boat.grounding().last_result();
        if (r.contacts > 0 && std::isnan(g.first_contact_x)) g.first_contact_x = s.position.x();
        g.max_penetration = std::max(g.max_penetration, r.max_penetration);
        g.max_vertical_speed =
            std::max(g.max_vertical_speed, std::abs(s.world_linear_velocity().z()));
        g.max_kinetic_ratio = std::max(g.max_kinetic_ratio, kinetic_energy(boat.body(), s) / ke0);
        g.furthest_x = std::max(g.furthest_x, s.position.x());
    }
    g.rest_x = sim.state().position.x();
    return g;
}

}  // namespace

TEST_CASE("grounding: bilinear bathymetry samples between pixel centres and clamps outside") {
    // 3 x 2 grid, pixel 2 m, pixel (0, 0) at world (10, 20).
    const Bathymetry b(3, 2, 2.0, 10.0, 20.0, {0.0F, 1.0F, 3.0F, 2.0F, 3.0F, 7.0F});
    CHECK(b.height(10.0, 20.0) == doctest::Approx(0.0));
    CHECK(b.height(14.0, 18.0) == doctest::Approx(7.0));
    CHECK(b.height(11.0, 19.0) == doctest::Approx(1.5));
    const SeabedSample s = b.sample(11.0, 19.0);
    CHECK(s.dzdx == doctest::Approx(0.5));
    CHECK(s.dzdy == doctest::Approx(-1.0));
    CHECK(b.height(-100.0, 100.0) == doctest::Approx(0.0));
    CHECK(b.sample(-100.0, 100.0).dzdx == 0.0);
    CHECK(b.max_height(10.0, 11.0, 19.0, 20.0) == doctest::Approx(3.0));
    CHECK(b.max_height(-50.0, -40.0, 100.0, 110.0) == doctest::Approx(0.0));
    CHECK(Bathymetry().height(0.0, 0.0) == -std::numeric_limits<double>::infinity());
}

TEST_CASE("grounding: the bay's start depth matches meta.json") {
    const Bathymetry bay = Bathymetry::load(kDataDir / "scenes" / "bay" / "meta.json");
    CHECK(bay.width() == 1024);
    CHECK(bay.rows() == 1024);
    MESSAGE("start seabed z " << bay.height(0.0, -1600.0) << " m");
    CHECK(std::abs(bay.height(0.0, -1600.0) + 31.62) < 0.01);
}

TEST_CASE("grounding: contact points reach the keel tip") {
    const std::vector<Vec3> points = load_contact_points(kBoatDir);
    const BoatHydrostatics hydro = load_boat_hydrostatics(kBoatDir);
    double mesh_low = std::numeric_limits<double>::infinity();
    for (const Vec3& v : hydro.mesh.vertices) mesh_low = std::min(mesh_low, v.z());
    double low = std::numeric_limits<double>::infinity();
    for (const Vec3& p : points) low = std::min(low, p.z());
    MESSAGE(points.size() << " contact points of " << hydro.mesh.vertex_count()
                          << " mesh vertices");
    CHECK(low == mesh_low);
    CHECK(points.size() < hydro.mesh.vertex_count() / 2);
}

TEST_CASE("grounding: J/80 at 5 kn onto a 10 degree slope stops within a boat length") {
    const auto boat = BoatSim::load(kBoatDir, naca0012());
    const GroundingParams& p = boat->grounding().params();
    MESSAGE("mass " << p.mass << " kg, stiffness " << p.stiffness() << " N/m, damping "
                    << p.damping() << " N*s/m");
    const Grounding g = run_aground(*boat, 40.0);
    const State s = boat->simulation().state();
    const GroundingResult& r = boat->grounding().last_result();
    MESSAGE("thrust " << g.thrust << " N, first contact at x " << g.first_contact_x
                      << ", furthest x " << g.furthest_x << ", rest at x " << g.rest_x << ", max |vz| "
                      << g.max_vertical_speed << " m/s, max KE/KE0 " << g.max_kinetic_ratio
                      << ", max penetration " << g.max_penetration << " m, heel "
                      << heel_angle(s.rotation()) / kDeg << " deg, contacts " << r.contacts
                      << ", normal force " << r.normal_force << " N");
    REQUIRE(g.finite);
    REQUIRE_FALSE(std::isnan(g.first_contact_x));
    CHECK(g.furthest_x - g.first_contact_x < kBoatLength);
    CHECK(horizontal_speed(s) < 0.02);
    CHECK(g.max_vertical_speed < 0.5);
    CHECK(g.max_kinetic_ratio < 1.05);
    CHECK(g.max_penetration < 0.05);
    CHECK(r.contacts > 0);
    CHECK(std::abs(heel_angle(s.rotation())) < 45.0 * kDeg);

    SUBCASE("a thrust astern backs it off") {
        boat->thrust().set_thrust(-2.0 * g.thrust);
        Simulation& sim = boat->simulation();
        for (int i = 0; i < static_cast<int>(30.0 / Simulation::kFixedDt); ++i) boat->step();
        MESSAGE("astern: x " << sim.state().position.x() << ", contacts "
                             << boat->grounding().last_result().contacts);
        REQUIRE(finite(sim.state()));
        CHECK(sim.state().position.x() < g.rest_x - 5.0);
        CHECK(boat->grounding().last_result().contacts == 0);
    }
    SUBCASE("heeled by the crew it comes off") {
        const CrewParams& crew = boat->crew().params();
        boat->update_crew(crew.hiking_y(), crew.centre_z - crew.hiking_drop, CrewPosture::hiking);
        Simulation& sim = boat->simulation();
        double max_heel = 0.0;
        for (int i = 0; i < static_cast<int>(30.0 / Simulation::kFixedDt); ++i) {
            sim.step();
            max_heel = std::max(max_heel, std::abs(heel_angle(sim.state().rotation())));
        }
        const Vec3 moved = sim.state().position - s.position;
        MESSAGE("crew heel: moved " << std::hypot(moved.x(), moved.y()) << " m, max heel "
                                    << max_heel / kDeg << " deg, contacts "
                                    << boat->grounding().last_result().contacts);
        REQUIRE(finite(sim.state()));
        CHECK(std::hypot(moved.x(), moved.y()) > 2.0);
        CHECK(boat->grounding().last_result().contacts == 0);
    }
}

TEST_CASE("grounding: with no seabed in reach the trajectory is bit-identical to none") {
    auto run = [](std::shared_ptr<const Bathymetry> seabed) {
        BoatSimOptions options;
        options.bathymetry = std::move(seabed);
        const auto boat = BoatSim::load(kBoatDir, naca0012(), options);
        boat->set_wind(std::make_shared<const UniformWind>(5.0, 135.0));
        Controls c;
        c.mainsheet = 0.7;
        c.jib_sheet_starboard = 0.7;
        boat->set_controls(c);
        State initial = boat->equilibrium().pose;
        initial.linear_velocity = Vec3(2.0, 0.0, 0.0);
        boat->start(initial);
        for (int i = 0; i < 1000; ++i) boat->step();
        return boat->simulation().state();
    };
    const State none = run(nullptr);
    const State deep = run(constant(-1000.0));
    const State empty = run(std::make_shared<const Bathymetry>());
    for (const State* other : {&deep, &empty}) {
        CHECK(std::memcmp(none.position.data(), other->position.data(), sizeof(double) * 3) == 0);
        CHECK(std::memcmp(none.orientation.coeffs().data(), other->orientation.coeffs().data(),
                          sizeof(double) * 4) == 0);
        CHECK(std::memcmp(none.linear_velocity.data(), other->linear_velocity.data(),
                          sizeof(double) * 3) == 0);
    }

    const auto boat = BoatSim::load(kBoatDir, naca0012());
    CHECK_FALSE(boat->bathymetry());
    Wrench w;
    boat->grounding().accumulate(boat->equilibrium().pose, 0.0, w);
    CHECK(w.force == Vec3::Zero());
    CHECK(w.moment == Vec3::Zero());
}

TEST_CASE("grounding: the bay start is clear of the seabed") {
    BoatSimOptions options;
    options.bathymetry = std::make_shared<const Bathymetry>(
        Bathymetry::load(kDataDir / "scenes" / "bay" / "meta.json"));
    const auto boat = BoatSim::load(kBoatDir, naca0012(), options);
    State initial = boat->equilibrium().pose;
    initial.position.x() = 0.0;
    initial.position.y() = -1600.0;
    boat->start(initial);
    for (int i = 0; i < 200; ++i) {
        boat->step();
        CHECK_FALSE(boat->grounding().last_result().tested);
    }
}

TEST_CASE("grounding: step cost with and without contact" * doctest::skip(kTimingUnreliable)) {
    using Clock = std::chrono::steady_clock;
    constexpr int kSteps = 2000;
    auto time_steps = [](BoatSim& boat) {
        const auto begin = Clock::now();
        for (int i = 0; i < kSteps; ++i) boat.step();
        return std::chrono::duration<double, std::micro>(Clock::now() - begin).count() / kSteps;
    };
    auto fresh = [](std::shared_ptr<const Bathymetry> seabed) {
        BoatSimOptions options;
        options.bathymetry = std::move(seabed);
        auto boat = BoatSim::load(kBoatDir, naca0012(), options);
        boat->set_aero(false);
        boat->start(boat->equilibrium().pose);
        return boat;
    };
    const auto none = fresh(nullptr);
    const auto deep = fresh(constant(-30.0));
    const double none_us = time_steps(*none);
    const double deep_us = time_steps(*deep);
    const auto grounded = BoatSim::load(kBoatDir, naca0012());
    run_aground(*grounded, 30.0);
    REQUIRE(grounded->grounding().last_result().contacts > 0);
    const double contact_us = time_steps(*grounded);
    Wrench w;
    const State& s = grounded->simulation().state();
    GroundingModule& module = grounded->grounding();
    const auto begin = Clock::now();
    for (int i = 0; i < kSteps; ++i) module.accumulate(s, 0.0, w);
    const double module_us =
        std::chrono::duration<double, std::micro>(Clock::now() - begin).count() / kSteps;
    MESSAGE("step: no bathymetry " << none_us << " us, deep water " << deep_us
                                   << " us, aground " << contact_us << " us (grounding module "
                                   << module_us << " us, " << module.last_result().contacts
                                   << " contacts)");
    CHECK(deep_us < 1.2 * none_us + 2.0);
}
