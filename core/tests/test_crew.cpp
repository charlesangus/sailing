#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <memory>
#include <numbers>
#include <stdexcept>

#include "sail/boat_sim.hpp"
#include "sail/crew.hpp"
#include "sail/foil.hpp"

namespace {

using namespace sail;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);
const std::filesystem::path kBoatDir = kDataDir / "boats" / "j80";

constexpr double kKnot = 0.514444;
constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kDt = Simulation::kFixedDt;

std::shared_ptr<const FoilSection> naca0012() {
    static const auto section = std::make_shared<const FoilSection>(
        FoilSection::load(kDataDir / "foils" / "naca0012.csv"));
    return section;
}

CrewParams j80_params() {
    const MassModel model = MassModel::load(kBoatDir, MassConfiguration::sailing);
    return load_crew_params(kBoatDir, model);
}

ApparentWind wind(double aws, double awa_deg) {
    ApparentWind w;
    w.aws = aws;
    w.awa = awa_deg * kDeg;
    w.tack = awa_deg >= 0.0 ? Tack::Starboard : Tack::Port;
    return w;
}

int steps_to_settle(CrewController& crew, double heel, const ApparentWind& w, int limit) {
    int n = 0;
    while (n < limit && crew.position().y != crew.target_y(heel, w)) {
        crew.step(kDt, heel, w);
        ++n;
    }
    return n;
}

// A constant roll moment about the body x axis.
class HeelingMoment final : public ForceModule {
public:
    explicit HeelingMoment(double moment) : moment_(moment) {}
    void accumulate(const State&, double, Wrench& wrench) override { wrench.moment.x() += moment_; }

private:
    double moment_;
};

double mean_heel_after_run(bool crew_on, double moment, double seconds, double* final_y) {
    const auto boat = BoatSim::load(kBoatDir, naca0012());
    State initial = boat->equilibrium().pose;
    Simulation& sim = boat->start(initial);
    HeelingMoment heeling(moment);
    sim.add_module(heeling);
    // Wind from the south over a boat heading east: over the starboard side, 90 degrees.
    const UniformWind wind_field(10.0, 180.0);
    const int steps = static_cast<int>(std::lround(seconds / kDt));
    double sum = 0.0;
    int count = 0;
    for (int i = 0; i < steps; ++i) {
        if (crew_on) boat->step_crew(kDt, wind_field);
        sim.step();
        if (i >= steps - static_cast<int>(2.0 / kDt)) {
            sum += heel_angle(sim.state().rotation());
            ++count;
        }
    }
    *final_y = boat->mass_model().crew_position().y();
    return sum / count;
}

}  // namespace

TEST_CASE("crew rail geometry comes from the dossier") {
    const CrewParams p = j80_params();
    CHECK(p.rail_y == doctest::Approx(1.2145 - 0.20));
    CHECK(p.hiking_y() == doctest::Approx(p.rail_y + 0.30));
    CHECK(p.rail_y < 0.5 * 2.488);
    CHECK(p.centre_z > 0.6);
}

TEST_CASE("crew is at the windward rail at 20 degrees heel and centred upright") {
    CrewController crew(j80_params());
    const ApparentWind w = wind(10.0, 90.0);
    steps_to_settle(crew, 20.0 * kDeg, w, 1000);
    CHECK(crew.position().y == doctest::Approx(-crew.params().hiking_y()));
    CHECK(crew.position().posture == CrewPosture::hiking);
    CHECK(crew.position().z == doctest::Approx(crew.params().centre_z - crew.params().hiking_drop));

    CrewController upright(j80_params());
    for (int i = 0; i < 1000; ++i) upright.step(kDt, 0.0, w);
    CHECK(upright.position().y == 0.0);
    CHECK(upright.position().posture == CrewPosture::sitting);
    CHECK(upright.position().z == doctest::Approx(upright.params().centre_z));
}

TEST_CASE("crew position is proportional to heel between the threshold and full") {
    CrewController crew(j80_params());
    const ApparentWind w = wind(10.0, 90.0);
    const double mid = 0.5 * (crew.params().heel_threshold + crew.params().heel_full);
    CHECK(crew.target_y(mid, w) == doctest::Approx(-0.5 * crew.params().hiking_y()));
    CHECK(crew.target_y(15.0 * kDeg, w) == doctest::Approx(-crew.params().hiking_y()));
    CHECK(crew.target_y(30.0 * kDeg, w) == doctest::Approx(-crew.params().hiking_y()));
    CHECK(crew.target_y(-20.0 * kDeg, wind(10.0, -90.0)) == doctest::Approx(crew.params().hiking_y()));
}

TEST_CASE("crew crosses to the new windward rail in about the transit time") {
    CrewController crew(j80_params());
    steps_to_settle(crew, 20.0 * kDeg, wind(10.0, 90.0), 1000);
    REQUIRE(crew.position().y < 0.0);
    const ApparentWind port = wind(10.0, -90.0);
    const int n = steps_to_settle(crew, 20.0 * kDeg, port, 2000);
    CHECK(crew.position().y == doctest::Approx(crew.params().hiking_y()));
    CHECK(std::abs(n * kDt - crew.params().transit_time) <= kDt + 1e-9);
    CHECK(crew.params().transit_time == doctest::Approx(2.0));
}

TEST_CASE("crew rate limit is stepped by dt and reset per run") {
    CrewController crew(j80_params());
    const ApparentWind w = wind(10.0, 90.0);
    crew.step(0.1, 20.0 * kDeg, w);
    CHECK(crew.position().y == doctest::Approx(-crew.params().max_speed() * 0.1));
    crew.step(0.0, 20.0 * kDeg, w);
    crew.step(-1.0, 20.0 * kDeg, w);
    CHECK(crew.position().y == doctest::Approx(-crew.params().max_speed() * 0.1));
    crew.reset();
    CHECK(crew.position().y == 0.0);
    crew.reset(10.0);
    CHECK(crew.position().y == doctest::Approx(crew.params().hiking_y()));
}

TEST_CASE("crew stays centred in light air and downwind even when heeled") {
    CrewController light(j80_params());
    for (int i = 0; i < 1000; ++i) light.step(kDt, 20.0 * kDeg, wind(4.0 * kKnot, 90.0));
    CHECK(light.position().y == 0.0);

    CrewController downwind(j80_params());
    for (int i = 0; i < 1000; ++i) downwind.step(kDt, 20.0 * kDeg, wind(10.0, 150.0));
    CHECK(downwind.position().y == 0.0);

    CrewController up(j80_params());
    for (int i = 0; i < 1000; ++i) up.step(kDt, 20.0 * kDeg, wind(10.0 * kKnot * 2.0, 90.0));
    CHECK(up.position().y != 0.0);
}

TEST_CASE("update_crew moves the CG by m_crew / M_total times the move") {
    const auto boat = BoatSim::load(kBoatDir, naca0012());
    Simulation& sim = boat->start(boat->equilibrium().pose);
    sim.step();
    const MassModel& model = boat->mass_model();
    const Vec3 cg0 = model.cg();
    const Mat3 inertia0 = model.inertia();
    const State before = sim.state();
    const double dy = -1.3;
    boat->update_crew(dy, model.crew_position().z());

    const double fraction = model.crew_mass() / model.total_mass();
    CHECK((model.cg() - cg0).y() == doctest::Approx(fraction * dy));
    CHECK((model.cg() - cg0).x() == doctest::Approx(0.0).epsilon(1e-9));
    CHECK((model.cg() - cg0).z() == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(sim.body().mass == doctest::Approx(model.total_mass()));
    CHECK((sim.body().inertia - inertia0).norm() > 1.0);
    CHECK(sim.body().inertia(0, 0) == doctest::Approx(model.inertia()(0, 0)));
    CHECK(boat->hydrostatics().params().reference_point.y() == doctest::Approx(model.cg().y()));

    const Vec3 shift = model.cg() - cg0;
    const Vec3 expected = before.position + before.orientation * shift;
    CHECK((sim.state().position - expected).norm() < 1e-12);
    CHECK((sim.state().position + sim.state().orientation * (-shift) - before.position).norm() <
          1e-12);
}

TEST_CASE("crew listeners see each move") {
    const auto boat = BoatSim::load(kBoatDir, naca0012());
    int calls = 0;
    CrewPosture seen = CrewPosture::sitting;
    boat->add_crew_listener([&](const Vec3& cg, const Vec3& crew, CrewPosture posture) {
        ++calls;
        seen = posture;
        CHECK(cg.y() == doctest::Approx(boat->mass_model().cg().y()));
        CHECK(crew.y() == doctest::Approx(-1.0));
    });
    boat->update_crew(-1.0, 0.8, CrewPosture::hiking);
    CHECK(calls == 1);
    CHECK(seen == CrewPosture::hiking);
}

TEST_CASE("auto-hiking reduces the steady heel under a heeling moment") {
    constexpr double kMoment = -4000.0;
    double y_off = 0.0;
    double y_on = 0.0;
    const double heel_off = mean_heel_after_run(false, kMoment, 12.0, &y_off);
    const double heel_on = mean_heel_after_run(true, kMoment, 12.0, &y_on);
    MESSAGE("steady heel crew off " << heel_off / kDeg << " deg, on " << heel_on / kDeg
                                    << " deg, crew y " << y_on);
    CHECK(y_off == 0.0);
    CHECK(y_on < -0.5);
    CHECK(std::abs(heel_on) < std::abs(heel_off));
    CHECK(std::abs(heel_off) > 3.0 * kDeg);
}

TEST_CASE("the crew judges the wind at the rig, not at the CG near the water") {
    const auto boat = BoatSim::load(kBoatDir, naca0012());
    const Vec3 r = boat->rig_point();
    MESSAGE("rig point " << r.transpose() << " m from the CG");
    CHECK(r.z() > 3.0);
    const LogProfileWind log_wind(6.0 * kKnot, 180.0);
    State upright = boat->equilibrium().pose;
    const double at_rig = apparent_wind(upright, r, log_wind, 0.0).aws;
    const double at_cg = apparent_wind(upright, Vec3::Zero(), log_wind, 0.0).aws;
    MESSAGE("log-profile 6 kn: " << at_rig / kKnot << " kn at the rig, " << at_cg / kKnot
                                 << " kn at the CG");
    CHECK(at_rig > 1.5 * at_cg);
}

TEST_CASE("reset puts the crew, the CG and the mass properties back as loaded") {
    const auto boat = BoatSim::load(kBoatDir, naca0012());
    const auto fresh = BoatSim::load(kBoatDir, naca0012());
    boat->start(boat->equilibrium().pose);
    boat->update_crew(-1.0, 0.8, CrewPosture::hiking);
    REQUIRE(boat->mass_model().cg() != fresh->mass_model().cg());
    boat->reset();
    fresh->reset();
    CHECK(boat->mass_model().cg() == fresh->mass_model().cg());
    CHECK(boat->body().inertia == fresh->body().inertia);
    CHECK(boat->crew().position().y == 0.0);
    CHECK(boat->equilibrium().pose.position == fresh->equilibrium().pose.position);
    CHECK_THROWS_AS(boat->simulation(), std::logic_error);
}
