#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <numbers>

#include "sail/boat_data.hpp"
#include "sail/foil.hpp"
#include "sail/rigid_body.hpp"

namespace {

using sail::Foil;
using sail::FoilForceModule;
using sail::FoilForces;
using sail::FoilParams;
using sail::FoilSection;
using sail::State;
using sail::Vec3;
using sail::Wrench;

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = kPi / 180.0;
constexpr double kRho = 1025.0;

const std::filesystem::path kDataDir(SAIL_TEST_DATA_DIR);

std::shared_ptr<const FoilSection> naca0012() {
    static const auto section =
        std::make_shared<const FoilSection>(FoilSection::load(kDataDir / "foils" / "naca0012.csv"));
    return section;
}

// Rectangular, unswept, free tips, chord along +x and span along +y, so n = c x s = +z.
FoilParams wing(double aspect_ratio, double oswald = 0.9) {
    FoilParams p;
    p.geometry.root_chord = 1.0;
    p.geometry.tip_chord = 1.0;
    p.geometry.span = aspect_ratio;
    p.geometry.planform_area = aspect_ratio;
    p.geometry.mirror_factor = 1.0;
    p.geometry.span_direction = Vec3(0.0, 1.0, 0.0);
    p.geometry.chord_direction = Vec3(1.0, 0.0, 0.0);
    p.oswald_efficiency = oswald;
    return p;
}

Vec3 inflow_at(double alpha, double speed) {
    return speed * Vec3(std::cos(alpha), 0.0, std::sin(alpha));
}

double helmbold(double ar) { return 2.0 * kPi * ar / (2.0 + std::sqrt(ar * ar + 4.0)); }

}  // namespace

TEST_CASE("NACA 0012 table is symmetric and finds its stall") {
    const FoilSection& s = *naca0012();
    for (double a = -180.0; a <= 180.0; a += 2.5) {
        CHECK(s.cl(-a * kDeg) == doctest::Approx(-s.cl(a * kDeg)));
        CHECK(s.cd(-a * kDeg) == doctest::Approx(s.cd(a * kDeg)));
    }
    CHECK(s.cl(0.0) == 0.0);
    CHECK(std::abs(s.cl(kPi)) < 1e-12);
    CHECK(s.cl(13.0 * kDeg) == doctest::Approx(1.2169));
    CHECK(s.cl(12.5 * kDeg) == doctest::Approx(0.5 * (1.2072 + 1.2169)));
    CHECK(s.cd(90.0 * kDeg) == doctest::Approx(1.8));
    CHECK(s.cl(370.0 * kDeg) == doctest::Approx(s.cl(10.0 * kDeg)));
    CHECK(s.stall_angle() == doctest::Approx(13.0 * kDeg));
    CHECK(s.reverse_stall_angle() == doctest::Approx(10.0 * kDeg));
    CHECK(s.lift_slope() == doctest::Approx(0.11 / kDeg));
}

TEST_CASE("FoilSection rejects a missing file") {
    CHECK_THROWS_AS(FoilSection::load(kDataDir / "foils" / "missing.csv"), sail::BoatDataError);
}

TEST_CASE("Low-angle lift slope matches Helmbold for AR 2 and AR 5") {
    for (const double ar : {2.0, 5.0}) {
        CAPTURE(ar);
        const Foil foil(naca0012(), wing(ar));
        const double expected = helmbold(ar);
        CHECK(foil.lift_slope() == doctest::Approx(expected));

        const double da = 2.0 * kDeg;
        const double slope = (foil.lift_coefficient(da) - foil.lift_coefficient(-da)) / (2.0 * da);
        CHECK(std::abs(slope / expected - 1.0) < 0.02);

        const double speed = 3.0;
        const FoilForces f = foil.evaluate(inflow_at(da, speed), kRho);
        const double q_s = 0.5 * kRho * speed * speed * ar;
        CHECK(std::abs(f.lift.norm() / (q_s * da) / expected - 1.0) < 0.02);
    }
}

TEST_CASE("Finite-wing stall moves to the section stall over a / a0, keeping its Cl maximum") {
    const double section_max = naca0012()->cl(naca0012()->stall_angle());
    for (const double ar : {2.0, 5.0}) {
        CAPTURE(ar);
        const Foil foil(naca0012(), wing(ar));
        const double k = foil.lift_slope() / naca0012()->lift_slope();
        CHECK(foil.stall_angle() == doctest::Approx(naca0012()->stall_angle() / k));
        double best = 0.0;
        double best_alpha = 0.0;
        for (double a = 0.0; a <= 60.0; a += 0.05) {
            const double cl = foil.lift_coefficient(a * kDeg);
            if (cl > best) {
                best = cl;
                best_alpha = a;
            }
        }
        const double stall_deg = foil.stall_angle() / kDeg;
        CHECK(std::abs(best_alpha - stall_deg) < 0.1);
        CHECK(std::abs(best / section_max - 1.0) < 0.1);
        CHECK(foil.lift_coefficient((stall_deg + 3.0) * kDeg) < best);
        CHECK(foil.lift_coefficient((stall_deg + 6.0) * kDeg) <
              foil.lift_coefficient((stall_deg + 3.0) * kDeg));
        CHECK(foil.lift_coefficient(-foil.stall_angle()) ==
              doctest::Approx(-foil.lift_coefficient(foil.stall_angle())));
        CHECK(foil.lift_coefficient(90.0 * kDeg) == doctest::Approx(naca0012()->cl(90.0 * kDeg)));
    }

    const std::filesystem::path boat = kDataDir / "boats" / "j80";
    for (const char* name : {"keel", "rudder"}) {
        CAPTURE(name);
        const Foil foil(naca0012(), sail::load_foil_force_params(boat, name).foil);
        double best = 0.0;
        for (double a = 0.0; a <= 60.0; a += 0.05) best = std::max(best, foil.lift_coefficient(a * kDeg));
        CHECK(std::abs(best / section_max - 1.0) < 0.1);
        MESSAGE(name << ": finite-wing stall " << foil.stall_angle() / kDeg << " deg, CL max "
                     << best);
    }
}

TEST_CASE("Foil force is zero at zero inflow and finite at tiny inflow") {
    const Foil foil(naca0012(), wing(3.0));
    const FoilForces zero = foil.evaluate(Vec3::Zero(), kRho);
    CHECK(zero.force.norm() == 0.0);
    CHECK(zero.speed == 0.0);

    const FoilForces tiny = foil.evaluate(inflow_at(5.0 * kDeg, 1e-6), kRho);
    CHECK(std::isfinite(tiny.force.x()));
    CHECK(std::isfinite(tiny.force.y()));
    CHECK(std::isfinite(tiny.force.z()));
    CHECK(tiny.force.norm() > 0.0);
    CHECK(tiny.force.norm() < 1e-9);
    CHECK(tiny.alpha == doctest::Approx(5.0 * kDeg));
}

TEST_CASE("Symmetric section gives no lift at zero angle of attack") {
    const Foil foil(naca0012(), wing(3.0));
    const FoilForces f = foil.evaluate(Vec3(4.0, 0.0, 0.0), kRho);
    CHECK(f.alpha == 0.0);
    CHECK(f.lift.norm() < 1e-12);
    CHECK(f.drag.x() > 0.0);
}

TEST_CASE("Lift flips with the sign of the angle of attack") {
    const Foil foil(naca0012(), wing(3.0));
    const FoilForces up = foil.evaluate(inflow_at(6.0 * kDeg, 3.0), kRho);
    const FoilForces down = foil.evaluate(inflow_at(-6.0 * kDeg, 3.0), kRho);
    CHECK(up.alpha == doctest::Approx(6.0 * kDeg));
    CHECK(up.lift.z() > 0.0);
    CHECK(down.lift.z() == doctest::Approx(-up.lift.z()));
    CHECK(down.lift.x() == doctest::Approx(up.lift.x()));
    CHECK(up.lift.dot(up.inflow) == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("Flow from astern gives little lift and drag along the inflow") {
    const Foil foil(naca0012(), wing(3.0));
    for (const double a : {180.0, 179.0, -179.0}) {
        CAPTURE(a);
        const Vec3 u = inflow_at(a * kDeg, 3.0);
        const FoilForces f = foil.evaluate(u, kRho);
        CHECK(std::abs(std::abs(f.alpha) - kPi) < 1.01 * kDeg);
        CHECK(std::abs(f.cl) < 0.1);
        CHECK(f.drag.normalized().dot(u.normalized()) == doctest::Approx(1.0));
    }
    const FoilForces dead = foil.evaluate(Vec3(-3.0, 0.0, 0.0), kRho);
    CHECK(dead.lift.norm() < 1e-9);
    CHECK(dead.drag.x() < 0.0);
}

TEST_CASE("Induced drag is CL^2 / (pi AR_eff e)") {
    FoilParams p = wing(2.0, 0.8);
    p.geometry.mirror_factor = 2.0;
    const Foil foil(naca0012(), p);
    const double alpha = 5.0 * kDeg;
    const double cl = foil.lift_coefficient(alpha);
    const double expected = cl * cl / (kPi * 4.0 * 0.8);
    const double k = foil.lift_slope() / naca0012()->lift_slope();
    CHECK(foil.induced_drag_coefficient(alpha) == doctest::Approx(expected));
    CHECK(foil.drag_coefficient(alpha) - naca0012()->cd(k * alpha) == doctest::Approx(expected));
    const FoilForces f = foil.evaluate(inflow_at(alpha, 2.0), kRho);
    CHECK(f.cd_induced == doctest::Approx(expected));
    CHECK(f.drag.norm() == doctest::Approx(0.5 * kRho * 4.0 * 2.0 * f.cd));
}

TEST_CASE("Centre of pressure moves aft past stall") {
    FoilParams p = wing(3.0);
    p.geometry.quarter_chord_centre = Vec3(0.25, 1.0, 0.0);
    const Foil foil(naca0012(), p);
    CHECK(foil.cp_chord_fraction(5.0 * kDeg) == doctest::Approx(0.25));
    CHECK(foil.cp_chord_fraction(foil.stall_angle()) == doctest::Approx(0.25));
    CHECK(foil.cp_chord_fraction(90.0 * kDeg) == doctest::Approx(0.5));
    CHECK(foil.cp_chord_fraction(178.0 * kDeg) == doctest::Approx(0.75));
    double previous = 0.25;
    for (double a = std::ceil(foil.stall_angle() / kDeg); a <= 90.0; a += 2.0) {
        const double x = foil.cp_chord_fraction(a * kDeg);
        CHECK(x > previous);
        previous = x;
    }
    CHECK(foil.centre_of_pressure(5.0 * kDeg).x() == doctest::Approx(0.25));
    CHECK(foil.centre_of_pressure(60.0 * kDeg).x() > 0.3);
    CHECK(foil.evaluate(inflow_at(60.0 * kDeg, 2.0), kRho).centre_of_pressure.x() ==
          doctest::Approx(foil.centre_of_pressure(60.0 * kDeg).x()));
}

TEST_CASE("J/80 keel module resists leeway and takes moments about the CG") {
    const std::filesystem::path boat = kDataDir / "boats" / "j80";
    const Vec3 cg(-0.3, 0.0, -0.1);
    const sail::FoilForceParams params = sail::load_foil_force_params(boat, "keel", cg);
    CHECK(params.density == doctest::Approx(1025.0));
    CHECK(params.foil.geometry.mirror_factor == doctest::Approx(2.0).epsilon(1e-3));
    CHECK(params.foil.geometry.quarter_chord_sweep == doctest::Approx(16.04 * kDeg));
    CHECK(params.foil.geometry.quarter_chord_centre.z() == doctest::Approx(-0.8639));

    FoilForceModule keel(naca0012(), params);
    State state;
    state.linear_velocity = Vec3(3.0, -0.15, 0.0);
    state.angular_velocity = Vec3(0.0, 0.0, 0.05);
    Wrench w;
    keel.accumulate(state, 0.0, w);

    const FoilForces& f = keel.last_result();
    const Vec3 arm = params.foil.geometry.quarter_chord_centre - cg;
    const Vec3 expected_inflow = -(state.linear_velocity + state.angular_velocity.cross(arm));
    CHECK(f.inflow.x() == doctest::Approx(expected_inflow.x()));
    CHECK(f.inflow.y() == doctest::Approx(expected_inflow.y()));
    CHECK(f.spanwise_speed == doctest::Approx(0.0));
    CHECK(f.force.y() > 0.0);
    CHECK(f.drag.x() < 0.0);
    CHECK(w.force.y() == doctest::Approx(f.force.y()));
    const Vec3 moment = (f.centre_of_pressure - cg).cross(f.force);
    CHECK(w.moment.x() == doctest::Approx(moment.x()));
    CHECK(w.moment.z() == doctest::Approx(moment.z()));
}
