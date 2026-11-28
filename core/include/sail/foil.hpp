#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "sail/hull_forces.hpp"
#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace sail {

// Two-dimensional lift and drag coefficients of a symmetric section over the full circle of angle
// of attack. Built from a table over 0 to 180 degrees and mirrored, so Cl is odd and Cd even in
// the angle; interpolated linearly.
class FoilSection {
public:
    // alpha_deg strictly increasing from exactly 0 to exactly 180, with cl zero at both ends.
    // Throws std::invalid_argument otherwise, or if there is no Cl maximum below 90 degrees.
    FoilSection(std::vector<double> alpha_deg, std::vector<double> cl, std::vector<double> cd);

    // Reads a CSV with the header `alpha_deg,cl,cd`; see data/foils/README.md. Throws
    // BoatDataError on a missing or malformed file, or a table the constructor rejects.
    static FoilSection load(const std::filesystem::path& file);

    // alpha in rad, any value; wrapped to [-pi, pi].
    double cl(double alpha) const;
    double cd(double alpha) const;

    // rad: the first local maximum of Cl above zero.
    double stall_angle() const { return stall_; }
    // rad, measured from pi: the first local extreme of Cl below 180 degrees in reverse flow.
    double reverse_stall_angle() const { return reverse_stall_; }
    // Per rad: least-squares slope through the origin over the points up to stall_angle() / 2.
    double lift_slope() const { return lift_slope_; }

private:
    std::vector<double> alpha_;
    std::vector<double> cl_;
    std::vector<double> cd_;
    double stall_ = 0.0;
    double reverse_stall_ = 0.0;
    double lift_slope_ = 0.0;
};

// A straight-tapered lifting surface. Positions are hull frame, directions body frame (the two
// share axes).
struct FoilGeometry {
    // m, root to tip.
    double span = 1.0;
    double root_chord = 1.0;
    double tip_chord = 1.0;
    // rad, positive with the tip's quarter chord downstream (along chord_direction) of the root's.
    double quarter_chord_sweep = 0.0;
    // m^2.
    double planform_area = 1.0;
    // Effective aspect ratio over geometric: 2 for a fin closed off by a hull or a free surface at
    // its root, 1 for two free ends.
    double mirror_factor = 1.0;
    // Quarter-chord point of the mean aerodynamic chord, at the spanwise centre of area.
    Vec3 quarter_chord_centre = Vec3::Zero();
    // Unit vector root to tip.
    Vec3 span_direction = Vec3(0.0, 0.0, -1.0);
    // Unit vector leading edge to trailing edge, normal to span_direction.
    Vec3 chord_direction = Vec3(-1.0, 0.0, 0.0);

    double geometric_aspect_ratio() const { return span * span / planform_area; }
    double effective_aspect_ratio() const { return mirror_factor * geometric_aspect_ratio(); }
    // tan(sweep_half) = tan(sweep_quarter) + (tip_chord - root_chord) / (4 span).
    double half_chord_sweep() const;
    // 2/3 c_root (1 + l + l^2) / (1 + l), taper l = tip_chord / root_chord.
    double mean_aerodynamic_chord() const;
};

struct FoilParams {
    FoilGeometry geometry;
    // Oswald efficiency e in the induced drag CL^2 / (pi AR_eff e).
    double oswald_efficiency = 0.9;
    // m/s. Forces fade in as smoothstep(V / blend_speed), V the in-plane inflow speed.
    double blend_speed = 0.1;
};

// Everything one evaluation computed.
struct FoilForces {
    // Inflow (water relative to the foil), body frame, with the spanwise part removed, m/s.
    Vec3 inflow = Vec3::Zero();
    // |inflow|, and the removed spanwise component (along span_direction), m/s.
    double speed = 0.0;
    double spanwise_speed = 0.0;
    // rad in [-pi, pi]; near +-pi for flow from the trailing edge.
    double alpha = 0.0;
    // Finite-wing coefficients on planform_area. cd includes cd_induced.
    double cl = 0.0;
    double cd = 0.0;
    double cd_induced = 0.0;
    // Centre of pressure as a fraction of the mean aerodynamic chord from its leading edge.
    double cp_chord_fraction = 0.25;
    // Hull frame.
    Vec3 centre_of_pressure = Vec3::Zero();
    // Body frame, N, after the low-speed blend. force = lift + drag.
    Vec3 lift = Vec3::Zero();
    Vec3 drag = Vec3::Zero();
    Vec3 force = Vec3::Zero();
};

// Lift and drag of a finite foil from its section table.
//
// The inflow u is projected onto the plane normal to the span; the spanwise part is ignored, with
// no cosine-law correction, since the sweep enters through the lift slope instead. With c the
// chord direction and n = c x s (s the span direction), alpha = atan2(u.n, u.c), so the full
// circle is covered and alpha = +-pi is flow from the trailing edge. Lift acts along u x s
// (perpendicular to u in that plane) and drag along u, both on 0.5 rho |u|^2 S. The forces are
// written as 0.5 rho S |u| (CL u x s + CD u), so nothing divides by the speed.
//
// Finite span: the Helmbold lift slope a = 2 pi A / (2 + sqrt(A^2 (1 + tan^2 sweep_half) + 4)),
// A the effective aspect ratio, scales the angle at which the section table is read by
// k = a / a0 (a0 the section slope): in attached flow the finite wing at alpha has the section's
// coefficients at k alpha, so it reaches the section's Cl maximum at the stall angle divided by k.
// Past that, up to 90 degrees, the scale on the angle rises to 1 as a smoothstep, so the
// separated-flow coefficients near 90 degrees are the table's. Reverse flow is treated the same
// way about 180 degrees with the reverse stall angle. Either stall angle is capped halfway between
// the section's and 90 degrees. Induced drag CL^2 / (pi A e) is weighted by 1 - that smoothstep,
// so it fades out as the flow separates.
//
// The centre of pressure is on the mean aerodynamic chord through quarter_chord_centre, at 0.25 of
// it in attached flow, moving with the same smoothstep to 0.5 at 90 degrees and on to 0.75 in
// attached reverse flow.
class Foil {
public:
    // Throws std::invalid_argument on a null section or a non-positive span, chord, area, mirror
    // factor, efficiency or blend speed, or on parallel or zero span and chord directions.
    Foil(std::shared_ptr<const FoilSection> section, FoilParams params);

    // `inflow`: water velocity relative to the foil's quarter-chord centre, body frame, m/s.
    FoilForces evaluate(const Vec3& inflow, double density) const;

    // Finite-wing coefficients at alpha (rad); drag_coefficient includes the induced drag.
    double lift_coefficient(double alpha) const;
    double drag_coefficient(double alpha) const;
    double induced_drag_coefficient(double alpha) const;
    double cp_chord_fraction(double alpha) const;
    // Hull frame.
    Vec3 centre_of_pressure(double alpha) const;

    // Per rad, Helmbold.
    double lift_slope() const { return lift_slope_; }
    // rad: where the finite wing reaches the section's Cl maximum, ahead and in reverse flow
    // (the latter measured from pi).
    double stall_angle() const { return stall_; }
    double reverse_stall_angle() const { return reverse_stall_; }

    // Re-aims the foil, e.g. a rudder turned about its stock. chord_direction is made normal to
    // span_direction. Throws as the constructor does.
    void set_pose(const Vec3& quarter_chord_centre, const Vec3& span_direction,
                  const Vec3& chord_direction);

    const FoilParams& params() const { return params_; }
    const FoilSection& section() const { return *section_; }

private:
    struct Coefficients {
        double cl = 0.0;
        double cd = 0.0;
        double cd_induced = 0.0;
        double cp_chord_fraction = 0.25;
    };
    Coefficients coefficients(double alpha) const;

    std::shared_ptr<const FoilSection> section_;
    FoilParams params_;
    double lift_slope_ = 0.0;
    double attached_scale_ = 1.0;
    double stall_ = 0.0;
    double reverse_stall_ = 0.0;
    double induced_factor_ = 0.0;
    double mac_ = 0.0;
};

struct FoilForceParams {
    FoilParams foil;
    // kg/m^3
    double density = 1025.0;
    // Hull-frame location of the body's reference point (the CG).
    Vec3 reference_point = Vec3::Zero();
};

// Reads appendages.json `foils.<name>` (span, root_chord, tip_chord, quarter_chord_sweep,
// planform_area, geometric_aspect_ratio and effective_aspect_ratio for the mirror factor,
// quarter_chord_centre) and hull_resistance.json `water.density` from a boat dossier directory.
// The foil hangs straight down with its chord running aft. Throws BoatDataError on a missing or
// malformed file or field, or a non-positive size.
FoilForceParams load_foil_force_params(const std::filesystem::path& boat_dir,
                                       const std::string& name,
                                       const Vec3& reference_point = Vec3::Zero());

// One foil as a force module. The inflow is the water velocity less the body velocity at the
// quarter-chord centre, v + omega x r with r from the CG; the force acts at the centre of
// pressure. Allocates nothing per step.
class FoilForceModule final : public ForceModule {
public:
    // A null `current` means still water.
    FoilForceModule(std::shared_ptr<const FoilSection> section, FoilForceParams params,
                    std::shared_ptr<const WaterCurrent> current = nullptr);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    const FoilForces& last_result() const { return last_; }
    const Wrench& last_wrench() const { return last_wrench_; }
    const Foil& foil() const { return foil_; }
    Foil& foil() { return foil_; }

    // Moves the reference point (hull frame); see HullForceModule::set_reference_point.
    void set_reference_point(const Vec3& point) { reference_point_ = point; }

private:
    Foil foil_;
    double density_;
    Vec3 reference_point_;
    std::shared_ptr<const WaterCurrent> current_;
    FoilForces last_;
    Wrench last_wrench_;
};

}  // namespace sail
