#pragma once

#include <filesystem>
#include <memory>

#include "sail/damping.hpp"
#include "sail/hull_resistance.hpp"
#include "sail/mass_model.hpp"
#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace sail {

// Velocity of the water, world frame, m/s, at a world point and time t seconds.
class WaterCurrent {
public:
    virtual ~WaterCurrent() = default;
    virtual Vec3 velocity(const Vec3& world_point, double t) const = 0;
};

class StillWater final : public WaterCurrent {
public:
    Vec3 velocity(const Vec3&, double) const override { return Vec3::Zero(); }
};

struct HullForceParams {
    // kg/m^3
    double density = 1025.0;
    // Hull-frame centre of lateral resistance, where every force of this module acts.
    Vec3 clr_point = Vec3::Zero();
    // Hull-frame location of the body's reference point (the CG); see
    // HydrostaticParams::reference_point.
    Vec3 reference_point = Vec3::Zero();
    // Side force k * u_ax * u_lat, N*s^2/m^2.
    double side_force_coefficient = 0.0;
    // Induced drag = factor * |side force| * |sin(leeway)|.
    double induced_drag_factor = 0.5;
    // m/s. Side force and induced drag fade in as smoothstep(U / blend_speed).
    double blend_speed = 0.1;
    // Horizontal lateral velocity u_lat at the CLR (N*s/m, N*s^2/m^2) and body-frame yaw rate
    // (N*m*s/rad, N*m*s^2/rad^2).
    DampingCoefficient sway;
    DampingCoefficient yaw;
    // Fraction f of the integrator's ideal-fluid Munk yaw moment that this module cancels.
    double munk_correction_fraction = 0.0;
    // kg, the surge and sway diagonal of the body's added mass.
    double added_mass_surge = 0.0;
    double added_mass_sway = 0.0;
};

// Reads hull_forces.json, hull_hydro.json `conditions.sailing.result` (lwl, tc, lcb_x_canoe,
// sinkage_m) and hull_resistance.json `water.density` from a boat dossier directory and derives
// the coefficients as hull_forces.json describes. `reference_point` is the CG, hull frame, and
// `body` supplies the surge and sway added mass. Throws BoatDataError on a missing or malformed
// file or field, or a negative or non-positive value.
HullForceParams load_hull_force_params(const std::filesystem::path& boat_dir,
                                       const Vec3& reference_point, const BodyProperties& body);

// The same for the model's boat directory, CG and body properties. Throws std::invalid_argument
// unless the model is in the sailing configuration, the one whose displacement conditions.sailing
// describes.
HullForceParams load_hull_force_params(const MassModel& model);

// What the latest accumulate() computed.
struct HullForceResult {
    // Horizontal water-relative speed at the CLR, m/s, and its components along the horizontal
    // heading (u_ax) and 90 degrees to port of it (u_lat).
    double speed = 0.0;
    double u_ax = 0.0;
    double u_lat = 0.0;
    // atan2(u_lat, u_ax), rad: positive when drifting to port, near +-pi in sternway.
    double leeway = 0.0;
    // rad, positive starboard down.
    double heel = 0.0;
    HullResistance resistance;
    // World frame, N.
    Vec3 resistance_world = Vec3::Zero();
    Vec3 side_force_world = Vec3::Zero();
    Vec3 induced_drag_world = Vec3::Zero();
    // Body frame: sway damping force at the CLR, along the horizontal 90 degrees to port of the
    // heading, and the yaw damping moment.
    Vec3 damping_force = Vec3::Zero();
    Vec3 damping_moment = Vec3::Zero();
    // Body-frame yaw moment of the Munk correction, N*m.
    double munk_correction = 0.0;
    // Everything above, body frame, about the CG.
    Wrench wrench;
};

// Canoe-body resistance, low-aspect-ratio side force with its induced drag, and sway and yaw
// damping, all applied at the CLR. Allocates nothing per step.
//
// The horizontal water-relative velocity at the CLR is split along the horizontal heading into
// u_ax and u_lat. Resistance is the model's total at (|u_ax|, |heel|), opposite the horizontal
// velocity, so it follows the velocity into sternway. Side force is k u_ax u_lat perpendicular to
// the horizontal velocity, signed to oppose u_lat ahead and astern alike, and the induced drag
// opposes the velocity. The leeway angle is never divided by speed: its sine and cosine come from
// atan2, and below blend_speed the leeway terms fade to zero.
//
// Force modules add no Munk terms, yet this one adds the yaw moment
// +f (A_sway - A_surge) u v, with u and v the body-frame CG velocity the integrator uses (no
// current, no yaw-rate arm), so it is exactly -f times the integrator's term before the fade-in by
// smoothstep(U / blend_speed). It is not a second Munk term but a lift correction to
// the integrator's: the integrator applies the full ideal-fluid moment -(A_sway - A_surge) u v from
// the constant added mass, which assumes attached potential flow over the whole body. A lifting
// canoe body sheds its cross-flow from about the maximum section aft, and slender-body theory with
// that Kutta-like condition gives only about half the ideal-fluid moment, so this module cancels
// the fraction f of it.
class HullForceModule final : public ForceModule {
public:
    // A null `current` means still water.
    HullForceModule(std::shared_ptr<const HullResistanceModel> resistance, HullForceParams params,
                    std::shared_ptr<const WaterCurrent> current = nullptr);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    const HullForceResult& last_result() const { return last_; }
    const HullForceParams& params() const { return params_; }
    const HullResistanceModel& resistance_model() const { return *resistance_; }

    // Moves the reference point (hull frame), e.g. to the new CG after the crew moves; see
    // Simulation::set_body.
    void set_reference_point(const Vec3& point) { params_.reference_point = point; }

private:
    std::shared_ptr<const HullResistanceModel> resistance_;
    HullForceParams params_;
    std::shared_ptr<const WaterCurrent> current_;
    HullForceResult last_;
};

}  // namespace sail
