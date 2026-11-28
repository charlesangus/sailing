#pragma once

#include <filesystem>
#include <memory>

#include "sail/controls.hpp"
#include "sail/foil.hpp"
#include "sail/hull_forces.hpp"
#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace sail {

struct RudderParams {
    // The blade at zero rudder angle.
    FoilForceParams foil;
    // Hull frame point on the stock axis, and the axis as a unit vector (body frame). The rudder
    // angle is right-handed about this axis.
    Vec3 stock_position = Vec3::Zero();
    Vec3 stock_axis = Vec3(0.0, 0.0, 1.0);
    // rad and rad/s.
    double max_angle = 0.6;
    double rate = 1.0;
    // The keel turns the rudder's inflow by k * alpha_keel; see RudderForceModule.
    double downwash_factor = 0.0;
};

// Reads the blade as load_foil_force_params(boat_dir, "rudder") does, appendages.json
// `foils.rudder` stock_position and stock_axis_direction, and controls.json `rudder` max_angle
// (deg), max_rate (deg/s) and keel_downwash_factor. Throws BoatDataError on a missing or
// malformed file or field, a non-positive angle or rate, a negative factor or a zero axis.
RudderParams load_rudder_params(const std::filesystem::path& boat_dir,
                                const Vec3& reference_point = Vec3::Zero());

// The rudder as a force module: a Foil turned about its stock by a rate-limited actuator, working
// in the keel's downwash.
//
// Rudder angle: each accumulate() first steps the actuator by the simulation time elapsed since
// the previous accumulate() (Simulation calls it once per fixed step, so by kFixedDt); the first
// call, a repeated time or a time that runs backwards steps nothing. The blade is then posed by
// rotating its neutral quarter-chord centre, span and chord directions about the stock axis.
//
// Downwash: the keel's angle of attack alpha_k is computed here from the same state, so module
// order does not matter. The rudder's inflow (water relative to the blade's quarter-chord centre)
// is rotated about the blade's span direction by eps = k * alpha_k, which lowers the rudder's
// angle of attack by exactly eps. Beyond the keel's finite-wing stall angle alpha_k is held at
// that angle and fades to 0 at 90 degrees as the keel's circulation collapses, and with flow
// from the keel's trailing edge there is no downwash, since the rudder is then upstream.
class RudderForceModule final : public ForceModule {
public:
    // `keel` gives the keel's pose and stall angle, copied. A null `current` means still water.
    // Throws std::invalid_argument on a non-positive density, angle or rate, a negative or
    // non-finite downwash factor or a zero stock axis.
    RudderForceModule(std::shared_ptr<const FoilSection> section, RudderParams params,
                      const Foil& keel, std::shared_ptr<const WaterCurrent> current = nullptr);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    void set_controls(const Controls& controls) { actuator_.set_command(controls); }
    // Puts the rudder at `angle` (rad) holding there, and forgets the previous accumulate()'s
    // time, so the next accumulate() steps nothing whatever its time.
    void reset(double angle = 0.0);
    const RudderActuator& actuator() const { return actuator_; }
    RudderActuator& actuator() { return actuator_; }
    // rad, the angle the latest accumulate() used.
    double angle() const { return posed_angle_; }

    // The blade's evaluation at the latest accumulate(); its alpha is the downwashed one.
    const FoilForces& last_result() const { return last_; }
    const Wrench& last_wrench() const { return last_wrench_; }
    // N*m, the hydrodynamic moment on the blade about the stock axis at the latest accumulate():
    // ((r_cp - r_stock) x F) . stock_axis, with F and the centre of pressure from last_result().
    // Positive tends to increase the rudder angle (+delta, right-handed about the stock axis).
    double rudder_hinge_moment_Nm() const { return hinge_moment_; }
    // rad: the keel's angle of attack and the downwash angle eps at the latest accumulate().
    double keel_alpha() const { return keel_alpha_; }
    double downwash() const { return downwash_; }

    const Foil& foil() const { return foil_; }
    const RudderParams& params() const { return params_; }

    // Throws std::invalid_argument if negative or not finite.
    void set_downwash_factor(double k);
    // Moves the reference point (hull frame); see HullForceModule::set_reference_point.
    void set_reference_point(const Vec3& point) { params_.foil.reference_point = point; }

private:
    void pose(double angle);

    Foil foil_;
    RudderParams params_;
    FoilGeometry keel_;
    double keel_stall_;
    std::shared_ptr<const WaterCurrent> current_;
    RudderActuator actuator_;
    double posed_angle_ = 0.0;
    bool has_time_ = false;
    double last_time_ = 0.0;
    double keel_alpha_ = 0.0;
    double downwash_ = 0.0;
    double hinge_moment_ = 0.0;
    FoilForces last_;
    Wrench last_wrench_;
};

// The J/80's keel (a plain FoilForceModule) and rudder, built from a boat dossier, sharing one
// water current and one reference point. Registered with a Simulation by reference, so it is
// neither copyable nor movable.
class Appendages {
public:
    // Reads load_foil_force_params(boat_dir, "keel") and load_rudder_params(boat_dir). Throws as
    // those do.
    Appendages(std::shared_ptr<const FoilSection> keel_section,
               std::shared_ptr<const FoilSection> rudder_section,
               const std::filesystem::path& boat_dir, const Vec3& reference_point = Vec3::Zero(),
               std::shared_ptr<const WaterCurrent> current = nullptr);

    Appendages(const Appendages&) = delete;
    Appendages& operator=(const Appendages&) = delete;

    FoilForceModule& keel() { return keel_; }
    const FoilForceModule& keel() const { return keel_; }
    RudderForceModule& rudder() { return rudder_; }
    const RudderForceModule& rudder() const { return rudder_; }

    void set_controls(const Controls& controls) { rudder_.set_controls(controls); }
    double rudder_angle() const { return rudder_.angle(); }
    // N*m; see RudderForceModule::rudder_hinge_moment_Nm.
    double rudder_hinge_moment_Nm() const { return rudder_.rudder_hinge_moment_Nm(); }

    // Moves both modules' reference point, e.g. to the new CG; see Simulation::set_body.
    void set_reference_point(const Vec3& point);

    // Registers the keel and the rudder with `sim`; this object must outlive it.
    void add_to(Simulation& sim);

private:
    FoilForceModule keel_;
    RudderForceModule rudder_;
};

}  // namespace sail
