#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sail/appendages.hpp"
#include "sail/damping.hpp"
#include "sail/equilibrium.hpp"
#include "sail/foil.hpp"
#include "sail/gravity.hpp"
#include "sail/hull_forces.hpp"
#include "sail/hull_resistance.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/mass_model.hpp"
#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace sail {

struct BoatSimOptions {
    // The model hull_resistance.json names when empty.
    std::optional<HullResistanceKind> resistance;
};

// A constant force along the body x axis at the CG, N.
class BodyThrustModule final : public ForceModule {
public:
    void accumulate(const State&, double, Wrench& wrench) override { wrench.force.x() += thrust_; }
    void set_thrust(double thrust) { thrust_ = thrust; }
    double thrust() const { return thrust_; }

private:
    double thrust_ = 0.0;
};

// A boat dossier assembled into its force modules: gravity, hydrostatics, damping, canoe-body
// hull forces, keel, rudder and a body-x thrust (zero until set), all about the sailing
// configuration's CG, in still flat water. Only the sailing configuration is offered because the
// hull forces take their displacement, CLR and sinkage from hull_hydro.json conditions.sailing.
// The modules are registered with the Simulation by reference, so a BoatSim is neither copyable
// nor movable.
class BoatSim {
public:
    // `section` is the keel and rudder section table. Throws as the loaders it calls do.
    BoatSim(const std::filesystem::path& boat_dir, std::shared_ptr<const FoilSection> section,
            const BoatSimOptions& options = {});

    static std::unique_ptr<BoatSim> load(const std::filesystem::path& boat_dir,
                                         std::shared_ptr<const FoilSection> section,
                                         const BoatSimOptions& options = {});

    BoatSim(const BoatSim&) = delete;
    BoatSim& operator=(const BoatSim&) = delete;

    const MassModel& mass_model() const { return model_; }
    const BodyProperties& body() const { return body_; }
    const BoatHydrostatics& boat_hydrostatics() const { return hydro_; }
    const HydrostaticParams& hydrostatic_params() const { return params_; }

    GravityModule& gravity() { return gravity_; }
    HydrostaticsModule& hydrostatics() { return hydrostatics_; }
    const HydrostaticsModule& hydrostatics() const { return hydrostatics_; }
    DampingModule& damping() { return damping_; }
    HullForceModule& hull() { return hull_; }
    const HullForceModule& hull() const { return hull_; }
    Appendages& appendages() { return appendages_; }
    const Appendages& appendages() const { return appendages_; }
    BodyThrustModule& thrust() { return thrust_; }
    const BodyThrustModule& thrust() const { return thrust_; }

    // Flat-water equilibrium of the sailing mass, hydrostatics alone.
    Equilibrium equilibrium(const EquilibriumRequest& request = {}) const;

    // Sum of every module's wrench at `state` and time `t`, body frame about the CG, leaving
    // each module's last_result() at that state. Steps the rudder actuator as a Simulation step
    // would when `t` is later than the previous call's.
    Wrench evaluate(const State& state, double t);

    // A new Simulation from `initial` with every module registered, replacing any earlier one.
    // Extra modules may be added to the returned Simulation.
    Simulation& start(const State& initial);
    // Throws std::logic_error before start().
    Simulation& simulation();

private:
    MassModel model_;
    BodyProperties body_;
    BoatHydrostatics hydro_;
    HydrostaticParams params_;
    GravityModule gravity_;
    HydrostaticsModule hydrostatics_;
    DampingModule damping_;
    HullForceModule hull_;
    Appendages appendages_;
    BodyThrustModule thrust_;
    std::optional<Simulation> sim_;
};

// A steady tow in flat water: the boat moves horizontally at `speed` (m/s, negative for
// sternway) with its heading along world +x, its velocity `leeway` (rad, positive to port) off
// the heading, and its attitude held at `heel` (rad, positive starboard down) with zero yaw.
struct TowRequest {
    double speed = 0.0;
    double heel = 0.0;
    double leeway = 0.0;
};

// World-frame force on one component, resolved along and across the track.
struct TowComponent {
    // N, positive opposing the motion: -F . d with d the unit horizontal velocity.
    double resistance = 0.0;
    // N, F . n with n the horizontal unit 90 degrees to port of d.
    double side_force = 0.0;
    // N, world z.
    double vertical = 0.0;
};

struct TowResult {
    bool converged = false;
    int iterations = 0;
    // Pose and velocity at the balance.
    State state;
    // Static equilibrium at the same heel (hydrostatics only, free trim), for the sinkage and
    // trim the tow adds.
    Equilibrium still;
    // rad, positive bow down.
    double trim = 0.0;
    // World horizontal force the tow applies at the CG, N, the negative of the horizontal sum.
    Vec3 tow_force_world = Vec3::Zero();
    // N*m about world x and z through the CG that hold the heel and heading: the negative of
    // the boat's own moment about those axes.
    double roll_constraint_Nm = 0.0;
    double yaw_constraint_Nm = 0.0;
    TowComponent canoe;
    TowComponent keel;
    TowComponent rudder;
    TowComponent damping;
    TowComponent total;
};

// Finds the heave and trim at which the vertical force and the pitch moment about world y vanish
// with every module acting at the towed velocity, by Newton iteration from the static
// equilibrium, the heel and heading held. The tow force is then the direct horizontal balance at
// that state, applied at the CG, so the boat would not accelerate. The thrust is set to zero and
// the rudder centred, its actuator time base cleared, first. The modules are left evaluated at
// the balance.
TowResult solve_tow(BoatSim& boat, const TowRequest& request);

// A turning manoeuvre in flat water from the tow balance at `speed` (m/s, negative for sternway)
// along world +x, under a constant thrust along the body x axis at the CG equal to the
// straight-line tow resistance at that speed (zero at zero speed). The tiller is set at t = 0 and
// the rudder follows it at its rate limit.
struct TurnRequest {
    double speed = 0.0;
    // -1..1, +1 the tiller fully to port, which turns the boat to starboard.
    double tiller = 1.0;
    // s
    double duration = 60.0;
};

struct TurnSample {
    double t = 0.0;
    // World horizontal CG position, m.
    double x = 0.0;
    double y = 0.0;
    // rad, unwrapped, the angle of the body x axis about world z from world +x.
    double heading = 0.0;
    // rad/s about world z, negative to starboard.
    double yaw_rate = 0.0;
    // m/s, horizontal.
    double speed = 0.0;
    // rad, heel_angle() of the attitude.
    double heel = 0.0;
    // rad, the CG velocity's angle off the heading, positive to port.
    double drift = 0.0;
    double rudder = 0.0;
};

struct TurnCircle {
    double centre_x = 0.0;
    double centre_y = 0.0;
    double radius = 0.0;
    // RMS of the track's distance from the circle, m.
    double rms = 0.0;
};

// Means over the steady window, with the least-squares circle through the CG track over it.
struct TurnSteady {
    // "last full revolution" or "second half of the run".
    std::string window;
    double t_begin = 0.0;
    double t_end = 0.0;
    double heading_change = 0.0;
    double yaw_rate = 0.0;
    double speed = 0.0;
    double heel = 0.0;
    double drift = 0.0;
    double rudder = 0.0;
    std::optional<TurnCircle> circle;
};

struct TurnResult {
    // False when the straight-line tow balance did not converge; nothing else is then set.
    bool tow_converged = false;
    // N along body x.
    double thrust = 0.0;
    // False if any state or rudder value was not finite at any step; the run stops there.
    bool finite = true;
    std::optional<double> first_nonfinite_time;
    // The tow balance state and then each step, in order.
    std::vector<TurnSample> samples;
    // rad, last heading minus first.
    double heading_change = 0.0;
    std::optional<double> time_to_90deg;
    // The last full revolution of heading when the run has one, otherwise the second half of the
    // run when the heading has turned at least 90 degrees in all. Empty when neither, or when not
    // finite.
    std::optional<TurnSteady> steady;
    // N*m, signed, and the time of the step whose accumulate() computed it.
    double max_rudder_hinge_moment = 0.0;
    double max_rudder_hinge_moment_time = 0.0;
};

// Runs the turn on `boat`, replacing its Simulation; boat.thrust() keeps the turn's thrust, so the
// Simulation may be stepped on. The boat's modules are left evaluated at the last state.
TurnResult run_turn(BoatSim& boat, const TurnRequest& request);

}  // namespace sail
