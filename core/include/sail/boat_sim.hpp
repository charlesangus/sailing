#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sail/appendages.hpp"
#include "sail/controls.hpp"
#include "sail/crew.hpp"
#include "sail/damping.hpp"
#include "sail/equilibrium.hpp"
#include "sail/foil.hpp"
#include "sail/gravity.hpp"
#include "sail/hull_forces.hpp"
#include "sail/hull_resistance.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/mass_model.hpp"
#include "sail/rigid_body.hpp"
#include "sail/sails.hpp"
#include "sail/sheets.hpp"
#include "sail/simulation.hpp"
#include "sail/wind.hpp"
#include "sail/windage.hpp"

namespace sail {

struct BoatSimOptions {
    // The model hull_resistance.json names when empty.
    std::optional<HullResistanceKind> resistance;
    // The directory of the orc_<type>.csv sail tables; boat_dir/../../sails when empty.
    std::optional<std::filesystem::path> sail_data_dir;
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

// A boat dossier assembled into the whole sailing boat: gravity, hydrostatics, damping,
// canoe-body hull forces, keel, rudder, a body-x thrust (zero until set), the sheet controller,
// the sail set and the windage, all about the sailing configuration's CG, in flat still water,
// with the crew controller moving the crew mass. Only the sailing configuration is offered
// because the hull forces take their displacement, CLR and sinkage from hull_hydro.json
// conditions.sailing. The wind is a parameter, calm until set_wind(). The modules are registered
// with the Simulation by reference, so a BoatSim is neither copyable nor movable.
//
// With aero off (set_aero(false)) the sheet controller, sails and windage are left out of
// evaluate() and of the Simulation start() builds: the bare hull that solve_tow() and
// run_turn() use.
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

    SailSet& sails() { return sails_; }
    const SailSet& sails() const { return sails_; }
    SheetController& sheets() { return sheets_; }
    const SheetController& sheets() const { return sheets_; }
    WindageModule& windage() { return windage_; }
    const WindageModule& windage() const { return windage_; }

    CrewController& crew() { return crew_; }
    const CrewController& crew() const { return crew_; }

    const std::shared_ptr<const WindField>& wind() const { return wind_; }
    // Gives `wind` to the sails, sheets and windage and to step_crew(dt). Throws
    // std::invalid_argument on a null wind.
    void set_wind(std::shared_ptr<const WindField> wind);

    // The tiller to the rudder, the sail controls to the sheet controller.
    void set_controls(const Controls& controls);

    // Takes effect in evaluate() at once and in the Simulation at the next start().
    void set_aero(bool on) { aero_ = on; }
    bool aero() const { return aero_; }

    // Called after every crew move with the new CG and crew position (hull frame) and the posture,
    // after the modules BoatSim owns have followed, for anything else that needs the crew.
    using CrewListener =
        std::function<void(const Vec3& cg, const Vec3& crew_position, CrewPosture posture)>;
    void add_crew_listener(CrewListener listener) { crew_listeners_.push_back(std::move(listener)); }

    // Moves the crew point mass to (y, z), hull frame: recomputes the mass properties, then moves
    // the CG reference point of gravity, hydrostatics, hull, appendages, sails and windage, gives
    // the windage the crew's position and posture, and calls the listeners.
    // In a running Simulation the body is replaced by Simulation::set_body with the CG shift: the
    // hull's own motion is preserved (every hull point keeps its position and velocity), so
    // velocity is carried across, not momentum, which changes with the new mass distribution. It
    // does not move the CrewController.
    void update_crew(double y, double z, CrewPosture posture = CrewPosture::sitting);

    // Steps the crew controller by dt for the running Simulation's heel and the apparent wind in
    // `wind` at rig_point(), then calls update_crew() if the crew moved at least
    // CrewParams::update_min_move since the last update or changed posture. Call once per
    // Simulation::step(), before it, with dt = Simulation::kFixedDt. Throws std::logic_error before
    // start().
    const CrewPosition& step_crew(double dt, const WindField& wind);
    // The same in the boat's own wind.
    const CrewPosition& step_crew(double dt) { return step_crew(dt, *wind_); }

    // m, body frame from the CG: the sail plan's area-weighted centre of effort as the sails
    // were last placed, or the CG with no sail area. The crew judges the wind there because the
    // wind near the water, under the log profile, is far lighter than what drives the rig.
    Vec3 rig_point() const;

    // Back to the state a fresh load leaves, apart from crew listeners: no Simulation, the crew
    // controller and the crew mass on the centreline with the mass properties and every module's
    // CG reference following, default controls, the sheet controller reset with the reef out, the
    // sail set reset, the rudder centred with its time base cleared, zero thrust, aero on and a
    // calm wind. A run that starts from here depends only on its own inputs, not on earlier runs.
    void reset();

    // One fixed step: step_crew(Simulation::kFixedDt), then Simulation::step(). Throws
    // std::logic_error before start().
    void step();
    // Simulation::advance(real_dt) with step_crew(Simulation::kFixedDt) before each fixed step it
    // runs, so a real-time caller moves the crew exactly as step() does. Returns the number of
    // steps. Throws std::logic_error before start().
    int advance(double real_dt);

    // Flat-water equilibrium of the sailing mass, hydrostatics alone.
    Equilibrium equilibrium(const EquilibriumRequest& request = {}) const;

    // Sum of every module's wrench at `state` and time `t`, body frame about the CG, leaving
    // each module's last_result() at that state. Steps the rudder actuator and the reef as a
    // Simulation step would when `t` is later than the previous call's.
    Wrench evaluate(const State& state, double t);

    // A new Simulation from `initial` with every module registered, replacing any earlier one:
    // the sheet controller first, so it trims the sails before they are evaluated, then gravity,
    // hydrostatics, damping, hull, keel, rudder, thrust, and with aero on the sails and the
    // windage. The crew goes back to the centreline, or to lateral position `crew_y` (clamped to
    // the hiking range), the crew controller is reset there, and the sheet controller is reset
    // with the reef at its control's value. Extra modules may be added to the returned
    // Simulation.
    Simulation& start(const State& initial);
    Simulation& start(const State& initial, double crew_y);
    // Throws std::logic_error before start().
    Simulation& simulation();

private:
    // Everything update_crew() does except the Simulation; returns the CG shift.
    Vec3 apply_crew(double y, double z, CrewPosture posture);
    Simulation& start_with_crew(const State& initial);

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
    std::shared_ptr<const WindField> wind_;
    SailSet sails_;
    SheetController sheets_;
    WindageModule windage_;
    bool aero_ = true;
    CrewController crew_;
    Vec3 pushed_crew_;
    CrewPosture pushed_posture_ = CrewPosture::sitting;
    std::vector<CrewListener> crew_listeners_;
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
// the rudder centred, its actuator time base cleared, first. The hull is towed bare: aero is
// off for the solve and set back as it was afterwards. The modules are left evaluated at the
// balance.
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

// Runs the turn on `boat` with aero off, which it leaves off, replacing its Simulation;
// boat.thrust() keeps the turn's thrust, so the Simulation may be stepped on. The boat's modules are left evaluated at the last state.
TurnResult run_turn(BoatSim& boat, const TurnRequest& request);

}  // namespace sail
