#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include "common.hpp"
#include "sail/boat_sim.hpp"

namespace sailcli {

namespace {

using Json = nlohmann::ordered_json;
using sail::Vec3;

constexpr const char* kUsage =
    "usage: sailsim turn --boat DIR [--speed KN] [--tiller T] [--duration S]\n"
    "                    [--model orc|dsyhs] [--section FILE]\n"
    "\n"
    "Turning manoeuvre in flat still water, sailing configuration. The boat starts upright at the\n"
    "tow balance for --speed (see 'sailsim tow') with that velocity along its heading, under a\n"
    "constant thrust along the body x axis at the CG equal to the straight-line tow resistance at\n"
    "that speed (astern for negative speed, none at zero). The tiller is set at t = 0 and the\n"
    "rudder follows it at its rate limit. Runs the full dynamic simulation for --duration seconds\n"
    "and prints JSON.\n"
    "\n"
    "  --boat DIR      boat dossier directory, e.g. data/boats/j80\n"
    "  --speed KN      initial speed in knots, default 5; negative is sternway\n"
    "  --tiller T      -1..1, +1 the tiller fully to port (turns to starboard), default 1\n"
    "  --duration S    seconds, 1..3600, default 60\n"
    "  --model NAME    canoe-body resistance model, orc or dsyhs; default the dossier's\n"
    "  --section FILE  keel and rudder section table, default DIR/../../foils/naca0012.csv\n"
    "\n"
    "The steady window is the last full revolution of heading when the run has one, otherwise\n"
    "the second half of the run when the heading has turned at least 90 degrees in all; the\n"
    "turning circle is a least-squares circle through the CG track over it, and the steady values\n"
    "are means over it. Without a window they are null. Yaw rate is about world z (negative is to\n"
    "starboard), heel positive starboard down, drift the angle of the CG's velocity off the\n"
    "heading, positive to port. final_yaw_moment_budget_Nm splits the body z moment at the last\n"
    "state by source, the integrator's rigid-body and added-mass terms included. finite is\n"
    "false, with the exit status 1, if any state or rudder value was not finite at any step.\n";

constexpr double kKnot = 0.514444;

Json nullable(const std::optional<double>& v) { return v ? Json(*v) : Json(nullptr); }

// particulars.json `loa.value`, empty when absent.
std::optional<double> read_loa(const std::filesystem::path& boat_dir) {
    std::ifstream in(boat_dir / "particulars.json");
    if (!in) return std::nullopt;
    const Json j = Json::parse(in, nullptr, false);
    if (j.is_discarded() || !j.contains("loa") || !j["loa"].contains("value") ||
        !j["loa"]["value"].is_number()) {
        return std::nullopt;
    }
    return j["loa"]["value"].get<double>();
}

int turn(const Options& options) {
    const double speed_kn = options.number("--speed", 5.0);
    const double tiller = options.number("--tiller", 1.0);
    const double duration = options.number("--duration", 60.0);
    if (std::abs(speed_kn) > 20.0) throw UsageError("--speed must be within +-20 kn");
    if (std::abs(tiller) > 1.0) throw UsageError("--tiller must be in [-1, 1]");
    if (!(duration >= 1.0 && duration <= 3600.0)) throw UsageError("--duration must be in [1, 3600]");

    sail::BoatSimOptions sim_options;
    sim_options.resistance = parse_resistance_model(options.get("--model"));
    const auto boat = load_boat_sim(options, sim_options);

    sail::TurnRequest request;
    request.speed = speed_kn * kKnot;
    request.tiller = tiller;
    request.duration = duration;
    const sail::TurnResult result = sail::run_turn(*boat, request);
    if (!result.tow_converged) throw std::runtime_error("straight-line tow balance did not converge");
    const bool all_finite = result.finite;
    const std::vector<sail::TurnSample>& samples = result.samples;
    sail::Simulation& sim = boat->simulation();
    const sail::TurnSample& last = samples.back();

    Json steady = nullptr;
    Json circle = nullptr;
    if (result.steady) {
        const sail::TurnSteady& st = *result.steady;
        const double initial = std::abs(request.speed);
        steady = Json::object();
        steady["window"] = st.window;
        steady["window_s"] = {st.t_begin, st.t_end};
        steady["window_heading_change_deg"] = deg(st.heading_change);
        steady["yaw_rate_deg_s"] = deg(st.yaw_rate);
        steady["speed_kn"] = st.speed / kKnot;
        steady["speed_loss_kn"] = (initial - st.speed) / kKnot;
        steady["speed_loss_pct"] = initial > 0.0 ? Json(100.0 * (initial - st.speed) / initial)
                                                 : Json(nullptr);
        steady["heel_deg"] = deg(st.heel);
        steady["drift_deg"] = deg(st.drift);
        steady["rudder_deg"] = deg(st.rudder);
        steady["diameter_from_yaw_rate_m"] =
            std::abs(st.yaw_rate) > 1e-9 ? Json(2.0 * st.speed / std::abs(st.yaw_rate))
                                         : Json(nullptr);
        if (st.circle) {
            const sail::TurnCircle& c = *st.circle;
            const std::optional<double> loa = read_loa(boat->mass_model().data().directory);
            circle = Json::object();
            circle["diameter_m"] = 2.0 * c.radius;
            circle["diameter_boat_lengths"] =
                loa ? Json(2.0 * c.radius / *loa) : Json(nullptr);
            circle["loa_m"] = nullable(loa);
            circle["centre_m"] = {c.centre_x, c.centre_y};
            circle["fit_rms_m"] = c.rms;
        }
    }

    Json budget = nullptr;
    if (all_finite) {
        const sail::State& s = sim.state();
        const sail::Wrench all = boat->evaluate(s, sim.time());
        const sail::HullForceResult& hull = boat->hull().last_result();
        const Vec3 arm = boat->hull().params().clr_point - boat->hull().params().reference_point;
        const double sway = arm.cross(hull.damping_force).z();
        sail::Vec6 nu;
        nu << s.linear_velocity, s.angular_velocity;
        const sail::Vec6 p = boat->body().generalised_mass() * nu;
        const Vec3 p_lin = p.head<3>();
        const Vec3 p_ang = p.tail<3>();
        const Vec3 omega = s.angular_velocity;
        budget = Json::object();
        budget["rudder"] = boat->appendages().rudder().last_wrench().moment.z();
        budget["keel"] = boat->appendages().keel().last_wrench().moment.z();
        budget["hull_side_force_and_drag"] =
            hull.wrench.moment.z() - hull.damping_moment.z() - sway - hull.munk_correction;
        budget["hull_sway_damping"] = sway;
        budget["hull_munk_correction"] = hull.munk_correction;
        budget["hull_yaw_damping"] = hull.damping_moment.z();
        budget["coriolis_and_munk"] = -(omega.cross(p_ang) + s.linear_velocity.cross(p_lin)).z();
        budget["sum"] = all.moment.z() + budget["coriolis_and_munk"].get<double>();
    }

    Json out;
    out["config"] = "sailing";
    out["total_mass_kg"] = boat->body().mass;
    out["resistance_model"] =
        boat->hull().resistance_model().kind() == sail::HullResistanceKind::orc ? "orc" : "dsyhs";
    out["speed_kn"] = speed_kn;
    out["tiller"] = tiller;
    out["rudder_command_deg"] = deg(boat->appendages().rudder().actuator().command());
    out["thrust_N"] = result.thrust;
    out["duration_s"] = duration;
    out["steps"] = samples.size() - 1;
    out["finite"] = all_finite;
    out["first_nonfinite_s"] = nullable(result.first_nonfinite_time);
    out["heading_change_deg"] = deg(result.heading_change);
    out["time_to_90deg_s"] = nullable(result.time_to_90deg);
    out["turning_circle"] = std::move(circle);
    out["steady"] = std::move(steady);
    out["max_rudder_hinge_moment_Nm"] = result.max_rudder_hinge_moment;
    out["max_rudder_hinge_moment_t_s"] = result.max_rudder_hinge_moment_time;
    Json final_state;
    final_state["t_s"] = last.t;
    final_state["position_m"] = {last.x, last.y};
    final_state["speed_kn"] = last.speed / kKnot;
    final_state["yaw_rate_deg_s"] = deg(last.yaw_rate);
    final_state["heel_deg"] = deg(last.heel);
    final_state["drift_deg"] = deg(last.drift);
    out["final"] = std::move(final_state);
    out["final_yaw_moment_budget_Nm"] = std::move(budget);
    std::puts(out.dump(2).c_str());
    return all_finite ? 0 : 1;
}

}  // namespace

int run_turn(const Args& args) {
    return guarded("turn", kUsage, turn, args,
                   {"--boat", "--speed", "--tiller", "--duration", "--model", "--section"});
}

}  // namespace sailcli
