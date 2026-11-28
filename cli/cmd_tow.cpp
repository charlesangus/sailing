#include <charconv>
#include <cmath>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "common.hpp"
#include "sail/boat_sim.hpp"

namespace sailcli {

namespace {

using Json = nlohmann::ordered_json;

constexpr const char* kUsage =
    "usage: sailsim tow --boat DIR [--speeds A..B | S1,S2,...] [--heel DEG] [--leeway DEG]\n"
    "                   [--model orc|dsyhs] [--section FILE]\n"
    "\n"
    "Tows the boat (sailing configuration: crew aboard) in flat still water at each speed and\n"
    "prints the steady force balance as JSON. The heel and the heading are held; heave and trim\n"
    "are solved, by Newton iteration from the hydrostatic equilibrium, so that the vertical force\n"
    "and the pitching moment vanish with every module acting at the towed velocity. The tow force\n"
    "is then the direct horizontal balance at that state, applied at the CG, so the boat does not\n"
    "accelerate: no time stepping is involved.\n"
    "\n"
    "  --boat DIR      boat dossier directory, e.g. data/boats/j80\n"
    "  --speeds LIST   knots: A..B for every whole knot from A to B, or a comma list;\n"
    "                  default 1..7; negative is sternway\n"
    "  --heel DEG      held heel, positive starboard down, default 0\n"
    "  --leeway DEG    angle of the velocity off the heading, positive to port, default 0\n"
    "  --model NAME    canoe-body resistance model, orc or dsyhs; default the dossier's\n"
    "  --section FILE  keel and rudder section table, default DIR/../../foils/naca0012.csv\n"
    "\n"
    "Per speed: resistance_N is the tow force along the track, side_force_N the force to port of\n"
    "the track it also has to supply, both split by component. The canoe's resistance is its\n"
    "residuary and viscous resistance plus the induced drag of its side force, and its side force\n"
    "includes the sway damping. Keel and rudder drag is split into profile (section Cd) and\n"
    "induced. heave_m is the CG's rise over the hydrostatic equilibrium at the same heel, trim_deg\n"
    "the trim (positive bow down) and trim_change_deg its change; roll_moment_Nm and\n"
    "yaw_moment_Nm are what holding the heel and heading takes, about world x and z.\n";

double parse_number(std::string_view text, const char* what) {
    double v = 0.0;
    const char* end = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(text.data(), end, v);
    if (text.empty() || ec != std::errc() || ptr != end || !std::isfinite(v)) {
        throw UsageError(std::string(what) + " expects numbers, got '" + std::string(text) + "'");
    }
    return v;
}

std::vector<double> parse_speeds(const std::string& text) {
    std::vector<double> out;
    if (const auto dots = text.find(".."); dots != std::string::npos) {
        const double a = parse_number(std::string_view(text).substr(0, dots), "--speeds");
        const double b = parse_number(std::string_view(text).substr(dots + 2), "--speeds");
        if (!(b >= a) || b - a > 100.0) throw UsageError("--speeds A..B needs A <= B <= A + 100");
        for (double s = a; s <= b + 1e-9; s += 1.0) out.push_back(s);
        return out;
    }
    std::string_view rest = text;
    while (true) {
        const auto comma = rest.find(',');
        out.push_back(parse_number(rest.substr(0, comma), "--speeds"));
        if (comma == std::string_view::npos) break;
        rest = rest.substr(comma + 1);
    }
    return out;
}

Json component(const sail::TowComponent& c) {
    Json j;
    j["resistance_N"] = c.resistance;
    j["side_force_N"] = c.side_force;
    j["vertical_N"] = c.vertical;
    return j;
}

Json foil(const sail::TowComponent& c, const sail::FoilForces& f) {
    Json j = component(c);
    const double drag = f.drag.norm();
    const double induced = f.cd > 0.0 ? drag * f.cd_induced / f.cd : 0.0;
    j["drag_profile_N"] = drag - induced;
    j["drag_induced_N"] = induced;
    j["alpha_deg"] = deg(f.alpha);
    return j;
}

int tow(const Options& options) {
    const std::vector<double> speeds = parse_speeds(options.get("--speeds").value_or("1..7"));
    sail::TowRequest request;
    request.heel = rad(options.number("--heel", 0.0));
    request.leeway = rad(options.number("--leeway", 0.0));
    if (std::abs(request.heel) > rad(90.0)) throw UsageError("--heel must be within +-90");
    if (std::abs(request.leeway) > rad(90.0)) throw UsageError("--leeway must be within +-90");

    sail::BoatSimOptions sim_options;
    sim_options.resistance = parse_resistance_model(options.get("--model"));
    const auto boat = load_boat_sim(options, sim_options);
    const char* model_name = resistance_model_name(boat->hull().resistance_model().kind());

    bool converged = true;
    Json points = Json::array();
    for (const double kn : speeds) {
        request.speed = kn * kKnot;
        const double sign = request.speed < 0.0 ? -1.0 : 1.0;
        const sail::Vec3 port_of_track =
            sign * sail::Vec3(-std::sin(request.leeway), std::cos(request.leeway), 0.0);
        const sail::TowResult r = sail::solve_tow(*boat, request);
        converged = converged && r.converged;
        const sail::HullForceResult& hull = boat->hull().last_result();

        Json canoe = component(r.canoe);
        canoe["residuary_N"] = hull.resistance.residuary_N;
        canoe["viscous_N"] = hull.resistance.viscous_N;
        canoe["induced_N"] = hull.induced_drag_world.norm();
        canoe["side_force_lift_N"] = hull.side_force_world.dot(port_of_track);
        canoe["fn"] = hull.resistance.fn;
        canoe["in_range"] = hull.resistance.in_range;

        Json j;
        j["speed_kn"] = kn;
        j["speed_mps"] = request.speed;
        j["converged"] = r.converged;
        j["iterations"] = r.iterations;
        j["resistance_N"] = r.total.resistance;
        j["side_force_N"] = r.total.side_force;
        j["tow_force_world_N"] = {r.tow_force_world.x(), r.tow_force_world.y()};
        j["heave_m"] = r.state.position.z() - r.still.pose.position.z();
        j["trim_deg"] = deg(r.trim);
        j["trim_change_deg"] = deg(r.trim - r.still.trim);
        j["roll_moment_Nm"] = r.roll_constraint_Nm;
        j["yaw_moment_Nm"] = r.yaw_constraint_Nm;
        Json parts;
        parts["canoe"] = std::move(canoe);
        parts["keel"] = foil(r.keel, boat->appendages().keel().last_result());
        parts["rudder"] = foil(r.rudder, boat->appendages().rudder().last_result());
        parts["damping"] = component(r.damping);
        j["components"] = std::move(parts);
        points.push_back(std::move(j));
    }

    Json out;
    out["config"] = "sailing";
    out["total_mass_kg"] = boat->body().mass;
    out["resistance_model"] = model_name;
    out["heel_deg"] = deg(request.heel);
    out["leeway_deg"] = deg(request.leeway);
    out["method"] =
        "heel and heading held; heave and trim solved for zero vertical force and pitch moment at "
        "the towed velocity; tow force = horizontal force balance, applied at the CG";
    out["converged"] = converged;
    out["points"] = std::move(points);
    std::puts(out.dump(2).c_str());
    return converged ? 0 : 1;
}

}  // namespace

int run_tow(const Args& args) {
    return guarded("tow", kUsage, tow, args,
                   {"--boat", "--speeds", "--heel", "--leeway", "--model", "--section"});
}

}  // namespace sailcli
