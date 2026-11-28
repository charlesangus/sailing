#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "common.hpp"

namespace sailcli {

namespace {

using Json = nlohmann::ordered_json;

constexpr const char* kUsage =
    "usage: sailsim gz --boat DIR [--config measurement|sailing|orc] [--step DEG]\n"
    "                  [--trim level|free]\n"
    "\n"
    "Sweeps heel from 0 to 180 degrees in flat water at the configuration's displacement. At each\n"
    "heel the heel is held and the boat sinks until buoyancy equals weight; with --trim free it\n"
    "also trims until the trimming moment vanishes, with --trim level (default) trim stays zero\n"
    "(the DWL level fore and aft). Prints JSON.\n"
    "\n"
    "  --boat DIR     boat dossier directory, e.g. data/boats/j80\n"
    "  --config NAME  measurement (default), sailing or orc; see 'sailsim hydro --help'\n"
    "  --step DEG     heel increment, 0 < DEG <= 90, default 5\n"
    "  --trim MODE    level (default) or free\n"
    "\n"
    "Conventions, in the body frame x forward, y to port, z up: heel is a rotation about +x, so\n"
    "positive heel puts starboard down; trim is a following rotation about the world y axis,\n"
    "positive bow down. rm_Nm is the hydrostatic moment about the CG, positive when it opposes\n"
    "the heel; gz_m is rm_Nm over the weight. avs_deg is the first angle past the GZ maximum at\n"
    "which GZ turns negative, bracketed on the sweep and refined by root finding.\n"
    "rm_1deg_Nm is solved at exactly 1 degree.\n";

using Point = sail::GzPoint;

int gz(const Options& options) {
    const ConfigChoice config = parse_config(options.get("--config"));
    const double step = options.number("--step", 5.0);
    if (!(step > 0.0 && step <= 90.0)) throw UsageError("--step must be in (0, 90]");
    const std::string trim_mode = options.get("--trim").value_or("level");
    if (trim_mode != "level" && trim_mode != "free") {
        throw UsageError("--trim must be level or free, got '" + trim_mode + "'");
    }
    const bool free_trim = trim_mode == "free";

    const LoadedBoat boat = load_boat(options, config.configuration);
    const double mass = boat.model.total_mass();

    const std::vector<Point> sweep =
        sail::gz_curve(boat.hydro.mesh, boat.params, mass, rad(step), free_trim);
    const Point one =
        sail::solve_gz_point(boat.hydro.mesh, boat.params, mass, rad(1.0), free_trim,
                             &sweep.front().eq);

    std::size_t imax = 0;
    for (std::size_t i = 1; i < sweep.size(); ++i) {
        if (sweep[i].gz > sweep[imax].gz) imax = i;
    }

    const int sign_changes = sail::gz_sign_changes(sweep);

    bool converged = one.eq.converged;
    for (const Point& p : sweep) converged = converged && p.eq.converged;
    const std::optional<double> avs = sail::angle_of_vanishing_stability(
        boat.hydro.mesh, boat.params, mass, sweep, free_trim, &converged);

    Json points = Json::array();
    for (const Point& p : sweep) {
        Json j;
        j["heel_deg"] = deg(p.heel);
        j["rm_Nm"] = p.eq.righting_moment;
        j["gz_m"] = p.gz;
        j["trim_deg"] = deg(p.eq.trim);
        if (!p.eq.converged) j["converged"] = false;
        points.push_back(std::move(j));
    }

    Json out;
    out["config"] = config.name;
    out["trim_mode"] = trim_mode;
    out["total_mass_kg"] = mass;
    out["step_deg"] = step;
    out["converged"] = converged;
    out["rm_1deg_Nm"] = one.eq.righting_moment;
    out["gz_max_m"] = sweep[imax].gz;
    out["gz_max_heel_deg"] = deg(sweep[imax].heel);
    out["avs_deg"] = avs ? Json(deg(*avs)) : Json(nullptr);
    out["sign_changes"] = sign_changes;
    out["heel_sign"] = "positive starboard down (rotation about body +x)";
    out["trim_sign"] = "positive bow down";
    out["points"] = std::move(points);
    std::puts(out.dump(2).c_str());
    return converged ? 0 : 1;
}

}  // namespace

int run_gz(const Args& args) {
    return guarded("gz", kUsage, gz, args, {"--boat", "--config", "--step", "--trim"});
}

}  // namespace sailcli
