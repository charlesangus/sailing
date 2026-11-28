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

struct Point {
    double heel = 0.0;
    sail::Equilibrium eq;
    double gz = 0.0;
};

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
    const double weight = mass * boat.params.gravity;

    const auto solve = [&](double heel_deg, const sail::Equilibrium* guess) {
        sail::EquilibriumRequest request;
        request.heel = rad(heel_deg);
        request.free_trim = free_trim;
        if (guess) {
            request.z = guess->pose.position.z();
            request.trim = free_trim ? guess->trim : 0.0;
        }
        Point p;
        p.heel = heel_deg;
        p.eq = sail::solve_equilibrium(boat.hydro.mesh, boat.params, mass, request);
        p.gz = p.eq.righting_moment / weight;
        return p;
    };

    std::vector<Point> sweep;
    const int count = static_cast<int>(std::ceil(180.0 / step - 1e-9));
    for (int i = 0; i <= count; ++i) {
        const double heel = std::min(180.0, i * step);
        sweep.push_back(solve(heel, sweep.empty() ? nullptr : &sweep.back().eq));
    }
    const Point one = solve(1.0, &sweep.front().eq);

    std::size_t imax = 0;
    for (std::size_t i = 1; i < sweep.size(); ++i) {
        if (sweep[i].gz > sweep[imax].gz) imax = i;
    }

    // 0 and 180 degrees are equilibria by symmetry, and what GZ the mesh leaves there is its
    // small port-starboard asymmetry, so only the open interval counts towards sign changes.
    constexpr double kZeroGz = 1e-5;
    int sign_changes = 0;
    int last_sign = 0;
    for (const Point& p : sweep) {
        if (p.heel <= 0.0 || p.heel >= 180.0) continue;
        const int sign = p.gz > kZeroGz ? 1 : (p.gz < -kZeroGz ? -1 : 0);
        if (sign != 0) {
            if (last_sign != 0 && sign != last_sign) ++sign_changes;
            last_sign = sign;
        }
    }

    std::optional<double> avs;
    bool converged = one.eq.converged;
    for (const Point& p : sweep) converged = converged && p.eq.converged;
    for (std::size_t i = imax + 1; i < sweep.size(); ++i) {
        if (sweep[i].heel < 180.0 && sweep[i].gz <= -kZeroGz) {
            Point a = sweep[i - 1];
            Point b = sweep[i];
            double root = a.heel - a.gz * (b.heel - a.heel) / (b.gz - a.gz);
            // Illinois-modified regula falsi on the solved GZ, from the linear interpolation.
            double ga = a.gz;
            double gb = b.gz;
            int side = 0;
            for (int k = 0; k < 40; ++k) {
                const Point m = solve(root, &a.eq);
                converged = converged && m.eq.converged;
                if (std::abs(m.gz) < 1e-7 || b.heel - a.heel < 1e-6) break;
                if (m.gz > 0.0) {
                    a = m;
                    ga = m.gz;
                    if (side == 1) gb *= 0.5;
                    side = 1;
                } else {
                    b = m;
                    gb = m.gz;
                    if (side == -1) ga *= 0.5;
                    side = -1;
                }
                root = a.heel - ga * (b.heel - a.heel) / (gb - ga);
            }
            avs = root;
            break;
        }
    }

    Json points = Json::array();
    for (const Point& p : sweep) {
        Json j;
        j["heel_deg"] = p.heel;
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
    out["gz_max_heel_deg"] = sweep[imax].heel;
    out["avs_deg"] = avs ? Json(*avs) : Json(nullptr);
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
