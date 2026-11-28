#include <cstdio>
#include <limits>

#include <nlohmann/json.hpp>

#include "common.hpp"

namespace sailcli {

namespace {

constexpr const char* kUsage =
    "usage: sailsim hydro --boat DIR [--config measurement|sailing|orc]\n"
    "\n"
    "Finds the boat's upright flat-water equilibrium (draft and free trim, no heel) and prints it\n"
    "as JSON. Positions are in the hull frame: x forward, y to port, z up, origin on the\n"
    "centreline at mid-LWL on the measurement-trim flotation plane (DWL).\n"
    "\n"
    "  --boat DIR     boat dossier directory, e.g. data/boats/j80\n"
    "  --config NAME  measurement (default): certificate measurement trim, no crew\n"
    "                 sailing: all items plus the class-maximum crew\n"
    "                 orc: measurement items plus ORC's stability crew and gear\n"
    "\n"
    "trim_deg is positive bow down. dwl_origin_heave_m is the world height of the hull-frame\n"
    "origin, negative when the boat floats deeper than its DWL. draft_m is the depth of the\n"
    "deepest point (the keel tip).\n";

using Json = nlohmann::ordered_json;

Json vec(const sail::Vec3& v) { return Json::array({v.x(), v.y(), v.z()}); }

int hydro(const Options& options) {
    const ConfigChoice config = parse_config(options.get("--config"));
    const LoadedBoat boat = load_boat(options, config.configuration);
    const double mass = boat.model.total_mass();

    const sail::Equilibrium eq = sail::solve_equilibrium(boat.hydro.mesh, boat.params, mass);
    const sail::State& pose = eq.pose;
    const sail::Mat3 rot = pose.rotation();
    const sail::Vec3& ref = boat.params.reference_point;
    const auto to_world = [&](const sail::Vec3& p) { return sail::Vec3(pose.position + rot * (p - ref)); };

    double deepest = std::numeric_limits<double>::infinity();
    sail::Vec3 deepest_point = sail::Vec3::Zero();
    for (const sail::Vec3& v : boat.hydro.mesh.vertices) {
        const double z = to_world(v).z();
        if (z < deepest) {
            deepest = z;
            deepest_point = v;
        }
    }
    const sail::HydrostaticResult& h = eq.hydrostatics;
    const sail::Vec3 cb = rot.transpose() * (h.centre_of_buoyancy - pose.position) + ref;

    Json out;
    out["config"] = config.name;
    out["converged"] = eq.converged;
    out["total_mass_kg"] = mass;
    out["draft_m"] = -deepest;
    out["deepest_point_m"] = vec(deepest_point);
    out["trim_deg"] = deg(eq.trim);
    out["dwl_origin_heave_m"] = to_world(sail::Vec3::Zero()).z();
    out["displaced_volume_m3"] = h.volume;
    out["displaced_mass_kg"] = h.volume * boat.params.density;
    out["lcb_m"] = cb.x();
    out["vcb_m"] = cb.z();
    out["centre_of_buoyancy_m"] = vec(cb);
    out["cg_m"] = vec(ref);
    out["waterplane_area_m2"] = h.waterplane_area;
    out["density_kg_m3"] = boat.params.density;
    out["frame"] = "hull: x forward, y port, z up, origin mid-LWL on the DWL";
    out["trim_sign"] = "positive bow down";
    std::puts(out.dump(2).c_str());
    return eq.converged ? 0 : 1;
}

}  // namespace

int run_hydro(const Args& args) { return guarded("hydro", kUsage, hydro, args, {"--boat", "--config"}); }

}  // namespace sailcli
