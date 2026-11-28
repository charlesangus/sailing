#include "sail/hull_forces.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

#include "dossier_json.hpp"

namespace sail {

namespace {

using dossier::Document;
using dossier::Node;
using dossier::read_document;

double non_negative(const Node& entry) {
    const double v = entry.value();
    if (v < 0.0) entry.at("value").fail("expected a non-negative number");
    return v;
}

double positive_number(const Node& node) {
    const double v = node.number();
    if (!(v > 0.0)) node.fail("expected a positive number");
    return v;
}

double smoothstep(double x) {
    const double s = std::clamp(x, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

double damping_force(const DampingCoefficient& c, double velocity) {
    return -(c.linear + c.quadratic * std::abs(velocity)) * velocity;
}

}  // namespace

HullForceParams load_hull_force_params(const std::filesystem::path& boat_dir,
                                       const Vec3& reference_point, const BodyProperties& body) {
    const Document forces_doc = read_document(boat_dir / "hull_forces.json");
    const Document hull_doc = read_document(boat_dir / "hull_hydro.json");
    const Document res_doc = read_document(boat_dir / "hull_resistance.json");

    const Node result = hull_doc.root().at("conditions").at("sailing").at("result");
    const double lwl = positive_number(result.at("lwl"));
    const double tc = positive_number(result.at("tc"));
    const double lcb_x = result.at("lcb_x_canoe").number();
    const double flotation_z = result.at("sinkage_m").number();

    HullForceParams p;
    p.density = res_doc.root().at("water").at("density").positive_value();
    p.reference_point = reference_point;

    const Node root = forces_doc.root();
    const double depth_fraction =
        non_negative(root.at("centre_of_lateral_resistance").at("depth_fraction_of_tc"));
    p.clr_point = Vec3(lcb_x, 0.0, flotation_z - depth_fraction * tc);

    const Node side = root.at("side_force");
    p.side_force_coefficient =
        non_negative(side.at("slope_multiplier")) * 0.5 * p.density * std::numbers::pi * tc * tc;
    p.induced_drag_factor = non_negative(side.at("induced_drag_factor"));
    p.blend_speed = side.at("blend_speed").positive_value();

    // Lateral profile with draft Tc (1 - (2x/L)^2): area 2 L Tc / 3 and integral of |x|^3 times
    // draft Tc L^4 / 96.
    const Node sway = root.at("sway");
    const double sway_quadratic = 0.5 * p.density *
                                  non_negative(sway.at("crossflow_drag_coefficient")) *
                                  (2.0 / 3.0) * lwl * tc;
    p.sway = {sway_quadratic * non_negative(sway.at("linear_crossover_speed")), sway_quadratic};

    const Node yaw = root.at("yaw");
    const double l2 = lwl * lwl;
    const double yaw_quadratic =
        0.5 * p.density * non_negative(yaw.at("crossflow_drag_coefficient")) * tc * l2 * l2 / 96.0;
    p.yaw = {yaw_quadratic * non_negative(yaw.at("linear_crossover_speed")) / (0.5 * lwl),
             yaw_quadratic};

    p.munk_correction_fraction = non_negative(root.at("munk_correction").at("fraction"));
    p.added_mass_surge = body.added_mass(0, 0);
    p.added_mass_sway = body.added_mass(1, 1);
    return p;
}

HullForceParams load_hull_force_params(const MassModel& model) {
    return load_hull_force_params(model.data().directory, model.cg(), model.body_properties());
}

HullForceModule::HullForceModule(std::shared_ptr<const HullResistanceModel> resistance,
                                 HullForceParams params,
                                 std::shared_ptr<const WaterCurrent> current)
    : resistance_(std::move(resistance)),
      params_(std::move(params)),
      current_(current ? std::move(current) : std::make_shared<StillWater>()) {
    if (!resistance_) throw std::invalid_argument("HullForceModule: null resistance model");
}

void HullForceModule::accumulate(const State& state, double t, Wrench& wrench) {
    HullForceResult& r = last_;
    const Mat3 rot = state.rotation();
    const Vec3 arm = params_.clr_point - params_.reference_point;

    // Only the yaw rate moves the CLR here: the roll- and pitch-rate parts of its velocity are
    // damped by DampingModule, and counting them again would double that damping.
    const Vec3 yaw_rate(0.0, 0.0, state.angular_velocity.z());
    const Vec3 clr_world = state.position + rot * arm;
    const Vec3 relative_body = state.linear_velocity + yaw_rate.cross(arm) -
                               rot.transpose() * current_->velocity(clr_world, t);
    const Vec3 relative_world = rot * relative_body;

    const Vec3 body_x = rot.col(0);
    const double heading = std::atan2(body_x.y(), body_x.x());
    const double ch = std::cos(heading);
    const double sh = std::sin(heading);
    const Vec3 x_h(ch, sh, 0.0);
    const Vec3 y_h(-sh, ch, 0.0);

    r.u_ax = x_h.dot(relative_world);
    r.u_lat = y_h.dot(relative_world);
    r.speed = std::hypot(r.u_ax, r.u_lat);
    r.leeway = std::atan2(r.u_lat, r.u_ax);
    r.heel = std::atan2(rot(2, 1), rot(2, 2));

    const double cb = std::cos(r.leeway);
    const double sb = std::sin(r.leeway);
    const Vec3 along_flow = cb * x_h + sb * y_h;
    const Vec3 across_flow = -sb * x_h + cb * y_h;

    r.resistance = resistance_->resistance(std::abs(r.u_ax), r.heel);
    r.resistance_world = -r.resistance.total_N * along_flow;

    // u_ax * u_lat changes sign with u_ax just as across_flow's lateral part does, so the lateral
    // component -k u_lat |u_ax| |cos(leeway)| opposes the drift both ahead and astern.
    const double blend = smoothstep(r.speed / params_.blend_speed);
    const double lift = blend * params_.side_force_coefficient * r.u_ax * r.u_lat;
    r.side_force_world = -lift * across_flow;
    r.induced_drag_world =
        -params_.induced_drag_factor * std::abs(lift) * std::abs(sb) * along_flow;

    const Vec3 hydro_body =
        rot.transpose() * (r.resistance_world + r.side_force_world + r.induced_drag_world);

    r.damping_force = Vec3(0.0, damping_force(params_.sway, relative_body.y()), 0.0);
    r.damping_moment = Vec3(0.0, 0.0, damping_force(params_.yaw, state.angular_velocity.z()));

    r.munk_correction = blend * params_.munk_correction_fraction *
                        (params_.added_mass_sway - params_.added_mass_surge) *
                        relative_body.x() * relative_body.y();

    const Vec3 force = hydro_body + r.damping_force;
    r.wrench.force = force;
    r.wrench.moment = arm.cross(force) + r.damping_moment + Vec3(0.0, 0.0, r.munk_correction);
    wrench += r.wrench;
}

}  // namespace sail
