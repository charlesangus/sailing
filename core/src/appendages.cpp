#include "sail/appendages.hpp"

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

constexpr double kHalfPi = 0.5 * std::numbers::pi;
constexpr double kDeg = std::numbers::pi / 180.0;

double smoothstep(double x) {
    const double s = std::clamp(x, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

// alpha as the keel's circulation sees it: alpha up to the stall angle, held there and fading to
// 0 at 90 degrees, and 0 in reverse flow.
double downwash_alpha(double alpha, double stall) {
    const double a = std::abs(alpha);
    if (a >= kHalfPi) return 0.0;
    const double held = a <= stall ? a : stall * (1.0 - smoothstep((a - stall) / (kHalfPi - stall)));
    return std::copysign(held, alpha);
}

const RudderParams& checked(const RudderParams& p) {
    if (!(p.max_angle > 0.0) || !(p.rate > 0.0) || !std::isfinite(p.max_angle) ||
        !std::isfinite(p.rate)) {
        throw std::invalid_argument("RudderForceModule: max angle and rate must be positive");
    }
    if (!(p.downwash_factor >= 0.0) || !std::isfinite(p.downwash_factor)) {
        throw std::invalid_argument("RudderForceModule: downwash factor must be non-negative");
    }
    if (!(p.stock_axis.norm() > 1e-9)) {
        throw std::invalid_argument("RudderForceModule: zero stock axis");
    }
    if (!(p.foil.density > 0.0)) {
        throw std::invalid_argument("RudderForceModule: density must be positive");
    }
    return p;
}

}  // namespace

RudderParams load_rudder_params(const std::filesystem::path& boat_dir,
                                const Vec3& reference_point) {
    RudderParams p;
    p.foil = load_foil_force_params(boat_dir, "rudder", reference_point);

    const Document app_doc = read_document(boat_dir / "appendages.json");
    const Node rudder = app_doc.root().at("foils").at("rudder");
    p.stock_position = rudder.at("stock_position").at("value").vec3();
    const Node axis = rudder.at("stock_axis_direction");
    const Vec3 a = axis.at("value").vec3();
    if (!(a.norm() > 1e-9)) axis.at("value").fail("expected a non-zero direction");
    p.stock_axis = a.normalized();

    const Document ctl_doc = read_document(boat_dir / "controls.json");
    const Node ctl = ctl_doc.root().at("rudder");
    p.max_angle = ctl.at("max_angle").positive_value() * kDeg;
    p.rate = ctl.at("max_rate").positive_value() * kDeg;
    const Node k = ctl.at("keel_downwash_factor");
    p.downwash_factor = k.value();
    if (!(p.downwash_factor >= 0.0)) k.at("value").fail("expected a non-negative number");
    return p;
}

RudderForceModule::RudderForceModule(std::shared_ptr<const FoilSection> section,
                                     RudderParams params, const Foil& keel,
                                     std::shared_ptr<const WaterCurrent> current)
    : foil_(std::move(section), checked(params).foil.foil),
      params_(std::move(params)),
      keel_(keel.params().geometry),
      keel_stall_(keel.section().stall_angle()),
      current_(current ? std::move(current) : std::make_shared<StillWater>()),
      actuator_(params_.max_angle, params_.rate) {
    params_.stock_axis.normalize();
    pose(0.0);
}

void RudderForceModule::set_downwash_factor(double k) {
    if (!(k >= 0.0) || !std::isfinite(k)) {
        throw std::invalid_argument("RudderForceModule: downwash factor must be non-negative");
    }
    params_.downwash_factor = k;
}

void RudderForceModule::pose(double angle) {
    const FoilGeometry& neutral = params_.foil.foil.geometry;
    const Mat3 turn = Eigen::AngleAxisd(angle, params_.stock_axis).toRotationMatrix();
    foil_.set_pose(params_.stock_position +
                       turn * (neutral.quarter_chord_centre - params_.stock_position),
                   turn * neutral.span_direction, turn * neutral.chord_direction);
    posed_angle_ = angle;
}

void RudderForceModule::accumulate(const State& state, double t, Wrench& wrench) {
    if (has_time_ && t > last_time_) actuator_.step(t - last_time_);
    has_time_ = true;
    last_time_ = t;
    if (actuator_.angle() != posed_angle_) pose(actuator_.angle());

    const Mat3 rot = state.rotation();
    const Vec3& ref = params_.foil.reference_point;

    const Vec3 keel_arm = keel_.quarter_chord_centre - ref;
    const Vec3 keel_inflow =
        rot.transpose() * current_->velocity(state.position + rot * keel_arm, t) -
        (state.linear_velocity + state.angular_velocity.cross(keel_arm));
    const Vec3 keel_normal = keel_.chord_direction.cross(keel_.span_direction);
    keel_alpha_ = std::atan2(keel_inflow.dot(keel_normal), keel_inflow.dot(keel_.chord_direction));
    downwash_ = params_.downwash_factor * downwash_alpha(keel_alpha_, keel_stall_);

    const FoilGeometry& g = foil_.params().geometry;
    const Vec3 arm = g.quarter_chord_centre - ref;
    const Vec3 inflow = rot.transpose() * current_->velocity(state.position + rot * arm, t) -
                        (state.linear_velocity + state.angular_velocity.cross(arm));
    const Vec3 downwashed = Eigen::AngleAxisd(downwash_, g.span_direction) * inflow;

    last_ = foil_.evaluate(downwashed, params_.foil.density);
    last_wrench_.force = last_.force;
    last_wrench_.moment = (last_.centre_of_pressure - ref).cross(last_.force);
    hinge_moment_ =
        (last_.centre_of_pressure - params_.stock_position).cross(last_.force).dot(params_.stock_axis);
    wrench += last_wrench_;
}

Appendages::Appendages(std::shared_ptr<const FoilSection> keel_section,
                       std::shared_ptr<const FoilSection> rudder_section,
                       const std::filesystem::path& boat_dir, const Vec3& reference_point,
                       std::shared_ptr<const WaterCurrent> current)
    : keel_(std::move(keel_section), load_foil_force_params(boat_dir, "keel", reference_point),
            current),
      rudder_(std::move(rudder_section), load_rudder_params(boat_dir, reference_point),
              keel_.foil(), std::move(current)) {}

void Appendages::set_reference_point(const Vec3& point) {
    keel_.set_reference_point(point);
    rudder_.set_reference_point(point);
}

void Appendages::add_to(Simulation& sim) {
    sim.add_module(keel_);
    sim.add_module(rudder_);
}

}  // namespace sail
