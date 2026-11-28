#include "sail/crew.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

#include "dossier_json.hpp"

namespace sail {

namespace {

using dossier::Document;
using dossier::Node;
using dossier::read_document;

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kKnot = 0.514444;

double non_negative(const Node& entry) {
    const double v = entry.value();
    if (v < 0.0) entry.at("value").fail("expected a non-negative number");
    return v;
}

// 0 at or below `edge - ramp / 2`, 1 at or above `edge + ramp / 2`.
double ramp_up(double x, double edge, double ramp) {
    if (ramp <= 0.0) return x >= edge ? 1.0 : 0.0;
    return std::clamp((x - (edge - 0.5 * ramp)) / ramp, 0.0, 1.0);
}

}  // namespace

CrewParams load_crew_params(const std::filesystem::path& boat_dir, const MassModel& model) {
    const Document doc = read_document(boat_dir / "crew.json");
    const Node root = doc.root();
    const BoatData& data = model.data();

    CrewParams p;
    p.centre_z = data.sheer_z(data.crew.x) + data.crew.height_above_sheer;
    const double inset = non_negative(root.at("rail_inset"));
    p.rail_y = root.at("deck_half_width_at_crew").positive_value() - inset;
    if (!(p.rail_y > 0.0)) root.at("rail_inset").at("value").fail("rail inset reaches the centreline");
    p.hiking_outboard = non_negative(root.at("hiking_outboard"));
    p.hiking_drop = non_negative(root.at("hiking_drop"));
    p.heel_threshold = non_negative(root.at("heel_threshold")) * kDeg;
    p.heel_full = root.at("heel_full").positive_value() * kDeg;
    if (!(p.heel_full > p.heel_threshold)) {
        root.at("heel_full").at("value").fail("expected more than heel_threshold");
    }
    p.light_air_aws = root.at("light_air_aws").positive_value() * kKnot;
    p.light_air_ramp = non_negative(root.at("light_air_ramp")) * kKnot;
    p.downwind_awa = root.at("downwind_awa").positive_value() * kDeg;
    p.downwind_ramp = non_negative(root.at("downwind_ramp")) * kDeg;
    p.transit_time = root.at("transit_time").positive_value();
    p.hiking_posture_fraction = root.at("hiking_posture_fraction").positive_value();
    p.update_min_move = non_negative(root.at("update_min_move"));
    return p;
}

CrewController::CrewController(const CrewParams& params) : params_(params) {
    if (!(params_.transit_time > 0.0) || !(params_.hiking_y() > 0.0) ||
        !(params_.heel_full > params_.heel_threshold)) {
        throw std::invalid_argument("CrewController: invalid parameters");
    }
    reset();
}

void CrewController::reset() { set_y(0.0); }

void CrewController::reset(double y) {
    const double limit = params_.hiking_y();
    set_y(std::isfinite(y) ? std::clamp(y, -limit, limit) : 0.0);
}

void CrewController::set_y(double y) {
    const double fraction = std::abs(y) / params_.hiking_y();
    position_.y = y;
    position_.z = params_.centre_z - fraction * params_.hiking_drop;
    position_.posture = fraction >= params_.hiking_posture_fraction ? CrewPosture::hiking
                                                                    : CrewPosture::sitting;
}

double CrewController::target_y(double heel, const ApparentWind& wind) const {
    const double heel_fraction = std::clamp(
        (std::abs(heel) - params_.heel_threshold) / (params_.heel_full - params_.heel_threshold),
        0.0, 1.0);
    const double light_air = ramp_up(wind.aws, params_.light_air_aws, params_.light_air_ramp);
    const double upwind = 1.0 - ramp_up(std::abs(wind.awa), params_.downwind_awa, params_.downwind_ramp);
    const double side = wind.tack == Tack::Starboard ? -1.0 : 1.0;
    return side * heel_fraction * light_air * upwind * params_.hiking_y();
}

const CrewPosition& CrewController::step(double dt, double heel, const ApparentWind& wind) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return position_;
    const double reach = params_.max_speed() * dt;
    const double target = target_y(heel, wind);
    set_y(position_.y + std::clamp(target - position_.y, -reach, reach));
    return position_;
}

}  // namespace sail
