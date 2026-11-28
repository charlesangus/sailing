#include "sail/mass_model.hpp"

#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace sail {

namespace {

// Thin uniform shell over the six faces of an a x b x c box. Each face contributes its own
// plate inertia plus the parallel-axis term for its offset from the centre.
Mat3 shell_box_inertia(double m, const Vec3& d) {
    const double a = d.x();
    const double b = d.y();
    const double c = d.z();
    const double ax = b * c;
    const double ay = a * c;
    const double az = a * b;
    const double area = 2.0 * (ax + ay + az);
    const double sigma = m / area;
    const double ixx = ax * (b * b + c * c) / 12.0 + ay * (c * c / 12.0 + b * b / 4.0) +
                       az * (b * b / 12.0 + c * c / 4.0);
    const double iyy = ay * (a * a + c * c) / 12.0 + ax * (c * c / 12.0 + a * a / 4.0) +
                       az * (a * a / 12.0 + c * c / 4.0);
    const double izz = az * (a * a + b * b) / 12.0 + ax * (b * b / 12.0 + a * a / 4.0) +
                       ay * (a * a / 12.0 + b * b / 4.0);
    return (2.0 * sigma * Vec3(ixx, iyy, izz)).asDiagonal();
}

}  // namespace

Mat3 item_inertia(const MassItem& item) {
    const Vec3& d = item.dims;
    switch (item.shape) {
        case MassShape::point:
            return Mat3::Zero();
        case MassShape::box:
            return (item.mass / 12.0 *
                    Vec3(d.y() * d.y() + d.z() * d.z(), d.x() * d.x() + d.z() * d.z(),
                         d.x() * d.x() + d.y() * d.y()))
                .asDiagonal();
        case MassShape::shell_box:
            return shell_box_inertia(item.mass, d);
    }
    return Mat3::Zero();
}

MassProperties combine(std::span<const MassItem> items) {
    MassProperties out;
    Vec3 moment = Vec3::Zero();
    for (const MassItem& item : items) {
        out.mass += item.mass;
        moment += item.mass * item.cg;
    }
    if (!(out.mass > 0.0)) throw std::invalid_argument("combine: total mass is not positive");
    out.cg = moment / out.mass;
    for (const MassItem& item : items) {
        const Vec3 r = item.cg - out.cg;
        out.inertia += item_inertia(item) +
                       item.mass * (r.squaredNorm() * Mat3::Identity() - r * r.transpose());
    }
    return out;
}

Mat6 strip_added_mass(const HullDims& hull, const AddedMassCoefficients& coeffs, double density,
                      double rigid_mass) {
    const double factor = coeffs.length_factor.value_or(hull.cp);
    const double length = factor * hull.lwl;
    if (!(length > 0.0 && hull.bwl > 0.0 && hull.tc > 0.0 && density > 0.0)) {
        throw std::invalid_argument("strip_added_mass: hull dimensions must be positive");
    }
    const double pi = std::numbers::pi;
    const double b2 = 0.25 * hull.bwl * hull.bwl;
    const double t2 = hull.tc * hull.tc;
    const double sway = density * pi * t2 / 2.0 * length;
    const double heave = density * pi * b2 / 2.0 * length;
    const double roll = density * pi * (b2 - t2) * (b2 - t2) / 16.0 * length;
    const double gyradius2 = length * length / 12.0;

    Vec6 diag;
    diag << coeffs.surge_fraction * rigid_mass, coeffs.sway_scale * sway,
        coeffs.heave_scale * heave, coeffs.roll_scale * roll,
        coeffs.pitch_scale * heave * gyradius2, coeffs.yaw_scale * sway * gyradius2;
    return diag.asDiagonal();
}

MassModel::MassModel(BoatData data, MassConfiguration configuration)
    : data_(std::move(data)), configuration_(configuration) {
    crew_.name = "crew";
    crew_.in_displacement = false;
    set_configuration(configuration);
}

MassModel MassModel::load(const std::filesystem::path& boat_dir, MassConfiguration configuration) {
    return MassModel(load_boat_data(boat_dir), configuration);
}

void MassModel::set_configuration(MassConfiguration configuration) {
    configuration_ = configuration;
    const CrewData& crew = data_.crew;
    crew_.cg = Vec3(crew.x, 0.0, data_.sheer_z(crew.x) + crew.height_above_sheer);
    switch (configuration) {
        case MassConfiguration::measurement_trim:
            crew_.mass = 0.0;
            break;
        case MassConfiguration::sailing:
            crew_.mass = crew.sailing_mass;
            break;
        case MassConfiguration::orc_stability:
            crew_.mass = crew.orc_stability_mass;
            crew_.cg.z() = crew.orc_stability_z;
            break;
    }
    recompute();
}

void MassModel::set_crew_position(double y, double z) {
    crew_.cg.y() = y;
    crew_.cg.z() = z;
    recompute();
}

void MassModel::set_crew_mass(double mass) {
    if (!(mass >= 0.0)) throw std::invalid_argument("MassModel: crew mass must be non-negative");
    crew_.mass = mass;
    recompute();
}

void MassModel::recompute() {
    const bool carried = configuration_ == MassConfiguration::sailing;
    std::vector<MassItem> items;
    items.reserve(data_.items.size() + 1);
    for (const MassItem& item : data_.items) {
        if (item.in_displacement || carried) items.push_back(item);
    }
    if (crew_.mass > 0.0) items.push_back(crew_);
    total_ = combine(items);
}

Mat6 MassModel::added_mass() const {
    return strip_added_mass(data_.hull, data_.added_mass, data_.density, total_.mass);
}

BodyProperties MassModel::body_properties() const {
    BodyProperties body;
    body.mass = total_.mass;
    body.inertia = total_.inertia;
    body.added_mass = added_mass();
    return body;
}

}  // namespace sail
