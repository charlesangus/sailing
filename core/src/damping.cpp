#include "sail/damping.hpp"

#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

#include "dossier_json.hpp"
#include "sail/gravity.hpp"

namespace sail {

namespace {

using dossier::Node;

double positive(const Node& node, double v) {
    if (!(v > 0.0)) node.fail("expected a positive number");
    return v;
}

DampingRatio parse_ratio(const Node& dof) {
    DampingRatio ratio;
    const Node linear = dof.at("zeta_linear");
    const Node quadratic = dof.at("zeta_quadratic");
    const Node reference = dof.at("reference_velocity");
    ratio.zeta_linear = linear.value();
    ratio.zeta_quadratic = quadratic.value();
    ratio.reference_velocity = reference.value();
    if (ratio.zeta_linear < 0.0) linear.at("value").fail("expected a non-negative number");
    if (ratio.zeta_quadratic < 0.0) quadratic.at("value").fail("expected a non-negative number");
    positive(reference.at("value"), ratio.reference_velocity);
    return ratio;
}

DampingCoefficient derive_one(const DampingRatio& ratio, double stiffness, double effective_mass) {
    const double b_crit = critical_damping(stiffness, effective_mass);
    return {ratio.zeta_linear * b_crit, ratio.zeta_quadratic * b_crit / ratio.reference_velocity};
}

double damping_force(const DampingCoefficient& c, double multiplier, double velocity) {
    return -multiplier * (c.linear + c.quadratic * std::abs(velocity)) * velocity;
}

}  // namespace

const DampingCoefficient& DampingCoefficients::of(DampingDof dof) const {
    switch (dof) {
        case DampingDof::heave: return heave;
        case DampingDof::roll: return roll;
        case DampingDof::pitch: return pitch;
    }
    throw std::invalid_argument("unknown DampingDof");
}

DampingCoefficient& DampingCoefficients::of(DampingDof dof) {
    return const_cast<DampingCoefficient&>(std::as_const(*this).of(dof));
}

DampingSpec load_damping_spec(const std::filesystem::path& boat_dir) {
    const dossier::Document doc = dossier::read_document(boat_dir / "damping.json");
    const Node root = doc.root();
    DampingSpec spec;
    spec.heave = parse_ratio(root.at("heave"));
    spec.roll = parse_ratio(root.at("roll"));
    spec.pitch = parse_ratio(root.at("pitch"));
    return spec;
}

DampingStiffness dossier_stiffness(const BoatData& data) {
    // kg/mm to kg/m.
    const double waterplane_area = data.sink * 1000.0 / data.density;
    const double rho_g = data.density * kGravity;
    const double cw = data.hull.cw;
    const double inertia_l = cw * cw * data.hull.bwl * data.hull.lwl * data.hull.lwl *
                             data.hull.lwl / 12.0;

    DampingStiffness k;
    k.heave = rho_g * waterplane_area;
    k.roll = data.righting_moment_1deg / std::sin(std::numbers::pi / 180.0);
    k.pitch = rho_g * inertia_l;
    return k;
}

double critical_damping(double stiffness, double effective_mass) {
    if (!(stiffness > 0.0) || !(effective_mass > 0.0)) {
        throw std::invalid_argument("critical_damping: stiffness and mass must be positive");
    }
    return 2.0 * std::sqrt(stiffness * effective_mass);
}

DampingCoefficients derive_damping(const DampingSpec& spec, const DampingStiffness& stiffness,
                                   const BodyProperties& body) {
    const Mat6 m = body.generalised_mass();
    DampingCoefficients c;
    c.heave = derive_one(spec.heave, stiffness.heave, m(2, 2));
    c.roll = derive_one(spec.roll, stiffness.roll, m(3, 3));
    c.pitch = derive_one(spec.pitch, stiffness.pitch, m(4, 4));
    return c;
}

DampingCoefficients load_damping(const MassModel& model) {
    const BoatData& data = model.data();
    return derive_damping(load_damping_spec(data.directory), dossier_stiffness(data),
                          model.body_properties());
}

DampingModule::DampingModule(const DampingCoefficients& coefficients)
    : coefficients_(coefficients) {}

void DampingModule::accumulate(const State& state, double, Wrench& wrench) {
    wrench.force.z() +=
        damping_force(coefficients_.heave, multipliers_[0], state.linear_velocity.z());
    wrench.moment.x() +=
        damping_force(coefficients_.roll, multipliers_[1], state.angular_velocity.x());
    wrench.moment.y() +=
        damping_force(coefficients_.pitch, multipliers_[2], state.angular_velocity.y());
}

void DampingModule::set_multiplier(DampingDof dof, double multiplier) {
    if (!std::isfinite(multiplier) || multiplier < 0.0) {
        throw std::invalid_argument("damping multiplier must be finite and non-negative");
    }
    multipliers_[static_cast<std::size_t>(dof)] = multiplier;
}

void DampingModule::set_multipliers(double multiplier) {
    set_multiplier(DampingDof::heave, multiplier);
    set_multiplier(DampingDof::roll, multiplier);
    set_multiplier(DampingDof::pitch, multiplier);
}

double DampingModule::multiplier(DampingDof dof) const {
    return multipliers_[static_cast<std::size_t>(dof)];
}

}  // namespace sail
