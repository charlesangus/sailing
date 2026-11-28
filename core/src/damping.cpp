#include "sail/damping.hpp"

#include <cmath>
#include <fstream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace sail {

namespace {

using Json = nlohmann::ordered_json;

constexpr double kGravity = 9.81;

class Reader {
public:
    Reader(const std::filesystem::path& file, const Json& json, std::string where)
        : file_(&file), json_(&json), where_(std::move(where)) {}

    [[noreturn]] void fail(const std::string& message) const {
        throw BoatDataError(*file_,
                            (where_.empty() ? std::string("top level") : where_) + ": " + message);
    }

    Reader at(const std::string& key) const {
        if (!json_->is_object()) fail("expected an object");
        const auto it = json_->find(key);
        if (it == json_->end()) fail("missing field '" + key + "'");
        return Reader(*file_, *it, where_.empty() ? key : where_ + "." + key);
    }

    double number() const {
        if (!json_->is_number()) fail("expected a number");
        const double v = json_->get<double>();
        if (!std::isfinite(v)) fail("expected a finite number");
        return v;
    }

    double value() const { return at("value").number(); }

private:
    const std::filesystem::path* file_;
    const Json* json_;
    std::string where_;
};

Json read_json(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw BoatDataError(file, "cannot open file");
    try {
        Json json = Json::parse(in);
        if (!json.is_object()) throw BoatDataError(file, "top level: expected an object");
        return json;
    } catch (const Json::exception& e) {
        throw BoatDataError(file, std::string("malformed JSON: ") + e.what());
    }
}

double positive(const Reader& node, double v) {
    if (!(v > 0.0)) node.fail("expected a positive number");
    return v;
}

DampingRatio parse_ratio(const Reader& dof) {
    DampingRatio ratio;
    const Reader linear = dof.at("zeta_linear");
    const Reader quadratic = dof.at("zeta_quadratic");
    const Reader reference = dof.at("reference_velocity");
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
    const std::filesystem::path file = boat_dir / "damping.json";
    const Json json = read_json(file);
    const Reader root(file, json, "");
    DampingSpec spec;
    spec.heave = parse_ratio(root.at("heave"));
    spec.roll = parse_ratio(root.at("roll"));
    spec.pitch = parse_ratio(root.at("pitch"));
    return spec;
}

DampingStiffness dossier_stiffness(const BoatData& data) {
    const std::filesystem::path part_file = data.directory / "particulars.json";
    const std::filesystem::path hydro_file = data.directory / "hull_hydro.json";
    const Json part_json = read_json(part_file);
    const Json hydro_json = read_json(hydro_file);
    const Reader part(part_file, part_json, "");
    const Reader hydro(hydro_file, hydro_json, "");

    const Reader sink = part.at("sink");
    const Reader rm = part.at("righting_moment_1deg");
    const Reader cw = hydro.at("result").at("cw");
    // kg/mm to kg/m.
    const double waterplane_area = positive(sink.at("value"), sink.value()) * 1000.0 / data.density;
    const double rm_1deg = positive(rm.at("value"), rm.value());
    const double waterplane_cw = positive(cw, cw.number());

    const double rho_g = data.density * kGravity;
    const double inertia_l = waterplane_cw * waterplane_cw * data.hull.bwl *
                             data.hull.lwl * data.hull.lwl * data.hull.lwl / 12.0;

    DampingStiffness k;
    k.heave = rho_g * waterplane_area;
    k.roll = rm_1deg / std::sin(std::numbers::pi / 180.0);
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
