#include "sail/foil.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include "dossier_json.hpp"

namespace sail {

namespace {

using dossier::Document;
using dossier::Node;
using dossier::read_document;

constexpr double kPi = std::numbers::pi;
constexpr double kHalfPi = 0.5 * std::numbers::pi;
constexpr double kDeg = std::numbers::pi / 180.0;

double smoothstep(double x) {
    const double s = std::clamp(x, 0.0, 1.0);
    return s * s * (3.0 - 2.0 * s);
}

bool parse_field(const std::string& text, double& out) {
    std::size_t begin = text.find_first_not_of(" \t\r");
    std::size_t end = text.find_last_not_of(" \t\r");
    if (begin == std::string::npos) return false;
    const char* first = text.data() + begin;
    const char* last = text.data() + end + 1;
    const auto [ptr, ec] = std::from_chars(first, last, out);
    return ec == std::errc() && ptr == last && std::isfinite(out);
}

Vec3 unit_or_throw(const Vec3& v, const char* what) {
    const double n = v.norm();
    if (!(n > 1e-9)) throw std::invalid_argument(std::string("Foil: zero ") + what);
    return v / n;
}

}  // namespace

FoilSection::FoilSection(std::vector<double> alpha_deg, std::vector<double> cl,
                         std::vector<double> cd)
    : alpha_(std::move(alpha_deg)), cl_(std::move(cl)), cd_(std::move(cd)) {
    const std::size_t n = alpha_.size();
    if (n < 3 || cl_.size() != n || cd_.size() != n) {
        throw std::invalid_argument("FoilSection: need 3 or more rows of alpha, cl and cd");
    }
    if (alpha_.front() != 0.0 || alpha_.back() != 180.0) {
        throw std::invalid_argument("FoilSection: alpha must run from 0 to 180 degrees");
    }
    for (std::size_t i = 1; i < n; ++i) {
        if (!(alpha_[i] > alpha_[i - 1])) {
            throw std::invalid_argument("FoilSection: alpha must be strictly increasing");
        }
    }
    if (cl_.front() != 0.0 || cl_.back() != 0.0) {
        throw std::invalid_argument("FoilSection: cl must be zero at 0 and 180 degrees");
    }
    for (double& a : alpha_) a *= kDeg;

    std::size_t stall = 0;
    for (std::size_t i = 1; i + 1 < n && alpha_[i] < kHalfPi; ++i) {
        if (cl_[i] > 0.0 && cl_[i] >= cl_[i - 1] && cl_[i] > cl_[i + 1]) {
            stall = i;
            break;
        }
    }
    if (stall == 0) throw std::invalid_argument("FoilSection: no Cl maximum below 90 degrees");
    stall_ = alpha_[stall];

    std::size_t reverse = 0;
    for (std::size_t j = n - 2; j > 0 && alpha_[j] > kHalfPi; --j) {
        if (std::abs(cl_[j]) >= std::abs(cl_[j + 1]) && std::abs(cl_[j]) > std::abs(cl_[j - 1])) {
            reverse = j;
            break;
        }
    }
    if (reverse == 0) {
        throw std::invalid_argument("FoilSection: no reverse-flow Cl extreme above 90 degrees");
    }
    reverse_stall_ = kPi - alpha_[reverse];

    double num = 0.0;
    double den = 0.0;
    for (std::size_t i = 1; i < n && alpha_[i] <= 0.5 * stall_ + 1e-12; ++i) {
        num += alpha_[i] * cl_[i];
        den += alpha_[i] * alpha_[i];
    }
    if (!(den > 0.0) || !(num > 0.0)) {
        throw std::invalid_argument("FoilSection: no positive lift slope below half the stall");
    }
    lift_slope_ = num / den;
}

FoilSection FoilSection::load(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw BoatDataError(file, "cannot open file");
    std::vector<double> alpha;
    std::vector<double> cl;
    std::vector<double> cd;
    std::string text;
    std::size_t line = 0;
    while (std::getline(in, text)) {
        ++line;
        if (line == 1) {
            if (text.rfind("alpha_deg,cl,cd", 0) != 0) {
                throw BoatDataError(file, "line 1: expected the header alpha_deg,cl,cd");
            }
            continue;
        }
        if (text.find_first_not_of(" \t\r") == std::string::npos) continue;
        double v[3];
        std::size_t start = 0;
        for (int k = 0; k < 3; ++k) {
            const std::size_t comma = text.find(',', start);
            const bool last = k == 2;
            if (last != (comma == std::string::npos) ||
                !parse_field(text.substr(start, last ? std::string::npos : comma - start), v[k])) {
                throw BoatDataError(file, "line " + std::to_string(line) +
                                              ": expected three numbers alpha_deg,cl,cd");
            }
            start = comma + 1;
        }
        alpha.push_back(v[0]);
        cl.push_back(v[1]);
        cd.push_back(v[2]);
    }
    try {
        return FoilSection(std::move(alpha), std::move(cl), std::move(cd));
    } catch (const std::invalid_argument& e) {
        throw BoatDataError(file, e.what());
    }
}

double FoilSection::cl(double alpha) const {
    const double a = std::remainder(alpha, 2.0 * kPi);
    const double x = std::min(std::abs(a), kPi);
    const auto it = std::upper_bound(alpha_.begin() + 1, alpha_.end() - 1, x);
    const std::size_t i = static_cast<std::size_t>(it - alpha_.begin());
    const double w = (x - alpha_[i - 1]) / (alpha_[i] - alpha_[i - 1]);
    const double v = cl_[i - 1] + w * (cl_[i] - cl_[i - 1]);
    return a < 0.0 ? -v : v;
}

double FoilSection::cd(double alpha) const {
    const double x = std::min(std::abs(std::remainder(alpha, 2.0 * kPi)), kPi);
    const auto it = std::upper_bound(alpha_.begin() + 1, alpha_.end() - 1, x);
    const std::size_t i = static_cast<std::size_t>(it - alpha_.begin());
    const double w = (x - alpha_[i - 1]) / (alpha_[i] - alpha_[i - 1]);
    return cd_[i - 1] + w * (cd_[i] - cd_[i - 1]);
}

double FoilGeometry::half_chord_sweep() const {
    return std::atan(std::tan(quarter_chord_sweep) + (tip_chord - root_chord) / (4.0 * span));
}

double FoilGeometry::mean_aerodynamic_chord() const {
    const double l = tip_chord / root_chord;
    return 2.0 / 3.0 * root_chord * (1.0 + l + l * l) / (1.0 + l);
}

Foil::Foil(std::shared_ptr<const FoilSection> section, FoilParams params)
    : section_(std::move(section)), params_(std::move(params)) {
    if (!section_) throw std::invalid_argument("Foil: null section");
    const FoilGeometry& g = params_.geometry;
    if (!(g.span > 0.0) || !(g.root_chord > 0.0) || !(g.tip_chord > 0.0) ||
        !(g.planform_area > 0.0) || !(g.mirror_factor > 0.0)) {
        throw std::invalid_argument("Foil: span, chords, area and mirror factor must be positive");
    }
    if (!(params_.oswald_efficiency > 0.0) || !(params_.blend_speed > 0.0)) {
        throw std::invalid_argument("Foil: Oswald efficiency and blend speed must be positive");
    }
    if (!(std::abs(g.quarter_chord_sweep) < kHalfPi)) {
        throw std::invalid_argument("Foil: sweep must be within +-90 degrees");
    }
    set_pose(g.quarter_chord_centre, g.span_direction, g.chord_direction);

    const double ar = g.effective_aspect_ratio();
    const double t = std::tan(g.half_chord_sweep());
    lift_slope_ = 2.0 * kPi * ar / (2.0 + std::sqrt(ar * ar * (1.0 + t * t) + 4.0));
    attached_scale_ = lift_slope_ / section_->lift_slope();
    induced_factor_ = 1.0 / (kPi * ar * params_.oswald_efficiency);
    mac_ = g.mean_aerodynamic_chord();
}

void Foil::set_pose(const Vec3& quarter_chord_centre, const Vec3& span_direction,
                    const Vec3& chord_direction) {
    const Vec3 s = unit_or_throw(span_direction, "span direction");
    const Vec3 c = unit_or_throw(chord_direction - chord_direction.dot(s) * s,
                                 "chord direction normal to the span");
    params_.geometry.quarter_chord_centre = quarter_chord_centre;
    params_.geometry.span_direction = s;
    params_.geometry.chord_direction = c;
}

double Foil::separation(double alpha) const {
    const double a = std::min(std::abs(std::remainder(alpha, 2.0 * kPi)), kPi);
    const double fwd = section_->stall_angle();
    const double rev = section_->reverse_stall_angle();
    if (a <= kHalfPi) return smoothstep((a - fwd) / (kHalfPi - fwd));
    return smoothstep((kPi - rev - a) / (kHalfPi - rev));
}

double Foil::lift_coefficient(double alpha) const {
    const double s = separation(alpha);
    return section_->cl(alpha) * (attached_scale_ + (1.0 - attached_scale_) * s);
}

double Foil::induced_drag_coefficient(double alpha) const {
    const double cl = lift_coefficient(alpha);
    return (1.0 - separation(alpha)) * cl * cl * induced_factor_;
}

double Foil::drag_coefficient(double alpha) const {
    return section_->cd(alpha) + induced_drag_coefficient(alpha);
}

double Foil::cp_chord_fraction(double alpha) const {
    const double s = separation(alpha);
    const double a = std::abs(std::remainder(alpha, 2.0 * kPi));
    return a <= kHalfPi ? 0.25 + 0.25 * s : 0.75 - 0.25 * s;
}

Vec3 Foil::centre_of_pressure(double alpha) const {
    const FoilGeometry& g = params_.geometry;
    return g.quarter_chord_centre + (cp_chord_fraction(alpha) - 0.25) * mac_ * g.chord_direction;
}

FoilForces Foil::evaluate(const Vec3& inflow, double density) const {
    const FoilGeometry& g = params_.geometry;
    const Vec3& s = g.span_direction;
    const Vec3& c = g.chord_direction;
    const Vec3 n = c.cross(s);

    FoilForces r;
    r.spanwise_speed = inflow.dot(s);
    r.inflow = inflow - r.spanwise_speed * s;
    r.speed = r.inflow.norm();
    r.alpha = std::atan2(r.inflow.dot(n), r.inflow.dot(c));

    const double sep = separation(r.alpha);
    r.cl = section_->cl(r.alpha) * (attached_scale_ + (1.0 - attached_scale_) * sep);
    r.cd_induced = (1.0 - sep) * r.cl * r.cl * induced_factor_;
    r.cd = section_->cd(r.alpha) + r.cd_induced;
    r.cp_chord_fraction = std::abs(r.alpha) <= kHalfPi ? 0.25 + 0.25 * sep : 0.75 - 0.25 * sep;
    r.centre_of_pressure =
        g.quarter_chord_centre + (r.cp_chord_fraction - 0.25) * mac_ * c;

    const double scale = 0.5 * density * g.planform_area * r.speed *
                         smoothstep(r.speed / params_.blend_speed);
    r.lift = scale * r.cl * r.inflow.cross(s);
    r.drag = scale * r.cd * r.inflow;
    r.force = r.lift + r.drag;
    return r;
}

FoilForceParams load_foil_force_params(const std::filesystem::path& boat_dir,
                                       const std::string& name, const Vec3& reference_point) {
    const Document app_doc = read_document(boat_dir / "appendages.json");
    const Document res_doc = read_document(boat_dir / "hull_resistance.json");
    const Node foil = app_doc.root().at("foils").at(name);

    FoilForceParams p;
    p.density = res_doc.root().at("water").at("density").positive_value();
    p.reference_point = reference_point;

    FoilGeometry& g = p.foil.geometry;
    g.span = foil.at("span").positive_value();
    g.root_chord = foil.at("root_chord").positive_value();
    g.tip_chord = foil.at("tip_chord").positive_value();
    const Node sweep = foil.at("quarter_chord_sweep");
    g.quarter_chord_sweep = sweep.value() * kDeg;
    if (!(std::abs(g.quarter_chord_sweep) < kHalfPi)) {
        sweep.at("value").fail("expected a sweep within +-90 degrees");
    }
    g.planform_area = foil.at("planform_area").positive_value();
    g.mirror_factor = foil.at("effective_aspect_ratio").positive_value() /
                      foil.at("geometric_aspect_ratio").positive_value();
    g.quarter_chord_centre = foil.at("quarter_chord_centre").at("value").vec3();
    g.span_direction = Vec3(0.0, 0.0, -1.0);
    g.chord_direction = Vec3(-1.0, 0.0, 0.0);
    return p;
}

FoilForceModule::FoilForceModule(std::shared_ptr<const FoilSection> section,
                                 FoilForceParams params,
                                 std::shared_ptr<const WaterCurrent> current)
    : foil_(std::move(section), std::move(params.foil)),
      density_(params.density),
      reference_point_(params.reference_point),
      current_(current ? std::move(current) : std::make_shared<StillWater>()) {
    if (!(density_ > 0.0)) throw std::invalid_argument("FoilForceModule: density must be positive");
}

void FoilForceModule::accumulate(const State& state, double t, Wrench& wrench) {
    const Mat3 rot = state.rotation();
    const Vec3 arm = foil_.params().geometry.quarter_chord_centre - reference_point_;
    const Vec3 point_world = state.position + rot * arm;
    const Vec3 inflow = rot.transpose() * current_->velocity(point_world, t) -
                        (state.linear_velocity + state.angular_velocity.cross(arm));

    last_ = foil_.evaluate(inflow, density_);
    last_wrench_.force = last_.force;
    last_wrench_.moment = (last_.centre_of_pressure - reference_point_).cross(last_.force);
    wrench += last_wrench_;
}

}  // namespace sail
