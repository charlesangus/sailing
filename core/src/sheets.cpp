#include "sail/sheets.hpp"

#include <algorithm>
#include <cmath>
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
constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kTiny = 1e-12;

Vec3 rotate(const Vec3& v, const Vec3& axis, double angle) {
    const double c = std::cos(angle);
    return v * c + axis.cross(v) * std::sin(angle) + axis * (axis.dot(v) * (1.0 - c));
}

// The sheet point's circle about the luff seen from a lead: the point minus the lead is
// q + cos(swing) c + sin(swing) w, with c and w perpendicular and of equal length, so its squared
// length is |q|^2 + |c|^2 + 2 rho cos(swing - psi).
struct SheetCircle {
    double base = 0.0;
    double rho = 0.0;
    double psi = 0.0;
};

SheetCircle sheet_circle(const Vec3& tack, const Vec3& axis, const Vec3& point, const Vec3& lead) {
    const Vec3 p = point - tack;
    const double along = axis.dot(p);
    const Vec3 c = p - along * axis;
    const Vec3 w = axis.cross(c);
    const Vec3 q = tack + along * axis - lead;
    SheetCircle out;
    out.base = q.squaredNorm() + c.squaredNorm();
    const double a = q.dot(c);
    const double b = q.dot(w);
    out.rho = std::hypot(a, b);
    out.psi = std::atan2(b, a);
    return out;
}

std::string range_error(const SheetedSailParams& p, const std::string& name) {
    if (!(p.max_angle > 0.0 && p.max_angle < kPi)) return name + " max_angle must be in (0, pi)";
    if (!(p.flat_at_full_backstay >= 0.0 && p.flat_at_full_backstay <= 1.0)) {
        return name + " flat_at_full_backstay must be in [0, 1]";
    }
    if (!(p.reefed_area_fraction > 0.0 && p.reefed_area_fraction <= 1.0)) {
        return name + " reefed_area_fraction must be in (0, 1]";
    }
    return {};
}

std::string range_error(const SheetParams& p) {
    std::string e = range_error(p.main, "main");
    if (!e.empty()) return e;
    e = range_error(p.jib, "jib");
    if (!e.empty()) return e;
    if (!(p.traveler_span >= 0.0) || !std::isfinite(p.traveler_span)) {
        return "traveler_span must be non-negative";
    }
    if (!(p.jib_flat_with_car_aft >= 0.0 && p.jib_flat_with_car_aft <= 1.0)) {
        return "jib flat_with_car_aft must be in [0, 1]";
    }
    if (!(p.reef_transition_time > 0.0) || !std::isfinite(p.reef_transition_time)) {
        return "reef transition_time must be positive";
    }
    return {};
}

double smoothstep(double x) {
    const double t = std::clamp(x, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

}  // namespace

SheetGeometry::SheetGeometry(const Vec3& tack, const Vec3& head, const Vec3& clew,
                             const Vec3& sheet_point)
    : tack_(tack), clew_(clew), sheet_point_(sheet_point) {
    const Vec3 luff = head - tack;
    if (!(luff.norm() > 1e-9)) throw std::invalid_argument("SheetGeometry: zero luff");
    axis_ = luff.normalized();
    const double swing_y = axis_.cross(clew - tack).y();
    if (!(std::abs(swing_y) > 1e-9)) {
        throw std::invalid_argument("SheetGeometry: the clew does not swing across the boat");
    }
    if (swing_y < 0.0) axis_ = -axis_;
    const Vec3 p = sheet_point - tack;
    if (!((p - axis_.dot(p) * axis_).norm() > 1e-9)) {
        throw std::invalid_argument("SheetGeometry: sheet point on the luff");
    }
}

Vec3 SheetGeometry::clew(double swing) const {
    return tack_ + rotate(clew_ - tack_, axis_, swing);
}

Vec3 SheetGeometry::sheet_point(double swing) const {
    return tack_ + rotate(sheet_point_ - tack_, axis_, swing);
}

double SheetGeometry::chord_angle(double swing) const {
    const Vec3 chord = clew(swing) - tack_;
    return std::atan2(chord.y(), -chord.x());
}

double SheetGeometry::swing_for_chord_angle(double angle) const {
    const double target = std::clamp(angle, 0.0, kPi);
    double lo = 0.0;
    double hi = kPi;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        const double a = chord_angle(mid);
        if (a < target && a >= 0.0) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

double SheetGeometry::sheet_length(double swing, const Vec3& lead) const {
    return (sheet_point(swing) - lead).norm();
}

double SheetGeometry::nearest_swing(const Vec3& lead) const {
    const SheetCircle s = sheet_circle(tack_, axis_, sheet_point_, lead);
    if (!(s.rho > kTiny)) return 0.0;
    return std::remainder(s.psi + kPi, 2.0 * kPi);
}

double SheetGeometry::limit_swing(const Vec3& lead, double length) const {
    const SheetCircle s = sheet_circle(tack_, axis_, sheet_point_, lead);
    if (!(s.rho > kTiny)) return kPi;
    const double nearest = std::remainder(s.psi + kPi, 2.0 * kPi);
    const double k = std::clamp((length * length - s.base) / (2.0 * s.rho), -1.0, 1.0);
    return std::min(nearest + kPi - std::acos(k), kPi);
}

ReefActuator::ReefActuator(double transition_time) : transition_time_(transition_time) {
    if (!(transition_time > 0.0) || !std::isfinite(transition_time)) {
        throw std::invalid_argument("ReefActuator: transition time must be positive");
    }
}

void ReefActuator::step(double dt) {
    if (!(dt > 0.0) || !std::isfinite(dt)) return;
    const double reach = dt / transition_time_;
    const double gap = command_ - progress_;
    // The tolerance absorbs the rounding of summed steps, so the reef lands on the step that
    // completes the transition time rather than one later.
    if (std::abs(gap) <= reach * (1.0 + 1e-9)) {
        progress_ = command_;
    } else {
        progress_ += gap > 0.0 ? reach : -reach;
    }
}

void ReefActuator::reset(double reefed) {
    Controls c;
    c.reef = reefed;
    progress_ = clamped(c).reef;
    command_ = progress_;
}

double ReefActuator::depth() const { return smoothstep(progress_); }

SheetParams load_sheet_params(const std::filesystem::path& boat_dir) {
    const Document sails_doc = read_document(boat_dir / "sails.json");
    const Document sheets_doc = read_document(boat_dir / "sheets.json");
    const Document hull_doc = read_document(boat_dir / "hull_hydro.json");

    const Node transform = hull_doc.root().at("transform");
    const Vec3 offset(transform.at("stem_x_in_body").number(), 0.0,
                      transform.at("flotation_z_in_body").number());
    const auto point = [&](const Node& n, const std::string& key) {
        return n.at(key).at("value").vec3() + offset;
    };
    const auto fraction = [](const Node& n, const std::string& key) {
        const Node entry = n.at(key);
        const double v = entry.value();
        if (!(v >= 0.0 && v <= 1.0)) entry.at("value").fail("expected a number in [0, 1]");
        return v;
    };
    const auto sail = [&](const std::string& name, const std::string& sheet_key) {
        const Node geometry = sails_doc.root().at(name);
        const Node trim = sheets_doc.root().at(name);
        SheetedSailParams p;
        p.sail = name;
        p.tack = point(geometry, "tack");
        p.head = point(geometry, "head");
        p.clew = point(geometry, "clew");
        p.sheet_point = point(geometry, sheet_key);
        const Node max_angle = trim.at("max_angle");
        p.max_angle = max_angle.positive_value() * kDeg;
        if (!(p.max_angle < kPi)) max_angle.at("value").fail("expected less than 180 degrees");
        p.flat_at_full_backstay = fraction(trim, "flat_at_full_backstay");
        p.reefed_area_fraction = fraction(trim, "reefed_area_fraction");
        if (!(p.reefed_area_fraction > 0.0)) {
            trim.at("reefed_area_fraction").at("value").fail("expected a positive number");
        }
        return p;
    };

    SheetParams p;
    p.main = sail("main", "sheet_point_on_boom");
    const Node main = sails_doc.root().at("main");
    p.traveler_centre = point(main, "traveler_centre");
    p.traveler_span = main.at("traveler_span").positive_value();

    p.jib = sail("jib", "clew");
    const Node jib = sails_doc.root().at("jib");
    p.jib_track_forward = point(jib, "track_fwd_end");
    p.jib_track_aft = point(jib, "track_aft_end");
    p.jib_flat_with_car_aft = fraction(sheets_doc.root().at("jib"), "flat_with_car_aft");
    p.reef_transition_time = sheets_doc.root().at("reef").at("transition_time").positive_value();

    try {
        SheetGeometry(p.main.tack, p.main.head, p.main.clew, p.main.sheet_point);
        SheetGeometry(p.jib.tack, p.jib.head, p.jib.clew, p.jib.sheet_point);
    } catch (const std::invalid_argument& e) {
        throw BoatDataError(sails_doc.file, e.what());
    }
    return p;
}

SheetController::SheetController(SheetParams params, SailSet& sails,
                                 std::shared_ptr<const WindField> wind)
    : params_(std::move(params)),
      sails_(sails),
      wind_(std::move(wind)),
      reef_(params_.reef_transition_time),
      main_(make_sail(params_.main, params_.traveler_centre)),
      jib_(make_sail(params_.jib, 0.5 * (params_.jib_track_forward + params_.jib_track_aft))) {
    if (!wind_) throw std::invalid_argument("SheetController: null wind field");
}

SheetController::Sail SheetController::make_sail(const SheetedSailParams& p,
                                                 const Vec3& reference_lead) const {
    const std::string error = range_error(params_);
    if (!error.empty()) throw std::invalid_argument("SheetController: " + error);
    Sail s(sails_.index(p.sail), SheetGeometry(p.tack, p.head, p.clew, p.sheet_point));
    s.stop_swing = s.geometry.swing_for_chord_angle(p.max_angle);
    s.shortest = s.geometry.sheet_length(s.geometry.nearest_swing(reference_lead), reference_lead);
    s.longest = s.geometry.sheet_length(s.stop_swing, reference_lead);
    if (!(s.longest > s.shortest + 1e-6)) {
        throw std::invalid_argument("SheetController: " + p.sail +
                                    " sheet does not lengthen as the sail eases");
    }
    s.luff_aoa = sails_.params().sails[s.index].coefficients->params().luff_aoa;
    return s;
}

void SheetController::set_controls(const Controls& controls) {
    controls_ = clamped(controls);
    reef_.set_command(controls_);
}

void SheetController::set_wind(std::shared_ptr<const WindField> wind) {
    if (!wind) throw std::invalid_argument("SheetController: null wind field");
    wind_ = std::move(wind);
}

void SheetController::reset(double reefed) {
    reef_.reset(reefed);
    has_time_ = false;
    last_time_ = 0.0;
    main_.tack.reset();
    jib_.tack.reset();
}

void SheetController::accumulate(const State& state, double t, Wrench&) { update(state, t); }

void SheetController::update(const State& state, double t) {
    if (has_time_ && t > last_time_) reef_.step(t - last_time_);
    has_time_ = true;
    last_time_ = t;

    const double backstay = controls_.backstay;
    const double main_flat = 1.0 - backstay * (1.0 - params_.main.flat_at_full_backstay);
    const double jib_flat = (1.0 - backstay * (1.0 - params_.jib.flat_at_full_backstay)) *
                            (1.0 - controls_.jib_car * (1.0 - params_.jib_flat_with_car_aft));

    const Vec3 car(0.0, 0.5 * controls_.traveler * params_.traveler_span, 0.0);
    trim(main_, params_.main, state, t, controls_.mainsheet, controls_.mainsheet,
         params_.traveler_centre + car, main_flat);
    const Vec3 jib_lead = params_.jib_track_forward +
                          controls_.jib_car * (params_.jib_track_aft - params_.jib_track_forward);
    trim(jib_, params_.jib, state, t, controls_.jib_sheet_port, controls_.jib_sheet_starboard,
         jib_lead, jib_flat);
}

void SheetController::trim(Sail& sail, const SheetedSailParams& p, const State& state, double t,
                           double port_sheet, double starboard_sheet, const Vec3& port_lead,
                           double flat) {
    const SailState& placed = sails_.last_result().sails[sail.index];
    const ApparentWind wind = apparent_wind(
        state, placed.centre_of_effort - sails_.params().reference_point, *wind_, t);
    const double awa = wind.awa_sail;
    if (!sail.tack) {
        sail.tack = awa < 0.0 ? Tack::Port : Tack::Starboard;
    } else if (*sail.tack == Tack::Starboard && awa < 0.0) {
        sail.tack = Tack::Port;
    } else if (*sail.tack == Tack::Port && awa > 0.0) {
        sail.tack = Tack::Starboard;
    }
    const double side = *sail.tack == Tack::Starboard ? 1.0 : -1.0;
    const double sheet = side > 0.0 ? port_sheet : starboard_sheet;

    const double length = sail.longest - sheet * (sail.longest - sail.shortest);
    const double limit = std::min(sail.geometry.limit_swing(port_lead, length), sail.stop_swing);
    const double sheet_angle = sail.geometry.chord_angle(limit);
    const double free_angle = std::abs(awa);
    const double angle = std::min(free_angle, sheet_angle);

    const double area_fraction = 1.0 - reef_.depth() * (1.0 - p.reefed_area_fraction);

    SheetedSailState& st = sail.state;
    st.wind = wind;
    st.tack = *sail.tack;
    st.lead = Vec3(port_lead.x(), side * port_lead.y(), port_lead.z());
    st.sheet_length = length;
    st.free_angle = side * free_angle;
    st.sheet_angle = side * sheet_angle;
    st.angle = side * angle;
    st.aoa = awa - st.angle;
    st.luffing = std::abs(st.aoa) < sail.luff_aoa;
    st.trim.flat = std::clamp(flat, 0.0, 1.0);
    st.trim.reef = std::sqrt(std::clamp(area_fraction, 0.0, 1.0));
    st.trim.angle_of_attack = st.aoa;
    sails_.set_trim(sail.index, st.trim);
}

}  // namespace sail
