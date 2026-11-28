#include "sail/sails.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <fstream>
#include <map>
#include <numbers>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/LU>

#include "dossier_json.hpp"
#include "sail/calibration.hpp"

namespace sail {

namespace {

using dossier::Document;
using dossier::Node;
using dossier::read_document;

constexpr double kPi = std::numbers::pi;
constexpr double kDeg = std::numbers::pi / 180.0;
// Below this speed (m/s) the apparent wind has no direction and the sails carry no load.
constexpr double kMinSpeed = 1e-9;
constexpr double kTiny = 1e-12;

// ORC VPP Documentation 2023, Figure 5.14 (revised 2023), read off the plotted curve at 5 degree
// steps of AWA from 0 to 90.
constexpr std::array<double, 19> kKheffAwaDeg = {0,  5,  10, 15, 20, 25, 30, 35, 40, 45,
                                                 50, 55, 60, 65, 70, 75, 80, 85, 90};
constexpr std::array<double, 19> kKheff = {1.003, 1.191, 1.350, 1.435, 1.450, 1.370, 1.250,
                                           1.136, 1.030, 0.948, 0.900, 0.869, 0.844, 0.825,
                                           0.810, 0.801, 0.799, 0.800, 0.800};
constexpr double kKheffReach = 0.800;

// ORC VPP Documentation 2023, 5.4.3, the fcdmult table against flat.
constexpr std::array<double, 15> kFcdFlat = {0.10, 0.20, 0.30, 0.40, 0.50, 0.55, 0.60, 0.65,
                                             0.70, 0.75, 0.80, 0.85, 0.90, 0.95, 1.00};
constexpr std::array<double, 15> kFcdMult = {1.06,  1.06,  1.06,  1.06,  1.06,
                                             1.06,  1.055, 1.048, 1.035, 1.020,
                                             1.008, 1.002, 1.000, 1.004, 1.06};

template <std::size_t N>
double interpolate(const std::array<double, N>& xs, const std::array<double, N>& ys, double x) {
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    const auto it = std::upper_bound(xs.begin(), xs.end(), x);
    const std::size_t i = static_cast<std::size_t>(it - xs.begin());
    const double w = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
    return ys[i - 1] + w * (ys[i] - ys[i - 1]);
}

double kheff(double awa) { return interpolate(kKheffAwaDeg, kKheff, awa / kDeg); }
double fcdmult(double flat) { return interpolate(kFcdFlat, kFcdMult, flat); }

// ORC VPP Documentation 2023, 5.6.2: only the part of the headsail beyond the foretriangle, the
// overlap behind the mast, is blanketed, from 135 degrees to fully at 180.
double headsail_blanketing(double beta, double area, double foretriangle_area, double scale) {
    if (beta <= 135.0 * kDeg || !(area > 0.0)) return 1.0;
    const double f = (area - std::min(area, foretriangle_area)) / area;
    return std::max(0.0, 1.0 - scale * f * (beta - 135.0 * kDeg) / (45.0 * kDeg));
}

std::string trim_text(const std::string& text) {
    const std::size_t begin = text.find_first_not_of(" \t\r");
    if (begin == std::string::npos) return {};
    const std::size_t end = text.find_last_not_of(" \t\r");
    return text.substr(begin, end - begin + 1);
}

std::vector<std::string> split(const std::string& text) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (true) {
        const std::size_t comma = text.find(',', start);
        out.push_back(trim_text(text.substr(start, comma == std::string::npos ? std::string::npos
                                                                             : comma - start)));
        if (comma == std::string::npos) return out;
        start = comma + 1;
    }
}

bool parse_number(const std::string& text, double& out) {
    if (text.empty()) return false;
    const char* first = text.data();
    const char* last = text.data() + text.size();
    const auto [ptr, ec] = std::from_chars(first, last, out);
    return ec == std::errc() && ptr == last && std::isfinite(out);
}

}  // namespace

CubicSpline::CubicSpline(std::vector<double> x, std::vector<double> y)
    : x_(std::move(x)), y_(std::move(y)) {
    const std::size_t n = x_.size();
    if (n < 4 || y_.size() != n) {
        throw std::invalid_argument("CubicSpline: need 4 or more points");
    }
    for (std::size_t i = 1; i < n; ++i) {
        if (!(x_[i] > x_[i - 1])) {
            throw std::invalid_argument("CubicSpline: x must be strictly increasing");
        }
    }
    const auto idx = [](std::size_t i) { return static_cast<Eigen::Index>(i); };
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(idx(n), idx(n));
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(idx(n));
    std::vector<double> h(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i) h[i] = x_[i + 1] - x_[i];
    for (std::size_t i = 1; i + 1 < n; ++i) {
        a(idx(i), idx(i - 1)) = h[i - 1];
        a(idx(i), idx(i)) = 2.0 * (h[i - 1] + h[i]);
        a(idx(i), idx(i + 1)) = h[i];
        rhs(idx(i)) = 6.0 * ((y_[i + 1] - y_[i]) / h[i] - (y_[i] - y_[i - 1]) / h[i - 1]);
    }
    // Not-a-knot: the third derivative is continuous across the second and the next-to-last
    // points.
    a(0, 0) = h[1];
    a(0, 1) = -(h[0] + h[1]);
    a(0, 2) = h[0];
    a(idx(n - 1), idx(n - 3)) = h[n - 2];
    a(idx(n - 1), idx(n - 2)) = -(h[n - 3] + h[n - 2]);
    a(idx(n - 1), idx(n - 1)) = h[n - 3];
    const Eigen::VectorXd m = a.partialPivLu().solve(rhs);
    m_.assign(m.data(), m.data() + n);
}

double CubicSpline::operator()(double x) const {
    if (x <= x_.front()) return y_.front();
    if (x >= x_.back()) return y_.back();
    const auto it = std::upper_bound(x_.begin(), x_.end(), x);
    const std::size_t i = static_cast<std::size_t>(it - x_.begin()) - 1;
    const double h = x_[i + 1] - x_[i];
    const double t = x - x_[i];
    const double slope = (y_[i + 1] - y_[i]) / h - h * (2.0 * m_[i] + m_[i + 1]) / 6.0;
    return y_[i] + t * (slope + t * (0.5 * m_[i] + t * (m_[i + 1] - m_[i]) / (6.0 * h)));
}

SailCoefficients::SailCoefficients(SailCoefficientParams params, std::vector<double> awa_deg,
                                   std::vector<double> cl_high, std::vector<double> cd_high,
                                   std::vector<double> cl_low, std::vector<double> cd_low)
    : params_(params) {
    const std::size_t n = awa_deg.size();
    if (n < 4 || cl_high.size() != n || cd_high.size() != n || cl_low.size() != n ||
        cd_low.size() != n) {
        throw std::invalid_argument("SailCoefficients: need 4 or more rows of every column");
    }
    if (awa_deg.front() < 0.0 || awa_deg.back() != 180.0) {
        throw std::invalid_argument("SailCoefficients: awa_deg must run from 0 or more to 180");
    }
    if (!(params_.kp >= 0.0) || !(params_.aoa_design > 0.0) || !(params_.max_ease >= 0.0) ||
        !(params_.luff_aoa >= 0.0) || !(params_.stall_width > 0.0) ||
        !(params_.normal_force >= 0.0)) {
        throw std::invalid_argument(
            "SailCoefficients: parameters must be non-negative, aoa_design and stall_width "
            "positive");
    }
    cl_high_ = CubicSpline(awa_deg, std::move(cl_high));
    cd_high_ = CubicSpline(awa_deg, std::move(cd_high));
    cl_low_ = CubicSpline(awa_deg, std::move(cl_low));
    cd_low_ = CubicSpline(std::move(awa_deg), std::move(cd_low));
}

SailCoefficients SailCoefficients::load(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw BoatDataError(file, "cannot open file");

    constexpr const char* kHeader = "awa_deg,cl_high,cd_high,cl_low,cd_low";
    std::map<std::string, std::string> keys;
    std::array<std::vector<double>, 5> columns;
    bool header = false;
    std::string text;
    std::size_t line = 0;
    while (std::getline(in, text)) {
        ++line;
        const std::string where = "line " + std::to_string(line) + ": ";
        const std::string row = trim_text(text);
        if (row.empty()) continue;
        if (!header) {
            if (row == kHeader) {
                header = true;
                continue;
            }
            const std::vector<std::string> fields = split(row);
            if (fields.size() != 2 || fields[0].empty()) {
                throw BoatDataError(file, where + "expected key,value or the header " + kHeader);
            }
            if (!keys.emplace(fields[0], fields[1]).second) {
                throw BoatDataError(file, where + "duplicate key '" + fields[0] + "'");
            }
            continue;
        }
        const std::vector<std::string> fields = split(row);
        double v[5];
        bool ok = fields.size() == 5;
        for (std::size_t k = 0; ok && k < 5; ++k) ok = parse_number(fields[k], v[k]);
        if (!ok) throw BoatDataError(file, where + "expected five numbers " + kHeader);
        for (std::size_t k = 0; k < 5; ++k) columns[k].push_back(v[k]);
    }
    if (!header) throw BoatDataError(file, std::string("missing the header ") + kHeader);

    const auto take = [&](const std::string& key) {
        const auto it = keys.find(key);
        if (it == keys.end()) throw BoatDataError(file, "missing key '" + key + "'");
        std::string value = it->second;
        keys.erase(it);
        return value;
    };
    const auto number = [&](const std::string& key) {
        double v = 0.0;
        if (!parse_number(take(key), v)) {
            throw BoatDataError(file, "key '" + key + "': expected a number");
        }
        return v;
    };

    SailCoefficientParams p;
    const std::string role = take("role");
    if (role == "main") {
        p.role = SailRole::main;
    } else if (role == "headsail") {
        p.role = SailRole::headsail;
    } else {
        throw BoatDataError(file, "key 'role': expected main or headsail");
    }
    p.kp = number("kp");
    p.aoa_design = number("aoa_design_deg") * kDeg;
    p.max_ease = number("max_ease_deg") * kDeg;
    p.luff_aoa = number("luff_aoa_deg") * kDeg;
    p.stall_width = number("stall_width_deg") * kDeg;
    p.normal_force = number("normal_force_coefficient");
    if (!keys.empty()) throw BoatDataError(file, "unknown key '" + keys.begin()->first + "'");

    try {
        return SailCoefficients(p, std::move(columns[0]), std::move(columns[1]),
                                std::move(columns[2]), std::move(columns[3]),
                                std::move(columns[4]));
    } catch (const std::invalid_argument& e) {
        throw BoatDataError(file, e.what());
    }
}

SailCoefficients::Envelope SailCoefficients::envelope(double awa, double high) const {
    const double beta = std::min(std::abs(awa), kPi) / kDeg;
    const double h = std::clamp(high, 0.0, 1.0);
    return {h * cl_high_(beta) + (1.0 - h) * cl_low_(beta),
            h * cd_high_(beta) + (1.0 - h) * cd_low_(beta)};
}

double SailCoefficients::optimum_aoa(double awa) const {
    const double beta = std::min(std::abs(awa), kPi);
    return std::max(std::min(params_.aoa_design, beta), beta - params_.max_ease);
}

SailSection SailCoefficients::optimal(double awa, double high) const {
    const double opt = optimum_aoa(awa);
    return section(awa, awa < 0.0 ? -opt : opt, high);
}

SailSection SailCoefficients::section(double awa, double aoa, double high) const {
    const double beta = std::min(std::abs(awa), kPi);
    const Envelope env = envelope(beta, high);
    const double opt = optimum_aoa(beta);
    const double wrapped = std::remainder(aoa, 2.0 * kPi);
    const double a = std::abs(wrapped);

    double cl = 0.0;
    double cd = env.cd;
    if (a <= opt) {
        cl = opt > kTiny ? env.cl * (a / opt) : 0.0;
    } else {
        const double x = (a - opt) / params_.stall_width;
        const double w = 1.0 / (1.0 + x * x);
        const double s = std::sin(a);
        const double c = std::cos(a);
        cl = w * env.cl + (1.0 - w) * params_.normal_force * s * c;
        cd = w * env.cd + (1.0 - w) * params_.normal_force * s * s;
    }

    SailSection out;
    out.cl = wrapped < 0.0 ? -cl : cl;
    out.cd = cd;
    out.aoa = wrapped;
    out.luffing = a < params_.luff_aoa;
    out.stalled = a > params_.aoa_design + kTiny;
    out.backed = a > kTiny && ((wrapped < 0.0) != (awa < 0.0));
    return out;
}

SailSetParams load_sail_set_params(const std::filesystem::path& boat_dir,
                                   const std::filesystem::path& sail_data_dir,
                                   const Vec3& reference_point) {
    const Document sails_doc = read_document(boat_dir / "sails.json");
    const Document rig_doc = read_document(boat_dir / "rig.json");
    const Document hull_doc = read_document(boat_dir / "hull_hydro.json");

    const Node transform = hull_doc.root().at("transform");
    const double flotation_z = transform.at("flotation_z_in_body").number();
    const Vec3 offset(transform.at("stem_x_in_body").number(), 0.0, flotation_z);

    const Calibration calibration = load_calibration(boat_dir);
    SailSetParams p;
    p.reference_point = reference_point;
    p.effective_height_scale = calibration.effective_height;
    p.kheff_rise_scale = calibration.kheff_upwind_rise;
    p.blanketing_scale = calibration.headsail_blanketing;
    p.rig.waterplane_z =
        hull_doc.root().at("conditions").at("sailing").at("result").at("sinkage_m").number();
    const Node rig = rig_doc.root();
    p.rig.base_z = rig.at("freeboard_at_mast").value() + flotation_z;
    p.rig.foretriangle_height = rig.at("i_jib").positive_value();
    p.rig.foretriangle_base = rig.at("j").positive_value();

    std::map<std::string, std::shared_ptr<const SailCoefficients>> tables;
    const Node root = sails_doc.root();
    for (auto it = root.json().begin(); it != root.json().end(); ++it) {
        if (!it->is_object()) continue;
        const Node entry = root.at(it.key());
        if (!(entry.has("tack") && entry.has("head") && entry.has("clew") && entry.has("area"))) {
            continue;
        }
        SailSpec s;
        s.name = it.key();
        s.type = entry.has("type") ? entry.at("type").at("value").string() : s.name;
        auto& table = tables[s.type];
        if (!table) {
            table = std::make_shared<const SailCoefficients>(
                SailCoefficients::load(sail_data_dir / ("orc_" + s.type + ".csv")));
        }
        s.coefficients = table;
        const bool main = table->params().role == SailRole::main;
        s.lift_scale = main ? calibration.main_lift : calibration.jib_lift;
        s.parasitic_drag_scale =
            main ? calibration.main_parasitic_drag : calibration.jib_parasitic_drag;
        s.area = entry.at("area").positive_value();
        s.tack = entry.at("tack").at("value").vec3() + offset;
        s.head = entry.at("head").at("value").vec3() + offset;
        s.clew = entry.at("clew").at("value").vec3() + offset;

        const Vec3 luff = s.head - s.tack;
        const Vec3 foot = s.clew - s.tack;
        const double triangle = 0.5 * luff.cross(foot).norm();
        if (!(triangle > 1e-6)) entry.fail("tack, head and clew form no triangle");
        s.luff = luff.norm();
        s.foot = foot.norm();
        s.leech = (s.clew - s.head).norm();
        s.roach = s.area / triangle - 1.0;
        const Vec3 along = luff / s.luff;
        s.luff_perpendicular = (foot - foot.dot(along) * along).norm();
        s.centre_of_effort = (s.tack + s.head + s.clew) / 3.0 + luff * (s.roach / 3.0);
        p.sails.push_back(std::move(s));
    }
    if (p.sails.empty()) throw BoatDataError(sails_doc.file, "no sails");
    return p;
}

SailSet::SailSet(SailSetParams params, std::shared_ptr<const WindField> wind)
    : params_(std::move(params)), wind_(std::move(wind)) {
    if (!wind_) throw std::invalid_argument("SailSet: null wind field");
    if (params_.sails.empty()) throw std::invalid_argument("SailSet: no sails");
    for (std::size_t i = 0; i < params_.sails.size(); ++i) {
        const SailSpec& s = params_.sails[i];
        if (!s.coefficients) throw std::invalid_argument("SailSet: sail without coefficients");
        const SailRole role = s.coefficients->params().role;
        if (role == SailRole::main && !main_) main_ = i;
        if (role == SailRole::headsail && !headsail_) headsail_ = i;
    }
    trims_.resize(params_.sails.size());
    last_.sails.resize(params_.sails.size());
    place(rig_state(), last_.sails);
}

std::size_t SailSet::index(const std::string& name) const {
    for (std::size_t i = 0; i < params_.sails.size(); ++i) {
        if (params_.sails[i].name == name) return i;
    }
    throw std::out_of_range("SailSet: no sail named '" + name + "'");
}

void SailSet::set_trim(std::size_t i, const SailTrim& trim) {
    if (i >= trims_.size()) throw std::out_of_range("SailSet: sail index out of range");
    if (!(trim.flat >= 0.0 && trim.flat <= 1.0) || !(trim.reef >= 0.0 && trim.reef <= 1.0)) {
        throw std::invalid_argument("SailSet: flat and reef must be in [0, 1]");
    }
    if (trim.angle_of_attack && !std::isfinite(*trim.angle_of_attack)) {
        throw std::invalid_argument("SailSet: angle of attack must be finite");
    }
    trims_[i] = trim;
}

void SailSet::set_wind(std::shared_ptr<const WindField> wind) {
    if (!wind) throw std::invalid_argument("SailSet: null wind field");
    wind_ = std::move(wind);
}

SailSet::RigState SailSet::rig_state() const {
    const SailRig& rig = params_.rig;
    RigState out;
    out.foretriangle_area = 0.5 * rig.foretriangle_height * rig.foretriangle_base;

    double main_top = 0.0;
    if (main_) {
        const SailSpec& m = params_.sails[*main_];
        main_top = m.tack.z() + (m.head.z() - m.tack.z()) * trims_[*main_].reef;
        out.roach = std::max(0.0, m.roach);
    }
    double head_top = 0.0;
    if (headsail_) {
        const SailSpec& h = params_.sails[*headsail_];
        const double reef = trims_[*headsail_].reef;
        const double ig = rig.foretriangle_height * reef;
        head_top = rig.base_z + ig;
        const double main_hoist = main_top - rig.base_z;
        if (main_ && main_hoist > kTiny) out.fractionality = std::min(1.0, ig / main_hoist);
        if (rig.foretriangle_base > 0.0) {
            out.overlap = h.luff_perpendicular * reef / rig.foretriangle_base;
        }
    }
    double top = main_ ? main_top : head_top;
    if (main_ && headsail_ && head_top > main_top) top = 0.5 * (main_top + head_top);
    out.top_above_water = std::max(0.0, top - rig.waterplane_z);
    return out;
}

void SailSet::place(const RigState& rig, std::vector<SailState>& states) const {
    const double base = params_.rig.base_z;
    for (std::size_t i = 0; i < states.size(); ++i) {
        const SailSpec& s = params_.sails[i];
        const SailTrim& trim = trims_[i];
        SailState& st = states[i];
        st.area = s.area * trim.reef * trim.reef;
        Vec3 ce = s.tack + (s.centre_of_effort - s.tack) * trim.reef;
        // ORC VPP Documentation 2023, eq. 5.49.
        const double slack = 1.0 - trim.flat;
        const double twist = 1.0 - 0.406 * slack - 0.902 * slack * (1.0 - rig.fractionality);
        ce.z() = base + (ce.z() - base) * twist;
        st.centre_of_effort = ce;
    }
}

SailPlanCoefficients SailSet::combine(const RigState& rig, std::vector<SailState>& states) const {
    SailPlanCoefficients plan;
    plan.fractionality = rig.fractionality;
    plan.overlap = rig.overlap;
    plan.effective_span = 1.1 + 0.08 * (rig.roach - 0.2) +
                          0.5 * (0.68 + 0.31 * rig.fractionality + 0.075 * rig.overlap - 1.1);

    double area = 0.0;
    double weight_sum = 0.0;
    double beta_sum = 0.0;
    double cl_sum = 0.0;
    double cd0_sum = 0.0;
    double quadratic = 0.0;
    for (std::size_t i = 0; i < states.size(); ++i) {
        const SailSpec& s = params_.sails[i];
        const SailTrim& trim = trims_[i];
        SailState& st = states[i];
        const SailCoefficients& c = *s.coefficients;
        const double awa = st.wind.awa_sail;
        const double beta = std::abs(awa);
        const SailSection sec = trim.angle_of_attack
                                    ? c.section(awa, *trim.angle_of_attack, s.high_coefficients)
                                    : c.optimal(awa, s.high_coefficients);
        const double bk = c.params().role == SailRole::headsail
                              ? headsail_blanketing(beta, st.area, rig.foretriangle_area,
                                                    params_.blanketing_scale)
                              : 1.0;
        const double weight = bk * st.area;
        const double cl = trim.flat * s.lift_scale * sec.cl;
        const double cd0 = s.parasitic_drag_scale * sec.cd;
        st.blanketing = bk;
        st.aoa = sec.aoa;
        st.luffing = sec.luffing;
        st.stalled = sec.stalled;
        st.backed = sec.backed;
        st.cl = cl;
        st.cd_parasitic = cd0;

        area += st.area;
        weight_sum += weight;
        beta_sum += beta * weight;
        cl_sum += cl * weight;
        cd0_sum += cd0 * weight;
        quadratic += c.params().kp * cl * cl * weight;
    }

    if (!(area > kTiny)) {
        for (SailState& st : states) {
            st.cl = 0.0;
            st.cd = 0.0;
            st.cd_parasitic = 0.0;
        }
        return plan;
    }

    plan.area = area;
    plan.cl = cl_sum / area;
    plan.cd_parasitic = cd0_sum / area;
    plan.kpp = std::abs(cl_sum) > kTiny ? quadratic * area / (cl_sum * cl_sum) : 0.0;
    const double mean_beta = weight_sum > kTiny ? beta_sum / weight_sum : 0.0;
    plan.kheff = kKheffReach +
                 params_.kheff_rise_scale * (kheff(std::min(mean_beta, 0.5 * kPi)) - kKheffReach);
    plan.effective_height =
        params_.effective_height_scale * plan.effective_span * plan.kheff * rig.top_above_water;
    const double induced =
        plan.effective_height > kTiny
            ? area / (kPi * plan.effective_height * plan.effective_height)
            : 0.0;

    double cd_sum = 0.0;
    for (std::size_t i = 0; i < states.size(); ++i) {
        const SailCoefficients& c = *params_.sails[i].coefficients;
        const double flat = trims_[i].flat;
        SailState& st = states[i];
        const double mult = fcdmult(flat);
        const double parasitic_scale = c.params().role == SailRole::headsail ? flat * mult : 1.0;
        const double cl = st.cl;
        const double cd = st.cd_parasitic * parasitic_scale +
                          mult * (c.params().kp * cl * cl + cl * plan.cl * induced);
        st.cl = st.blanketing * cl;
        st.cd = st.blanketing * cd;
        st.cd_parasitic = st.blanketing * st.cd_parasitic * parasitic_scale;
        cd_sum += st.area * st.cd;
    }
    plan.cd = cd_sum / area;
    return plan;
}

SailPlanCoefficients SailSet::coefficients(double awa) const {
    std::vector<SailState> states = last_.sails;
    const RigState rig = rig_state();
    place(rig, states);
    for (SailState& st : states) st.wind.awa_sail = awa;
    return combine(rig, states);
}

void SailSet::accumulate(const State& state, double t, Wrench& wrench) {
    const RigState rig = rig_state();
    place(rig, last_.sails);
    for (SailState& st : last_.sails) {
        st.wind = apparent_wind(state, st.centre_of_effort - params_.reference_point, *wind_, t);
    }
    last_.plan = combine(rig, last_.sails);

    last_.wrench.clear();
    for (SailState& st : last_.sails) {
        st.force.setZero();
        const double speed = st.wind.aws_sail;
        if (speed > kMinSpeed && st.area > 0.0) {
            const double wx = st.wind.body.x() / speed;
            const double wy = st.wind.body.y() / speed;
            const double load = 0.5 * params_.air_density * speed * speed * st.area;
            st.force = load * (st.cl * Vec3(wy, -wx, 0.0) + st.cd * Vec3(wx, wy, 0.0));
        }
        last_.wrench.force += st.force;
        last_.wrench.moment += (st.centre_of_effort - params_.reference_point).cross(st.force);
    }
    wrench += last_.wrench;
}

}  // namespace sail
