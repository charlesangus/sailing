#include "sail/harness/autotrim.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace sail::harness {

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

bool finite(const State& s) {
    return s.position.allFinite() && s.orientation.coeffs().allFinite() &&
           s.linear_velocity.allFinite() && s.angular_velocity.allFinite();
}

double tolerance(const SettleChannel& c, double v, double mean) {
    return v + c.relative * std::abs(mean);
}

enum Channel : std::size_t { kBsp, kHeel, kLeeway, kVmg, kRudder, kTwa, kChannels };

struct Coordinate {
    double Controls::*member;
    double lo;
    double hi;
};

constexpr std::array<Coordinate, 5> kCoordinates{{
    {&Controls::mainsheet, 0.0, 1.0},
    {&Controls::jib_sheet_port, 0.0, 1.0},
    {&Controls::traveler, -1.0, 1.0},
    {&Controls::jib_car, 0.0, 1.0},
    {&Controls::backstay, 0.0, 1.0},
}};

Controls with_jib_sheets_together(Controls c) {
    c.jib_sheet_starboard = c.jib_sheet_port;
    c.tiller = 0.0;
    return c;
}

TrimObjective objective_for(PolarKind kind) {
    switch (kind) {
        case PolarKind::beat:
            return TrimObjective::vmg_up;
        case PolarKind::run:
            return TrimObjective::vmg_down;
        case PolarKind::fixed:
            break;
    }
    return TrimObjective::bsp;
}

}  // namespace

GoldenResult golden_section_max(const std::function<double(double)>& f, double lo, double hi,
                                double tol, int max_evaluations) {
    GoldenResult out;
    out.x = lo;
    if (!(lo < hi) || max_evaluations < 1) return out;
    const double inv_phi = 0.5 * (std::sqrt(5.0) - 1.0);
    auto consider = [&](double x, double v) {
        ++out.evaluations;
        if (v > out.f || out.evaluations == 1) {
            out.x = x;
            out.f = v;
        }
    };
    double a = lo;
    double b = hi;
    double c = b - inv_phi * (b - a);
    double d = a + inv_phi * (b - a);
    const double fc = f(c);
    consider(c, fc);
    if (out.evaluations >= max_evaluations) return out;
    double fd = f(d);
    consider(d, fd);
    double fc_now = fc;
    while (b - a > tol && out.evaluations < max_evaluations) {
        if (fc_now >= fd) {
            b = d;
            d = c;
            fd = fc_now;
            c = b - inv_phi * (b - a);
            fc_now = f(c);
            consider(c, fc_now);
        } else {
            a = c;
            c = d;
            fc_now = fd;
            d = a + inv_phi * (b - a);
            fd = f(d);
            consider(d, fd);
        }
    }
    return out;
}

SettleDetector::SettleDetector(std::vector<SettleChannel> channels, std::size_t window)
    : channels_(std::move(channels)), window_(window) {
    if (channels_.empty()) throw std::invalid_argument("SettleDetector: no channels");
    if (window_ < 2) throw std::invalid_argument("SettleDetector: window under 2 samples");
    ring_.assign(channels_.size(), std::vector<double>(2 * window_, 0.0));
}

void SettleDetector::reset() {
    head_ = 0;
    count_ = 0;
}

void SettleDetector::push(std::span<const double> values) {
    if (values.size() != channels_.size()) {
        throw std::invalid_argument("SettleDetector::push: wrong channel count");
    }
    for (std::size_t c = 0; c < channels_.size(); ++c) ring_[c][head_] = values[c];
    head_ = (head_ + 1) % (2 * window_);
    ++count_;
}

double SettleDetector::at(std::size_t channel, std::size_t age) const {
    const std::size_t n = 2 * window_;
    return ring_[channel][(head_ + n - 1 - age) % n];
}

double SettleDetector::mean(std::size_t channel) const {
    const std::size_t n = std::min(count_, window_);
    if (n == 0) return 0.0;
    double sum = 0.0;
    for (std::size_t i = 0; i < n; ++i) sum += at(channel, i);
    return sum / static_cast<double>(n);
}

double SettleDetector::spread(std::size_t channel) const {
    const std::size_t n = std::min(count_, window_);
    if (n == 0) return 0.0;
    double lo = kInf;
    double hi = -kInf;
    for (std::size_t i = 0; i < n; ++i) {
        lo = std::min(lo, at(channel, i));
        hi = std::max(hi, at(channel, i));
    }
    return hi - lo;
}

double SettleDetector::mean_change(std::size_t channel) const {
    if (!full()) return 0.0;
    double previous = 0.0;
    for (std::size_t i = window_; i < 2 * window_; ++i) previous += at(channel, i);
    return mean(channel) - previous / static_cast<double>(window_);
}

bool SettleDetector::settled() const {
    if (!full()) return false;
    for (std::size_t c = 0; c < channels_.size(); ++c) {
        const SettleChannel& ch = channels_[c];
        const double m = mean(c);
        if (!std::isfinite(m)) return false;
        if (std::abs(mean_change(c)) > tolerance(ch, ch.mean_change, m)) return false;
        if (spread(c) > tolerance(ch, ch.spread, m)) return false;
    }
    return true;
}

SteadyResult evaluate_steady(BoatSim& boat, const SteadyRequest& request,
                             const std::optional<SailingState>& warm) {
    constexpr double dt = Simulation::kFixedDt;
    const SettleParams& sp = request.settle;
    const auto every = [](double interval) {
        return std::max<long long>(1, std::llround(interval / Simulation::kFixedDt));
    };
    const long long sample_every = every(sp.sample_interval);
    const long long check_every = every(sp.check_interval);
    const auto window = static_cast<std::size_t>(std::max<long long>(
        2, std::llround(sp.window / (static_cast<double>(sample_every) * dt))));
    const auto max_steps = std::llround(sp.max_time / dt);

    const double twa_deg = request.twa * 180.0 / std::numbers::pi;
    const auto wind = std::make_shared<const UniformWind>(request.tws, 90.0 + twa_deg);
    boat.set_wind(wind);
    boat.set_aero(true);
    boat.thrust().set_thrust(0.0);
    Controls trim = with_jib_sheets_together(request.trim);
    boat.set_controls(trim);

    Autopilot pilot(request.autopilot);
    pilot.set_target(request.twa);
    State initial;
    if (warm) {
        initial = warm->state;
        initial.position.x() = 0.0;
        initial.position.y() = 0.0;
        boat.appendages().rudder().reset(warm->rudder);
        pilot.reset(warm->autopilot_integral);
        boat.start(initial, warm->crew_y);
    } else {
        initial = boat.equilibrium().pose;
        initial.linear_velocity = Vec3(request.initial_speed, 0.0, 0.0);
        initial.angular_velocity = Vec3::Zero();
        boat.appendages().rudder().reset(0.0);
        pilot.reset(0.0);
        boat.start(initial);
    }
    Simulation& sim = boat.simulation();

    const Vec3 to = wind->sample(Vec3::Zero(), 0.0);
    const double to_norm = std::hypot(to.x(), to.y());
    const Vec3 upwind = to_norm > 0.0 ? Vec3(-to.x() / to_norm, -to.y() / to_norm, 0.0)
                                      : Vec3::Zero();

    SettleDetector detector({sp.bsp, sp.heel, sp.leeway, {}, {}, {}}, window);
    SteadyResult out;
    std::array<double, kChannels> sample{};
    bool settled = false;
    long long step = 0;
    for (; step < max_steps; ++step) {
        const State& s = sim.state();
        const Mat3 r = s.rotation();
        const double twa = true_wind_angle(s, *wind, sim.time());
        const double yaw_rate = (r * s.angular_velocity).z();
        trim.tiller = pilot.update(dt, twa, yaw_rate);
        boat.appendages().set_controls(trim);
        boat.step();
        const State& n = sim.state();
        if (!finite(n) || !std::isfinite(boat.appendages().rudder_angle())) {
            out.finite = false;
            ++step;
            break;
        }
        if ((step + 1) % sample_every == 0) {
            const Mat3 rn = n.rotation();
            const Vec3 v = n.world_linear_velocity();
            const double heading = std::atan2(rn(1, 0), rn(0, 0));
            const double along = v.x() * std::cos(heading) + v.y() * std::sin(heading);
            const double across = -v.x() * std::sin(heading) + v.y() * std::cos(heading);
            const double speed = std::hypot(v.x(), v.y());
            sample[kBsp] = speed;
            sample[kHeel] = heel_angle(rn);
            sample[kLeeway] = speed > 1e-6 ? std::atan2(across, along) : 0.0;
            sample[kVmg] = v.dot(upwind);
            sample[kRudder] = boat.appendages().rudder_angle();
            sample[kTwa] = true_wind_angle(n, *wind, sim.time());
            detector.push(sample);
        }
        if ((step + 1) % check_every == 0 && detector.settled()) {
            settled = true;
            ++step;
            break;
        }
    }
    out.time = static_cast<double>(step) * dt;
    out.window_bsp = detector.mean(kBsp);
    out.window_vmg = detector.mean(kVmg);
    out.window_bsp_spread = detector.spread(kBsp);
    out.window_heel_spread = detector.spread(kHeel);
    out.window_leeway_spread = detector.spread(kLeeway);
    out.settled = settled && out.finite;
    if (out.settled) {
        out.bsp = detector.mean(kBsp);
        out.vmg = detector.mean(kVmg);
        out.heel = detector.mean(kHeel);
        out.leeway = detector.mean(kLeeway);
        out.rudder = detector.mean(kRudder);
        out.twa = detector.mean(kTwa);
    }
    out.final.state = sim.state();
    out.final.crew_y = boat.mass_model().crew_position().y();
    out.final.rudder = boat.appendages().rudder_angle();
    out.final.autopilot_integral = pilot.integral();
    return out;
}

double trim_objective(const SteadyResult& r, TrimObjective objective) {
    if (!r.finite) return -kInf;
    const double bsp = r.settled ? r.bsp : r.window_bsp;
    const double vmg = r.settled ? r.vmg : r.window_vmg;
    double v = bsp;
    if (objective == TrimObjective::vmg_up) v = vmg;
    if (objective == TrimObjective::vmg_down) v = -vmg;
    if (!std::isfinite(v)) return -kInf;
    return r.settled ? v : v - kUnsettledPenalty;
}

Controls default_trim(double twa) {
    constexpr std::array<std::array<double, 2>, 4> kSheet{{
        {40.0, 0.92},
        {90.0, 0.7},
        {150.0, 0.1},
        {180.0, 0.0},
    }};
    const double a = std::clamp(std::abs(twa) * 180.0 / std::numbers::pi, kSheet.front()[0],
                                kSheet.back()[0]);
    double sheet = kSheet.back()[1];
    for (std::size_t i = 1; i < kSheet.size(); ++i) {
        if (a <= kSheet[i][0]) {
            const double u = (a - kSheet[i - 1][0]) / (kSheet[i][0] - kSheet[i - 1][0]);
            sheet = kSheet[i - 1][1] + u * (kSheet[i][1] - kSheet[i - 1][1]);
            break;
        }
    }
    Controls c;
    c.mainsheet = sheet;
    c.jib_sheet_port = sheet;
    c.jib_sheet_starboard = sheet;
    c.traveler = 0.0;
    c.backstay = 0.0;
    c.jib_car = 0.5;
    c.reef = 0.0;
    return c;
}

TrimResult auto_trim(BoatSim& boat, const SteadyRequest& base, const TrimParams& params,
                     const std::optional<SailingState>& warm) {
    const auto wall_start = std::chrono::steady_clock::now();
    TrimResult out;
    std::optional<SailingState> warm_state = warm;
    SteadyRequest request = base;

    auto evaluate = [&](const Controls& c) {
        request.trim = with_jib_sheets_together(c);
        const SteadyResult r = evaluate_steady(boat, request, warm_state);
        ++out.evaluations;
        out.sim_time += r.time;
        if (r.settled) warm_state = r.final;
        return r;
    };
    auto budget_left = [&] { return out.evaluations < params.max_evaluations; };

    Controls current = with_jib_sheets_together(base.trim);
    current.reef = clamped(current).reef;
    SteadyResult current_result = evaluate(current);
    double current_f = trim_objective(current_result, params.objective);

    for (std::size_t pass = 0; pass < params.passes.size() && budget_left(); ++pass) {
        const TrimPass& tp = params.passes[pass];
        for (const Coordinate& coord : kCoordinates) {
            if (!budget_left()) break;
            const double range = coord.hi - coord.lo;
            const double x0 = current.*coord.member;
            const double lo = std::max(coord.lo, x0 - tp.half_width * range);
            const double hi = std::min(coord.hi, x0 + tp.half_width * range);
            std::optional<SteadyResult> best_result;
            double best_f = -kInf;
            const auto f = [&](double x) {
                Controls c = current;
                c.*coord.member = x;
                SteadyResult r = evaluate(c);
                const double v = trim_objective(r, params.objective);
                if (!best_result || v > best_f) {
                    best_f = v;
                    best_result = std::move(r);
                }
                return v;
            };
            const int left = params.max_evaluations - out.evaluations;
            const GoldenResult g = golden_section_max(f, lo, hi, tp.tolerance * range, left);
            if (g.evaluations > 0 && g.f > current_f) {
                current.*coord.member = g.x;
                current = with_jib_sheets_together(current);
                current_f = g.f;
                current_result = *best_result;
            }
        }
        if (pass == 0 && params.search_reef && budget_left()) {
            Controls c = current;
            c.reef = current.reef < 0.5 ? 1.0 : 0.0;
            const SteadyResult r = evaluate(c);
            const double v = trim_objective(r, params.objective);
            if (v > current_f) {
                current = c;
                current_f = v;
                current_result = r;
            }
        }
    }

    out.trim = current;
    out.steady = current_result;
    out.objective = current_f;
    out.settled = current_result.settled;
    out.wall_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    return out;
}

const char* polar_kind_name(PolarKind kind) {
    switch (kind) {
        case PolarKind::beat:
            return "beat";
        case PolarKind::run:
            return "run";
        case PolarKind::fixed:
            break;
    }
    return "fixed";
}

PolarPointResult solve_polar_point(BoatSim& boat, const PolarPointRequest& request) {
    const auto wall_start = std::chrono::steady_clock::now();
    PolarPointResult out;
    out.tws = request.tws;
    out.kind = request.kind;
    const TrimObjective objective = objective_for(request.kind);

    SteadyRequest base;
    base.tws = request.tws;
    base.autopilot = request.autopilot;
    base.settle = request.settle;

    TrimParams first = request.trim;
    first.objective = objective;
    TrimParams later = request.refine;
    later.objective = objective;

    auto account = [&](const TrimResult& t) {
        out.evaluations += t.evaluations;
        out.sim_time += t.sim_time;
    };

    if (request.kind == PolarKind::fixed) {
        base.twa = request.twa;
        base.trim = request.initial_trim.value_or(default_trim(request.twa));
        out.best = auto_trim(boat, base, first);
        account(out.best);
        out.twa = request.twa;
    } else {
        std::optional<TrimResult> best;
        std::optional<SailingState> warm;
        std::optional<Controls> start_trim = request.initial_trim;
        double best_twa = request.twa_lo;
        int trims = 0;
        golden_section_max(
            [&](double twa) {
                const bool first_trim = !best;
                const double prior = best ? best->objective : -kInf;
                base.twa = twa;
                base.trim = start_trim.value_or(default_trim(twa));
                TrimResult t = auto_trim(boat, base, trims == 0 ? first : later, warm);
                ++trims;
                account(t);
                if (t.settled) warm = t.steady.final;
                const double v = t.objective;
                if (first_trim || v > prior) {
                    best_twa = twa;
                    start_trim = t.trim;
                    best = std::move(t);
                }
                return v;
            },
            request.twa_lo, request.twa_hi, request.twa_tolerance,
            std::max(1, request.max_twa_evaluations));
        out.best = std::move(*best);
        out.twa = best_twa;
    }
    out.wall_time =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    return out;
}

}  // namespace sail::harness
