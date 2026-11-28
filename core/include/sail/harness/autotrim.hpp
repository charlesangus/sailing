#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <vector>

#include "sail/boat_sim.hpp"
#include "sail/controls.hpp"
#include "sail/harness/autopilot.hpp"
#include "sail/rigid_body.hpp"

namespace sail::harness {

struct GoldenResult {
    // The best point evaluated and its value.
    double x = 0.0;
    double f = -std::numeric_limits<double>::infinity();
    int evaluations = 0;
};

// Maximises `f`, taken as unimodal, over [lo, hi] by golden-section search. Only interior points
// are evaluated; it stops once the bracket is narrower than `tolerance` or after
// `max_evaluations` evaluations, and returns the best point evaluated. With lo >= hi or
// max_evaluations < 1 it evaluates nothing and returns x = lo.
GoldenResult golden_section_max(const std::function<double(double)>& f, double lo, double hi,
                                double tolerance, int max_evaluations);

// golden_section_max on all but one of `max_evaluations`, then one evaluation at the vertex of the
// parabola through the best point and its neighbours in x, skipped when that vertex is within
// `min_step` of an evaluated point or the three points are not concave. A flat-topped objective
// leaves the golden bracket wide; the vertex resolves the peak inside it. Returns the best point
// evaluated. Deterministic for a deterministic `f`.
GoldenResult parabolic_peak_max(const std::function<double(double)>& f, double lo, double hi,
                                double tolerance, int max_evaluations, double min_step);

// Settle tolerances of one channel, each `absolute + relative * |mean|` of the latest window.
// An infinite tolerance does not check.
struct SettleChannel {
    // Largest change between the means of the two latest windows.
    double mean_change = std::numeric_limits<double>::infinity();
    // Largest max - min within the latest window.
    double spread = std::numeric_limits<double>::infinity();
    // Largest |tail()|: how far the latest window's mean may still be from where the approach
    // ends.
    double residual = std::numeric_limits<double>::infinity();
    double relative = 0.0;
};

// Rolling windows over equally spaced samples of several channels. Settled once two full
// windows have been seen and, in every channel, the latest window's spread, the change of its
// mean from the window before and the tail still to come are within tolerance: a drift fails the
// mean test, an oscillation or limit cycle the spread test, and a slow exponential approach,
// whose window-to-window change can be small while it is still far from its end, the tail
// test, so none is averaged into a steady value.
class SettleDetector {
public:
    // `max_tail_ratio` in [0, 1) caps the per-window decay ratio tail() assumes, so the slowest
    // approach it models has the time constant -window / ln(max_tail_ratio). Throws
    // std::invalid_argument on no channels, a window of fewer than 2 samples or a ratio outside
    // [0, 1).
    SettleDetector(std::vector<SettleChannel> channels, std::size_t window,
                   double max_tail_ratio = 0.9);

    void reset();
    // One sample of every channel, in the constructor's order. Throws std::invalid_argument on
    // the wrong count.
    void push(std::span<const double> values);

    std::size_t channel_count() const { return channels_.size(); }
    std::size_t window() const { return window_; }
    // Two windows seen.
    bool full() const { return count_ >= 2 * window_; }
    bool settled() const;
    // Over the latest window (all samples so far while fewer than a window).
    double mean(std::size_t channel) const;
    double spread(std::size_t channel) const;
    // Latest window's mean less the previous window's; 0 until full().
    double mean_change(std::size_t channel) const;
    // The change still to come from the latest window's mean to the end of an exponential
    // approach through the samples, 0 until full(). With h half a window, the decay ratio over
    // one window is rho = d_new / d_old, d_new the latest h samples' mean less the h before and
    // d_old the same one window earlier; the tail is then mean_change * rho / (1 - rho), rho
    // capped at max_tail_ratio, and taken at the cap when d_old is 0. When d_new and d_old differ in sign
    // the samples are not a monotonic approach and the tail is 0, leaving them to the mean and
    // spread tests.
    double tail(std::size_t channel) const;

private:
    double at(std::size_t channel, std::size_t age) const;
    // Mean of `count` samples from age `from` back.
    double block_mean(std::size_t channel, std::size_t from, std::size_t count) const;

    std::vector<SettleChannel> channels_;
    std::size_t window_;
    double max_tail_ratio_;
    std::vector<std::vector<double>> ring_;
    std::size_t head_ = 0;
    std::size_t count_ = 0;
};

struct SettleParams {
    // s
    double window = 6.0;
    double sample_interval = 0.05;
    double check_interval = 0.5;
    double max_time = 150.0;
    // The slowest approach SettleDetector::tail() models, as its time constant.
    double max_time_constant = 60.0;
    // m/s
    SettleChannel bsp{0.004, 0.03, 0.004, 0.002};
    // rad
    SettleChannel heel{0.0015, 0.01, 0.0015, 0.0};
    SettleChannel leeway{0.0015, 0.01, 0.0015, 0.0};
};

// What a run hands the next one to start from.
struct SailingState {
    // Heading near world +x as the wind is laid out from it.
    State state;
    // m, hull-frame lateral crew position.
    double crew_y = 0.0;
    // rad
    double rudder = 0.0;
    double autopilot_integral = 0.0;
};

// Steady sailing in flat water: a LogProfileWind of `tws` at 10 m laid out so that a course along
// world +x has true wind angle `twa`, the autopilot holding `twa` as course_true_wind_angle(),
// the sails at `trim` (its tiller is ignored) and the crew controller hiking.
struct SteadyRequest {
    // m/s
    double tws = 0.0;
    // rad, positive starboard tack: the course's, heading plus leeway, as the ORC and Python-VPP
    // polars tabulate it.
    double twa = 0.0;
    Controls trim;
    AutopilotParams autopilot;
    SettleParams settle;
    // m/s along the heading for a cold start, upright at the still-water equilibrium.
    double initial_speed = 3.0;
};

struct SteadyResult {
    bool settled = false;
    // False when a state or rudder value went non-finite; the run stops there.
    bool finite = true;
    // Means over the latest window when settled, NaN otherwise. Speeds in m/s, angles in rad.
    // bsp is the CG's horizontal speed; vmg its component toward the true wind's origin,
    // negative downwind; heel positive starboard down; leeway the velocity's angle off the
    // heading, positive to port; rudder positive with the tiller to port; twa the achieved
    // course TWA and heading_twa the heading's, twa less leeway.
    double bsp = std::numeric_limits<double>::quiet_NaN();
    double vmg = std::numeric_limits<double>::quiet_NaN();
    double heel = std::numeric_limits<double>::quiet_NaN();
    double leeway = std::numeric_limits<double>::quiet_NaN();
    double rudder = std::numeric_limits<double>::quiet_NaN();
    double twa = std::numeric_limits<double>::quiet_NaN();
    double heading_twa = std::numeric_limits<double>::quiet_NaN();
    // The latest window's mean and spread of bsp, heel and leeway whether settled or not, for
    // diagnosis only.
    double window_bsp = 0.0;
    double window_vmg = 0.0;
    double window_bsp_spread = 0.0;
    double window_heel_spread = 0.0;
    double window_leeway_spread = 0.0;
    // s simulated.
    double time = 0.0;
    // The last state, to warm-start the next run.
    SailingState final;
};

// Runs `boat` with aero on from `warm` (its position moved to the origin) or, without it, from
// a cold start, until settled or settle.max_time. It first calls BoatSim::reset(), so the
// result depends on the boat data, request and warm state alone, never on what the boat ran
// before; a cold start is upright at the still-water equilibrium of the centreline crew.
SteadyResult evaluate_steady(BoatSim& boat, const SteadyRequest& request,
                             const std::optional<SailingState>& warm = std::nullopt);

enum class TrimObjective {
    bsp,
    // VMG toward the wind, and away from it.
    vmg_up,
    vmg_down,
};

// Taken from an unsettled run's objective: below any settled run's, so a settled run always
// wins, while unsettled runs still rank among themselves and a search can climb out of them.
inline constexpr double kUnsettledPenalty = 1000.0;

// The value auto_trim() maximises: bsp, vmg or -vmg when settled; the latest window's value less
// kUnsettledPenalty when not; -infinity when not finite.
double trim_objective(const SteadyResult& result, TrimObjective objective);

// A starting trim for `twa` (rad, either tack): sheets from nearly two-blocked close-hauled to
// eased square downwind, everything else at its middle or off.
Controls default_trim(double twa);

// One pass over the controls: a golden-section search on each in turn, over the control's value
// +- half_width times its range (clipped to the range), to `tolerance` times its range.
struct TrimPass {
    double half_width = 1.0;
    double tolerance = 0.1;
};

struct TrimParams {
    TrimObjective objective = TrimObjective::bsp;
    // Coarse to fine.
    std::vector<TrimPass> passes{{1.0, 0.15}, {0.15, 0.04}};
    // Try the other reef setting at the end of the first pass, and keep it if better.
    bool search_reef = true;
    // Steady evaluations, the starting one included.
    int max_evaluations = 64;
};

struct TrimResult {
    // The best trim found; jib_sheet_port and jib_sheet_starboard are equal.
    Controls trim;
    // Its evaluation.
    SteadyResult steady;
    double objective = -std::numeric_limits<double>::infinity();
    bool settled = false;
    int evaluations = 0;
    // Simulated s over every evaluation, and wall-clock s.
    double sim_time = 0.0;
    double wall_time = 0.0;
};

// Coordinate-wise golden-section search maximising the objective over mainsheet, the jib sheet
// (both sides together), traveler, jib car and backstay, in that order, each pass in turn, with
// the reef tried both ways. Each evaluation is evaluate_steady() from `base` with the trim
// replaced, warm-started from the latest settled evaluation (`warm` before any). A search that
// finds nothing better keeps the control where it was. Stops at the budget.
TrimResult auto_trim(BoatSim& boat, const SteadyRequest& base, const TrimParams& params,
                     const std::optional<SailingState>& warm = std::nullopt);

enum class PolarKind {
    fixed,
    // The TWA of best VMG upwind, and downwind.
    beat,
    run,
};

const char* polar_kind_name(PolarKind kind);

struct PolarPointRequest {
    double tws = 0.0;
    PolarKind kind = PolarKind::fixed;
    // rad, the fixed point's TWA.
    double twa = 0.0;
    // rad: the beat or run search's TWA range and stop width, and its budget of trims.
    double twa_lo = 0.0;
    double twa_hi = 0.0;
    double twa_tolerance = 0.035;
    // Includes the final evaluation at the fitted parabola's vertex.
    int max_twa_evaluations = 7;
    // The trim from which the search starts, default_trim() of the first TWA tried when empty;
    // the tiller is ignored.
    std::optional<Controls> initial_trim;
    // The first auto-trim, and those at the beat or run search's later angles, which start from
    // the best trim so far. Their objective is replaced by the kind's.
    TrimParams trim;
    TrimParams refine{TrimObjective::bsp, {{0.1, 0.05}}, false, 22};
    AutopilotParams autopilot;
    SettleParams settle;
};

struct PolarPointResult {
    double tws = 0.0;
    PolarKind kind = PolarKind::fixed;
    // rad: the fixed TWA, or the best one found.
    double twa = 0.0;
    TrimResult best;
    // Over every auto-trim.
    int evaluations = 0;
    double sim_time = 0.0;
    double wall_time = 0.0;
};

// Deterministic for the same boat data and request.
PolarPointResult solve_polar_point(BoatSim& boat, const PolarPointRequest& request);

}  // namespace sail::harness
