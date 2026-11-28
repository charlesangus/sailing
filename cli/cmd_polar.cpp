#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "common.hpp"
#include "sail/boat_sim.hpp"
#include "sail/harness/autotrim.hpp"

namespace sailcli {

namespace {

using sail::harness::PolarKind;
using sail::harness::PolarPointRequest;
using sail::harness::PolarPointResult;

constexpr const char* kUsage =
    "usage: sailsim polar --boat DIR --tws KN[,KN...] [--twa DEG[,DEG...]] [--beat] [--run]\n"
    "                     [--threads N] [--out FILE] [--max-evals N] [--max-time S]\n"
    "                     [--kp K] [--ki K] [--kd K] [--model orc|dsyhs] [--section FILE]\n"
    "\n"
    "Steady polar of the sailing boat in flat water, by the full dynamic simulation: an autopilot\n"
    "holds each course true wind angle on starboard tack with the crew hiking, and an auto-trimmer\n"
    "searches mainsheet, jib sheet, traveler, jib car, backstay and reef for the best steady\n"
    "boat speed. Each evaluation runs until boat speed, heel and leeway settle in rolling windows;\n"
    "a point that never settles (drift, oscillation or limit cycle) is reported with settled 0\n"
    "and nan values. Prints CSV, one row per point, in the order: for each --tws, the beat, each\n"
    "--twa as given, then the run.\n"
    "\n"
    "  --boat DIR      boat dossier directory, e.g. data/boats/j80\n"
    "  --tws LIST      true wind speeds at 10 m, knots, with the ORC log wind profile below\n"
    "  --twa LIST      course true wind angles, degrees, 1..180, trimmed for boat speed (kind fixed)\n"
    "  --beat          add the TWA of best upwind VMG per TWS, searched over 30..60 degrees by golden section\n"
    "                  plus a final parabolic-vertex step, 7 trims\n"
    "  --run           add the TWA of best downwind VMG per TWS, searched over 130..178 degrees\n"
    "  --threads N     points solved in parallel, default 1; the output does not depend on it\n"
    "  --out FILE      write the CSV to FILE instead of stdout\n"
    "  --max-evals N   steady evaluations per auto-trim, default 64\n"
    "  --max-time S    simulated seconds per evaluation before it counts as unsettled, default 150\n"
    "  --kp, --ki, --kd  autopilot gains, tiller per rad, per rad*s and per rad/s of yaw rate\n"
    "  --model NAME    canoe-body resistance model, orc or dsyhs; default the dossier's\n"
    "  --section FILE  keel and rudder section table, default DIR/../../foils/naca0012.csv\n"
    "\n"
    "Every TWA is the course's, between the true wind and the boat's track through the water\n"
    "(heading plus leeway), as the ORC and Python-VPP polars tabulate it.\n"
    "\n"
    "Columns: tws_kn, twa_deg (the course TWA held), twa_heading_deg (the heading's, settled\n"
    "mean), bsp_kn, vmg_kn (bsp along the true wind, negative downwind), heel_deg (negative to\n"
    "port), leeway_deg (positive to leeward on starboard tack), rudder_deg, settled, the trim\n"
    "(mainsheet, traveler, backstay, jib_sheet, jib_car, reef), evaluations, sim_time_s, and kind\n"
    "as orc_polar.csv.\n"
    "Wall times go to stderr. The exit status is 3 when any point did not settle.\n";

struct Flags {
    bool beat = false;
    bool run = false;
};

Flags g_flags;

std::vector<double> number_list(const Options& options, std::string_view name) {
    std::vector<double> out;
    const auto text = options.get(name);
    if (!text) return out;
    std::string_view rest = *text;
    while (!rest.empty()) {
        const auto comma = rest.find(',');
        const std::string_view item = rest.substr(0, comma);
        double v = 0.0;
        const auto [ptr, ec] = std::from_chars(item.data(), item.data() + item.size(), v);
        if (item.empty() || ec != std::errc() || ptr != item.data() + item.size() ||
            !std::isfinite(v)) {
            throw UsageError("option '" + std::string(name) + "' expects numbers, got '" + *text +
                             "'");
        }
        out.push_back(v);
        if (comma == std::string_view::npos) break;
        rest = rest.substr(comma + 1);
    }
    return out;
}

int whole_number(const Options& options, std::string_view name, int fallback, int lo, int hi) {
    const double v = options.number(name, fallback);
    if (v != std::floor(v) || v < lo || v > hi) {
        throw UsageError("option '" + std::string(name) + "' must be a whole number in [" +
                         std::to_string(lo) + ", " + std::to_string(hi) + "]");
    }
    return static_cast<int>(v);
}

void write_row(std::FILE* out, const PolarPointResult& p) {
    const sail::harness::SteadyResult& s = p.best.steady;
    const sail::Controls& t = p.best.trim;
    std::fprintf(out,
                 "%.6g,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.0f,%d,%.2f,"
                 "%s\n",
                 p.tws / kKnot, deg(p.twa), deg(s.heading_twa), s.bsp / kKnot, s.vmg / kKnot,
                 deg(s.heel),
                 deg(s.leeway), deg(s.rudder), s.settled ? 1 : 0, t.mainsheet, t.traveler,
                 t.backstay, t.jib_sheet_port, t.jib_car, t.reef, p.evaluations, p.sim_time,
                 sail::harness::polar_kind_name(p.kind));
}

int polar(const Options& options) {
    const std::vector<double> tws = number_list(options, "--tws");
    const std::vector<double> twa = number_list(options, "--twa");
    if (tws.empty()) throw UsageError("--tws is required");
    for (double v : tws) {
        if (!(v > 0.0 && v <= 40.0)) throw UsageError("--tws values must be in (0, 40] kn");
    }
    for (double v : twa) {
        if (!(v >= 1.0 && v <= 180.0)) throw UsageError("--twa values must be in [1, 180] deg");
    }
    if (twa.empty() && !g_flags.beat && !g_flags.run) {
        throw UsageError("give --twa, --beat or --run");
    }
    const int threads = whole_number(options, "--threads", 1, 1, 256);
    const int max_evals = whole_number(options, "--max-evals", 64, 1, 100000);
    const double max_time = options.number("--max-time", 150.0);
    if (!(max_time >= 20.0 && max_time <= 3600.0)) {
        throw UsageError("--max-time must be in [20, 3600] s");
    }

    PolarPointRequest proto;
    proto.trim.max_evaluations = max_evals;
    proto.settle.max_time = max_time;
    proto.autopilot.kp = options.number("--kp", proto.autopilot.kp);
    proto.autopilot.ki = options.number("--ki", proto.autopilot.ki);
    proto.autopilot.kd = options.number("--kd", proto.autopilot.kd);

    std::vector<PolarPointRequest> points;
    for (double w : tws) {
        PolarPointRequest r = proto;
        r.tws = w * kKnot;
        if (g_flags.beat) {
            r.kind = PolarKind::beat;
            r.twa_lo = rad(30.0);
            r.twa_hi = rad(60.0);
            points.push_back(r);
        }
        for (double a : twa) {
            r.kind = PolarKind::fixed;
            r.twa = rad(a);
            points.push_back(r);
        }
        if (g_flags.run) {
            r.kind = PolarKind::run;
            r.twa_lo = rad(130.0);
            r.twa_hi = rad(178.0);
            points.push_back(r);
        }
    }

    sail::BoatSimOptions sim_options;
    sim_options.resistance = parse_resistance_model(options.get("--model"));
    std::unique_ptr<sail::BoatSim> first_boat = load_boat_sim(options, sim_options);

    std::FILE* out = stdout;
    std::unique_ptr<std::FILE, int (*)(std::FILE*)> file(nullptr, std::fclose);
    if (const auto path = options.get("--out")) {
        file.reset(std::fopen(path->c_str(), "w"));
        if (!file) throw std::runtime_error("cannot write '" + *path + "'");
        out = file.get();
    }

    const auto wall_start = std::chrono::steady_clock::now();
    std::vector<std::optional<PolarPointResult>> results(points.size());
    std::atomic<std::size_t> next{0};
    std::mutex log_mutex;
    std::exception_ptr failure;
    auto worker = [&](std::unique_ptr<sail::BoatSim> boat) {
        try {
            if (!boat) boat = load_boat_sim(options, sim_options);
            for (std::size_t i = next++; i < points.size(); i = next++) {
                PolarPointResult r = sail::harness::solve_polar_point(*boat, points[i]);
                {
                    const std::lock_guard lock(log_mutex);
                    std::fprintf(stderr,
                                 "polar: tws %.4g kn twa %.2f deg %s: bsp %.3f kn, settled %d, "
                                 "%d evaluations, %.0f s simulated, %.2f s wall\n",
                                 r.tws / kKnot, deg(r.twa), sail::harness::polar_kind_name(r.kind),
                                 r.best.steady.bsp / kKnot, r.best.settled ? 1 : 0, r.evaluations,
                                 r.sim_time, r.wall_time);
                }
                results[i] = std::move(r);
            }
        } catch (...) {
            const std::lock_guard lock(log_mutex);
            if (!failure) failure = std::current_exception();
            next = points.size();
        }
    };
    const int n = std::min<int>(threads, static_cast<int>(points.size()));
    std::vector<std::thread> pool;
    for (int i = 1; i < n; ++i) pool.emplace_back(worker, nullptr);
    worker(std::move(first_boat));
    for (std::thread& t : pool) t.join();
    if (failure) std::rethrow_exception(failure);

    std::fputs(
        "tws_kn,twa_deg,twa_heading_deg,bsp_kn,vmg_kn,heel_deg,leeway_deg,rudder_deg,settled,"
        "mainsheet,traveler,backstay,jib_sheet,jib_car,reef,evaluations,sim_time_s,kind\n",
        out);
    bool all_settled = true;
    for (const auto& r : results) {
        write_row(out, *r);
        all_settled = all_settled && r->best.settled;
    }
    std::fflush(out);
    const double wall =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    std::fprintf(stderr, "polar: %zu points, %d threads, %.2f s wall\n", points.size(), n, wall);
    return all_settled ? 0 : 3;
}

}  // namespace

int run_polar(const Args& args) {
    g_flags = {};
    Args rest;
    for (std::string_view a : args) {
        if (a == "--beat") {
            g_flags.beat = true;
        } else if (a == "--run") {
            g_flags.run = true;
        } else {
            rest.push_back(a);
        }
    }
    return guarded("polar", kUsage, polar, rest,
                   {"--boat", "--tws", "--twa", "--threads", "--out", "--max-evals", "--max-time",
                    "--kp", "--ki", "--kd", "--model", "--section"});
}

}  // namespace sailcli
