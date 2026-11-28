#include "sail/hull_resistance.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <numbers>
#include <string>
#include <utility>

#include "dossier_json.hpp"
#include "sail/boat_data.hpp"
#include "sail/gravity.hpp"

namespace sail {

namespace {

using dossier::Document;
using dossier::Node;
using dossier::read_document;

std::vector<std::string> split_csv(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool quoted = false;
    for (const char c : line) {
        if (c == '"') {
            quoted = !quoted;
        } else if (c == ',' && !quoted) {
            fields.push_back(field);
            field.clear();
        } else if (c != '\r') {
            field.push_back(c);
        }
    }
    fields.push_back(field);
    return fields;
}

std::optional<double> parse_number(const std::string& text) {
    if (text.empty()) return std::nullopt;
    char* end = nullptr;
    const double v = std::strtod(text.c_str(), &end);
    if (end != text.c_str() + text.size() || !std::isfinite(v)) return std::nullopt;
    return v;
}

bool strictly_ascending(const std::vector<double>& axis) {
    return std::adjacent_find(axis.begin(), axis.end(),
                              [](double a, double b) { return !(a < b); }) == axis.end();
}

struct Cell {
    std::size_t i = 0;
    double t = 0.0;
};

// x must already lie within the axis, or beyond its last point to extrapolate.
Cell locate(const std::vector<double>& axis, double x) {
    const auto it = std::upper_bound(axis.begin() + 1, axis.end() - 1, x);
    const std::size_t i = static_cast<std::size_t>(it - axis.begin()) - 1;
    return {i, (x - axis[i]) / (axis[i + 1] - axis[i])};
}

double table_at(const std::vector<double>& x, const std::vector<double>& y, double at) {
    if (x.size() == 1) return y.front();
    const double clamped = std::clamp(at, x.front(), x.back());
    const Cell c = locate(x, clamped);
    return y[c.i] + c.t * (y[c.i + 1] - y[c.i]);
}

void optional_positive(const Node& block, const std::string& key, std::optional<double>& out) {
    if (block.has(key)) out = block.at(key).positive_value();
}

std::vector<double> positive_array(const Node& node) {
    std::vector<double> out(node.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = node[i].number();
        if (!(out[i] > 0.0)) node[i].fail("expected a positive number");
    }
    return out;
}

OrcHullOverrides read_overrides(const Node& orc) {
    OrcHullOverrides o;
    optional_positive(orc, "lsm1", o.lsm1);
    optional_positive(orc, "lsm4", o.lsm4);
    optional_positive(orc, "lsm6", o.lsm6);
    optional_positive(orc, "btr_effective", o.btr_effective);
    if ((o.lsm4.has_value() || o.lsm6.has_value()) &&
        !(o.lsm1.has_value() && o.lsm4.has_value() && o.lsm6.has_value())) {
        orc.fail("lsm4 and lsm6 need lsm1, lsm4 and lsm6 together");
    }
    if (orc.has("heeled")) {
        const Node heeled = orc.at("heeled");
        const Node heel = heeled.at("heel_deg");
        if (heel.size() == 0) heel.fail("expected at least one heel angle");
        o.heel_deg.resize(heel.size());
        for (std::size_t i = 0; i < o.heel_deg.size(); ++i) o.heel_deg[i] = heel[i].number();
        if (!strictly_ascending(o.heel_deg)) heel.fail("expected strictly ascending angles");
        const auto column = [&](const char* key, std::vector<double>& out) {
            if (!heeled.has(key)) return;
            const Node node = heeled.at(key);
            if (node.size() != o.heel_deg.size()) node.fail("expected one value per heel_deg");
            out = positive_array(node);
        };
        column("wetted_area", o.wetted_area);
        column("btr", o.btr);
        column("lvr", o.lvr);
    }
    return o;
}

}  // namespace

OrcDragSurfaces OrcDragSurfaces::load(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw BoatDataError(file, "cannot open file");
    const auto fail = [&](std::size_t line, const std::string& message) {
        throw BoatDataError(file, "line " + std::to_string(line) + ": " + message);
    };

    OrcDragSurfaces s;
    s.fn_.push_back(0.0);
    std::vector<double> block_lvr;
    std::vector<double> block_btr;
    std::vector<double> block_values;
    bool in_block = false;
    std::size_t blocks = 0;

    const auto finish_block = [&](std::size_t line) {
        if (!in_block) return;
        if (block_btr.size() < 2 || block_lvr.size() < 2) fail(line, "expected a 2x2 grid at least");
        if (blocks == 0) {
            s.lvr_ = block_lvr;
            s.btr_ = block_btr;
            s.values_.assign(s.lvr_.size() * s.btr_.size(), 0.0);
        } else if (block_lvr != s.lvr_ || block_btr != s.btr_) {
            fail(line, "LVR or BTR axis differs from the first Froude number's");
        }
        s.values_.insert(s.values_.end(), block_values.begin(), block_values.end());
        ++blocks;
        block_lvr.clear();
        block_btr.clear();
        block_values.clear();
        in_block = false;
    };

    std::string text;
    std::size_t line = 0;
    while (std::getline(in, text)) {
        ++line;
        if (line == 1) continue;
        const std::vector<std::string> f = split_csv(text);
        if (std::all_of(f.begin(), f.end(), [](const std::string& x) { return x.empty(); })) {
            continue;
        }
        if (f.size() > 2 && f[1] == "Fn=") {
            finish_block(line);
            const auto fn = parse_number(f[2]);
            if (!fn || !(*fn > 0.0)) fail(line, "expected a positive Froude number");
            s.fn_.push_back(*fn);
            in_block = true;
            continue;
        }
        if (!in_block) fail(line, "expected a Froude number header");
        if (block_btr.empty()) {
            if (!f[0].empty()) fail(line, "expected the BTR row");
            for (std::size_t j = 1; j < f.size() && !f[j].empty(); ++j) {
                const auto b = parse_number(f[j]);
                if (!b) fail(line, "malformed BTR value");
                block_btr.push_back(*b);
            }
            continue;
        }
        const auto lvr = parse_number(f[0]);
        if (!lvr) fail(line, "malformed LVR value");
        if (f.size() < block_btr.size() + 1) fail(line, "expected one value per BTR");
        block_lvr.push_back(*lvr);
        for (std::size_t j = 1; j <= block_btr.size(); ++j) {
            const auto value = parse_number(f[j]);
            if (!value) fail(line, "malformed surface value");
            block_values.push_back(*value);
        }
    }
    finish_block(line);
    if (blocks < 1) throw BoatDataError(file, "no Froude number blocks");
    if (!strictly_ascending(s.fn_) || !strictly_ascending(s.lvr_) || !strictly_ascending(s.btr_)) {
        throw BoatDataError(file, "Fn, LVR and BTR axes must be strictly ascending");
    }
    return s;
}

OrcDragSurfaces::Lookup OrcDragSurfaces::lookup(double fn, double lvr, double btr) const {
    Lookup out;
    fn = std::max(fn, 0.0);
    out.fn_extrapolated = fn > fn_.back();
    out.lvr_clamped = lvr < lvr_.front() || lvr > lvr_.back();
    out.btr_clamped = btr < btr_.front() || btr > btr_.back();
    // ORC 2023 section 6.3.1: outside the LVR and BTR ranges "the value defaults to that of the
    // closest point of the surface", while Fn outside the range is extrapolated.
    const Cell cf = locate(fn_, fn);
    const Cell cl = locate(lvr_, std::clamp(lvr, lvr_.front(), lvr_.back()));
    const Cell cb = locate(btr_, std::clamp(btr, btr_.front(), btr_.back()));

    const auto plane = [&](std::size_t i_fn) {
        const double lo = at(i_fn, cl.i, cb.i) + cb.t * (at(i_fn, cl.i, cb.i + 1) - at(i_fn, cl.i, cb.i));
        const double hi = at(i_fn, cl.i + 1, cb.i) +
                          cb.t * (at(i_fn, cl.i + 1, cb.i + 1) - at(i_fn, cl.i + 1, cb.i));
        return lo + cl.t * (hi - lo);
    };
    const double a = plane(cf.i);
    const double b = plane(cf.i + 1);
    out.value = a + cf.t * (b - a);
    return out;
}

OrcHullResistance::OrcHullResistance(OrcHullParameters params, OrcDragSurfaces surfaces)
    : params_(std::move(params)), surfaces_(std::move(surfaces)) {}

double OrcHullResistance::dynamic_length(double speed_mps) const {
    const OrcHullOverrides& o = params_.overrides;
    const double l1 = o.lsm1.value_or(params_.lwl);
    if (!(o.lsm4 && o.lsm6)) return l1;
    const double l4 = *o.lsm4;
    const double l6 = *o.lsm6;
    const double v = std::abs(speed_mps);
    const auto mix = [&](double fn) {
        if (fn <= 0.125) return l1;
        if (fn <= 0.3) return l1 + (fn - 0.125) / 0.175 * (l6 - l1);
        if (fn <= 0.4) return l6 + (fn - 0.3) / 0.1 * (l4 - l6);
        const double share1 = 0.3 * std::min(1.0, (fn - 0.4) / 0.1);
        return (1.0 - share1) * l4 + share1 * l1;
    };
    // The phase boundaries are Froude numbers on L itself, so L is the fixed point of
    // L = mix(V / sqrt(g L)); the iteration contracts because the lengths differ by far less
    // than L.
    double l = l1;
    for (int i = 0; i < 50; ++i) {
        const double next = mix(v / std::sqrt(kGravity * l));
        if (std::abs(next - l) <= 1e-12 * l) return next;
        l = next;
    }
    return l;
}

double OrcHullResistance::rail_factor(double heel_deg) const {
    // Python-VPP's immersed-rail correction (HydroMod._get_Rr). Not in the ORC VPP
    // Documentation 2023; kept because the Python-VPP downwind target is computed with it.
    const double phi = std::abs(heel_deg);
    if (!(phi > params_.rail_threshold_deg)) return 1.0;
    const double d = phi - params_.rail_threshold_deg;
    return 1.0 + params_.rail_coefficient * d * d;
}

HullResistance OrcHullResistance::resistance(double speed_mps, double heel_rad) const {
    const OrcHullOverrides& o = params_.overrides;
    const double v = std::abs(speed_mps);
    const double heel_deg = std::abs(heel_rad) * 180.0 / std::numbers::pi;

    const double l_res = dynamic_length(v);
    const double l_visc = o.lsm1.value_or(params_.lwl);
    const double lvr = o.lvr.empty() ? l_res / std::cbrt(params_.canoe_volume)
                                     : table_at(o.heel_deg, o.lvr, heel_deg);
    const double btr = !o.btr.empty()        ? table_at(o.heel_deg, o.btr, heel_deg)
                       : o.btr_effective ? *o.btr_effective
                                         : params_.bwl / params_.tc;
    const double area =
        o.wetted_area.empty() ? params_.wetted_area : table_at(o.heel_deg, o.wetted_area, heel_deg);

    HullResistance r;
    r.fn = v / std::sqrt(kGravity * l_res);
    r.lvr = lvr;
    r.btr = btr;
    const OrcDragSurfaces::Lookup look = surfaces_.lookup(r.fn, lvr, btr);
    r.lvr_clamped = look.lvr_clamped;
    r.btr_clamped = look.btr_clamped;
    r.fn_extrapolated = look.fn_extrapolated;
    r.in_range = !(r.lvr_clamped || r.btr_clamped || r.fn_extrapolated);
    if (v == 0.0) return r;

    r.residuary_N = look.value * params_.displacement * kGravity * 1e-3 * rail_factor(heel_deg);

    // Hughes's line is singular at Re = 10^2.03; Python-VPP floors Re at 1e4, which only ever
    // applies at speeds where the force is negligible.
    const double re = std::max(1e4, 0.85 * v * l_visc / params_.kinematic_viscosity);
    const double denom = std::log10(re) - 2.03;
    const double cf = 0.066 / (denom * denom);
    r.viscous_N = 0.5 * params_.density * v * v * area * cf * 1.05;
    r.total_N = r.residuary_N + r.viscous_N;
    return r;
}

OrcHullParameters load_orc_hull_parameters(const std::filesystem::path& boat_dir) {
    const Document hull_doc = read_document(boat_dir / "hull_hydro.json");
    const Document res_doc = read_document(boat_dir / "hull_resistance.json");

    OrcHullParameters p;
    const Node sailing = hull_doc.root().at("conditions").at("sailing");
    const Node result = sailing.at("result");
    const auto positive = [](const Node& node) {
        const double v = node.number();
        if (!(v > 0.0)) node.fail("expected a positive number");
        return v;
    };
    p.displacement = positive(result.at("displacement_kg"));
    p.canoe_volume = positive(result.at("canoe_volume"));
    p.lwl = positive(result.at("lwl"));
    p.bwl = positive(result.at("bwl"));
    p.tc = positive(result.at("tc"));
    p.wetted_area = positive(result.at("wetted_area_canoe"));
    if (sailing.has("orc")) p.overrides = read_overrides(sailing.at("orc"));

    const Node res = res_doc.root();
    const Node water = res.at("water");
    p.density = water.at("density").positive_value();
    p.kinematic_viscosity = water.at("dynamic_viscosity").positive_value() / p.density;
    const Node orc = res.at("orc");
    const Node rail = orc.at("immersed_rail");
    p.rail_threshold_deg = rail.at("threshold_deg").value();
    p.rail_coefficient = rail.at("coefficient").value();
    if (p.rail_coefficient < 0.0) rail.at("coefficient").fail("expected a non-negative number");
    const std::filesystem::path surfaces = orc.at("drag_surfaces").at("path").string();
    p.drag_surfaces_file = surfaces.is_absolute() ? surfaces : boat_dir / surfaces;
    return p;
}

std::unique_ptr<HullResistanceModel> load_hull_resistance(const std::filesystem::path& boat_dir,
                                                          HullResistanceKind kind) {
    switch (kind) {
        case HullResistanceKind::orc: {
            OrcHullParameters params = load_orc_hull_parameters(boat_dir);
            OrcDragSurfaces surfaces = OrcDragSurfaces::load(params.drag_surfaces_file);
            return std::make_unique<OrcHullResistance>(std::move(params), std::move(surfaces));
        }
        case HullResistanceKind::dsyhs:
            break;
    }
    throw BoatDataError(boat_dir / "hull_resistance.json",
                        "hull resistance model 'dsyhs' is not implemented");
}

std::unique_ptr<HullResistanceModel> load_hull_resistance(const std::filesystem::path& boat_dir) {
    const Document doc = read_document(boat_dir / "hull_resistance.json");
    const Node root = doc.root();
    HullResistanceKind kind = HullResistanceKind::orc;
    if (root.has("model")) {
        const Node model = root.at("model");
        const std::string name = model.string();
        if (name == "orc") {
            kind = HullResistanceKind::orc;
        } else if (name == "dsyhs") {
            kind = HullResistanceKind::dsyhs;
        } else {
            model.fail("unknown hull resistance model '" + name + "'");
        }
    }
    return load_hull_resistance(boat_dir, kind);
}

}  // namespace sail
