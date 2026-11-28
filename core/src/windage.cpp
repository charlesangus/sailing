#include "sail/windage.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "dossier_json.hpp"
#include "sail/sails.hpp"

namespace sail {

namespace {

using dossier::Document;
using dossier::Node;
using dossier::read_document;

// Below this air speed (m/s) the direction of the drag is undefined and the drag is zero.
constexpr double kMinSpeed = 1e-9;

double positive_number(const Node& entry) {
    const double v = entry.value();
    if (!(v > 0.0)) entry.at("value").fail("expected a positive number");
    return v;
}

double non_negative_number(const Node& entry) {
    const double v = entry.value();
    if (v < 0.0) entry.at("value").fail("expected a non-negative number");
    return v;
}

int count_value(const Node& entry) {
    const double v = positive_number(entry);
    if (v != std::floor(v) || v > 1000.0) entry.at("value").fail("expected a whole number");
    return static_cast<int>(v);
}

struct Polygon {
    std::array<Vec3, 8> v;
    int n = 0;
};

// The part of a convex polygon with world height >= 0, heights being z0 + row . r for body-frame
// point r.
Polygon clip_above_water(const std::array<Vec3, 4>& corners, double z0, const Eigen::RowVector3d& row) {
    Polygon out;
    for (int i = 0; i < 4; ++i) {
        const Vec3& p = corners[static_cast<std::size_t>(i)];
        const Vec3& q = corners[static_cast<std::size_t>((i + 1) % 4)];
        const double hp = z0 + row.dot(p);
        const double hq = z0 + row.dot(q);
        if (hp >= 0.0) out.v[static_cast<std::size_t>(out.n++)] = p;
        if ((hp >= 0.0) != (hq >= 0.0)) {
            out.v[static_cast<std::size_t>(out.n++)] = p + (q - p) * (hp / (hp - hq));
        }
    }
    return out;
}

struct AreaCentre {
    double area = 0.0;
    Vec3 centre = Vec3::Zero();
};

AreaCentre area_and_centre(const Polygon& poly) {
    AreaCentre out;
    if (poly.n < 3) return out;
    const Vec3& o = poly.v[0];
    Vec3 weighted = Vec3::Zero();
    for (int i = 1; i + 1 < poly.n; ++i) {
        const Vec3& a = poly.v[static_cast<std::size_t>(i)];
        const Vec3& b = poly.v[static_cast<std::size_t>(i + 1)];
        const double tri = 0.5 * (a - o).cross(b - o).norm();
        out.area += tri;
        weighted += tri * (o + a + b) / 3.0;
    }
    if (out.area > 0.0) out.centre = weighted / out.area;
    return out;
}

// Piecewise-linear freeboard through the stem, mast and transom certificate values, flat aft of
// the transom point. `x` is the distance aft of the stem.
struct Freeboard {
    double mast_x = 0.0;
    double aft_x = 0.0;
    double fwd = 0.0;
    double at_mast = 0.0;
    double aft = 0.0;

    double operator()(double x) const {
        if (x <= mast_x) return fwd + (at_mast - fwd) * (x / mast_x);
        if (x <= aft_x) return at_mast + (aft - at_mast) * ((x - mast_x) / (aft_x - mast_x));
        return aft;
    }

    double mean(double length) const {
        const double upto_aft = std::min(length, aft_x);
        double integral = 0.0;
        const double first = std::min(upto_aft, mast_x);
        integral += 0.5 * (fwd + (*this)(first)) * first;
        if (upto_aft > mast_x) {
            integral += 0.5 * ((*this)(mast_x) + (*this)(upto_aft)) * (upto_aft - mast_x);
        }
        if (length > aft_x) integral += aft * (length - aft_x);
        return integral / length;
    }
};

}  // namespace

WindageParams load_windage_params(const std::filesystem::path& boat_dir,
                                  const Vec3& reference_point) {
    const Document windage_doc = read_document(boat_dir / "windage.json");
    const Document rig_doc = read_document(boat_dir / "rig.json");
    const Document particulars_doc = read_document(boat_dir / "particulars.json");
    const Document hull_doc = read_document(boat_dir / "hull_hydro.json");

    const Node transform = hull_doc.root().at("transform");
    const Vec3 offset(transform.at("stem_x_in_body").number(), 0.0,
                      transform.at("flotation_z_in_body").number());
    const Node tc_node = hull_doc.root().at("conditions").at("sailing").at("result").at("tc");
    const double tc = tc_node.number();
    if (!(tc > 0.0)) tc_node.fail("expected a positive number");

    const Node w = windage_doc.root();
    const Node rig = rig_doc.root();
    const Node part = particulars_doc.root();

    WindageParams p;
    p.air_density = SailSetParams{}.air_density;
    p.reference_point = reference_point;

    const Vec3 mast_axis = rig.at("mast_axis_at_sheer").at("value").vec3();
    const double mast_top = positive_number(rig.at("mast_top_above_water"));

    {
        const Node mast = w.at("mast");
        const int strips = count_value(mast.at("strips"));
        const double transverse = positive_number(rig.at("mast_section_transverse"));
        const double fore_aft = positive_number(rig.at("mast_section_fore_aft"));
        const double cd_front = positive_number(mast.at("cd_front"));
        const double cd_side = positive_number(mast.at("cd_side"));
        const double base = mast_axis.z();
        if (!(mast_top > base)) rig.at("mast_top_above_water").fail("not above the mast base");
        const double dz = (mast_top - base) / strips;
        for (int i = 0; i < strips; ++i) {
            WindageBox b;
            b.name = "mast_strip_" + std::to_string(i);
            b.group = WindageGroup::mast;
            b.centre = Vec3(mast_axis.x(), 0.0, base + (i + 0.5) * dz) + offset;
            b.area = Vec3(transverse * dz, fore_aft * dz, 0.0);
            b.cd = Vec3(cd_front, cd_side, 0.0);
            p.boxes.push_back(std::move(b));
        }
    }

    {
        const Node boom = w.at("boom");
        const double length = positive_number(rig.at("e"));
        const double depth = positive_number(rig.at("boom_section_depth"));
        const double height = positive_number(rig.at("boom_height_above_water"));
        const double cd_side = positive_number(boom.at("cd_side"));
        WindageBox b;
        b.name = "boom";
        b.group = WindageGroup::boom;
        b.centre = Vec3(mast_axis.x() - 0.5 * length, 0.0, height) + offset;
        b.area = Vec3(depth * depth, length * depth, length * depth);
        b.cd = Vec3(positive_number(boom.at("cd_end")), cd_side, cd_side);
        p.boxes.push_back(std::move(b));
    }

    {
        const Node rigging = w.at("rigging");
        const double cd =
            positive_number(rigging.at("cd")) * (1.0 + non_negative_number(rigging.at("spreader_factor")));
        const Vec3 chainplate = rig.at("shroud_chainplate_base").at("value").vec3();
        const double upper_spreader =
            mast_axis.z() + positive_number(rig.at("spreader_upper_height_above_sheer"));
        const double lower_spreader =
            mast_axis.z() + positive_number(rig.at("spreader_lower_height_above_sheer"));
        const double hounds = rig.at("hounds_forestay_on_mast").at("value").vec3().z();
        const auto add = [&](std::string name, const Vec3& a, const Vec3& b, const Node& diameter) {
            WindageWire wire;
            wire.name = std::move(name);
            wire.a = a + offset;
            wire.b = b + offset;
            wire.diameter = positive_number(diameter);
            wire.cd = cd;
            p.wires.push_back(std::move(wire));
        };
        add("forestay", rig.at("forestay_deck_point").at("value").vec3(), rig.at("hounds_forestay_on_mast").at("value").vec3(),
            rigging.at("forestay_diameter"));
        const struct {
            const char* name;
            double z;
            const char* diameter;
        } shrouds[] = {{"upper_shroud", hounds, "upper_shroud_diameter"},
                       {"intermediate_shroud", upper_spreader, "intermediate_shroud_diameter"},
                       {"lower_shroud", lower_spreader, "lower_shroud_diameter"}};
        for (const auto& s : shrouds) {
            for (const double side : {1.0, -1.0}) {
                add(std::string(s.name) + (side > 0.0 ? "_port" : "_starboard"),
                    Vec3(mast_axis.x(), 0.0, s.z),
                    Vec3(chainplate.x(), side * std::abs(chainplate.y()), chainplate.z()),
                    rigging.at(s.diameter));
            }
        }
        add("backstay", Vec3(mast_axis.x(), 0.0, mast_top),
            Vec3(-positive_number(part.at("safp")), 0.0, positive_number(part.at("freeboard_aft_fam"))),
            rigging.at("backstay_diameter"));
    }

    {
        const Node hull = w.at("hull");
        const int per_side = count_value(hull.at("panels_per_side"));
        const double cd = positive_number(hull.at("cd"));
        const double beam = positive_number(part.at("beam"));
        const double loa = positive_number(part.at("loa"));
        Freeboard fb;
        fb.mast_x = -mast_axis.x();
        fb.aft_x = positive_number(part.at("safp"));
        fb.fwd = positive_number(part.at("freeboard_fwd_ffm"));
        fb.at_mast = positive_number(part.at("freeboard_at_mast"));
        fb.aft = positive_number(part.at("freeboard_aft_fam"));
        if (!(fb.mast_x < fb.aft_x)) part.at("safp").fail("not aft of the mast");

        const double half_beam = 0.5 * beam;
        for (const double side : {1.0, -1.0}) {
            for (int i = 0; i < per_side; ++i) {
                const double x0 = -loa * i / per_side;
                const double x1 = -loa * (i + 1) / per_side;
                const double top = fb(-0.5 * (x0 + x1));
                WindagePanel panel;
                panel.name = std::string(side > 0.0 ? "port_side_" : "starboard_side_") + std::to_string(i);
                panel.corners = {Vec3(x0, side * half_beam, -tc) + offset,
                                 Vec3(x1, side * half_beam, -tc) + offset,
                                 Vec3(x1, side * half_beam, top) + offset,
                                 Vec3(x0, side * half_beam, top) + offset};
                panel.outward = Vec3(0.0, side, 0.0);
                panel.cd = cd;
                p.panels.push_back(std::move(panel));
            }
        }
        const double x_beam = -positive_number(part.at("max_beam_station_from_stem"));
        const double top = fb.mean(loa);
        for (const double dir : {1.0, -1.0}) {
            WindagePanel panel;
            panel.name = dir > 0.0 ? "bow" : "stern";
            panel.corners = {Vec3(x_beam, -half_beam, -tc) + offset, Vec3(x_beam, half_beam, -tc) + offset,
                             Vec3(x_beam, half_beam, top) + offset, Vec3(x_beam, -half_beam, top) + offset};
            panel.outward = Vec3(dir, 0.0, 0.0);
            panel.cd = cd;
            p.panels.push_back(std::move(panel));
        }
    }

    {
        const Node crew = w.at("crew");
        p.crew.cd = positive_number(crew.at("cd"));
        p.crew.sitting_front = non_negative_number(crew.at("sitting_front_area_per_person"));
        p.crew.sitting_side = non_negative_number(crew.at("sitting_side_area_per_person"));
        p.crew.hiking_front = non_negative_number(crew.at("hiking_front_area_per_person"));
        p.crew.hiking_side = non_negative_number(crew.at("hiking_side_area_per_person"));
    }
    return p;
}

WindageParams load_windage_params(const MassModel& model) {
    WindageParams p = load_windage_params(model.data().directory, model.cg());
    if (model.crew_mass() > 0.0) {
        const Document mass_doc = read_document(model.data().directory / "mass.json");
        p.crew.count = count_value(mass_doc.root().at("crew_count"));
        p.crew.position = model.crew_position();
    }
    return p;
}

WindageModule::WindageModule(WindageParams params, std::shared_ptr<const WindField> wind)
    : params_(std::move(params)), wind_(std::move(wind)) {
    if (!wind_) throw std::invalid_argument("WindageModule: null wind field");
    if (!(params_.air_density > 0.0)) {
        throw std::invalid_argument("WindageModule: air density must be positive");
    }
}

void WindageModule::set_wind(std::shared_ptr<const WindField> wind) {
    if (!wind) throw std::invalid_argument("WindageModule::set_wind: null wind field");
    wind_ = std::move(wind);
}

void WindageModule::set_crew(int count, const Vec3& position, CrewPosture posture) {
    if (count < 0) throw std::invalid_argument("WindageModule::set_crew: negative crew count");
    params_.crew.count = count;
    params_.crew.position = position;
    params_.crew.posture = posture;
}

void WindageModule::accumulate(const State& state, double t, Wrench& wrench) {
    last_ = WindageResult{};
    const Mat3 rotation = state.rotation();
    const Eigen::RowVector3d up_row = rotation.row(2);
    const Vec3& ref = params_.reference_point;
    const double half_rho = 0.5 * params_.air_density;

    const auto apply = [&](WindageGroup group, const Vec3& r, double drag_area, const Vec3& v) {
        const Vec3 force = half_rho * drag_area * v.norm() * v;
        Wrench& g = last_.groups[static_cast<std::size_t>(group)];
        g.force += force;
        g.moment += r.cross(force);
    };

    const auto box = [&](WindageGroup group, const Vec3& hull_centre, const Vec3& area, const Vec3& cd) {
        const Vec3 r = hull_centre - ref;
        const Vec3 v = apparent_wind(state, r, *wind_, t).body;
        const double speed = v.norm();
        if (speed < kMinSpeed) return;
        const Vec3 n = v / speed;
        const double cda = std::abs(n.x()) * cd.x() * area.x() + std::abs(n.y()) * cd.y() * area.y() +
                           std::abs(n.z()) * cd.z() * area.z();
        apply(group, r, cda, v);
    };

    for (const WindageBox& b : params_.boxes) box(b.group, b.centre, b.area, b.cd);

    for (const WindageWire& wire : params_.wires) {
        const Vec3 along = wire.b - wire.a;
        const double length = along.norm();
        if (!(length > 0.0)) continue;
        const Vec3 r = 0.5 * (wire.a + wire.b) - ref;
        const Vec3 v = apparent_wind(state, r, *wind_, t).body;
        const double speed = v.norm();
        if (speed < kMinSpeed) continue;
        const double sine = (v / speed).cross(along / length).norm();
        apply(WindageGroup::rigging, r, wire.cd * wire.diameter * length * sine, v);
    }

    for (const WindagePanel& panel : params_.panels) {
        std::array<Vec3, 4> body;
        for (std::size_t i = 0; i < 4; ++i) body[i] = panel.corners[i] - ref;
        const AreaCentre exposed =
            area_and_centre(clip_above_water(body, state.position.z(), up_row));
        if (panel.outward.y() > 0.5) {
            last_.topsides_port += exposed.area;
        } else if (panel.outward.y() < -0.5) {
            last_.topsides_starboard += exposed.area;
        } else if (panel.outward.x() > 0.5) {
            last_.topsides_bow += exposed.area;
        } else {
            last_.topsides_stern += exposed.area;
        }
        if (!(exposed.area > 0.0)) continue;
        const Vec3 v = apparent_wind(state, exposed.centre, *wind_, t).body;
        const double speed = v.norm();
        if (speed < kMinSpeed) continue;
        const double facing = -panel.outward.dot(v) / speed;
        if (facing <= 0.0) continue;
        apply(WindageGroup::hull, exposed.centre, panel.cd * exposed.area * facing, v);
    }

    const WindageCrew& crew = params_.crew;
    if (crew.count > 0) {
        const bool hiking = crew.posture == CrewPosture::hiking;
        const double people = crew.count;
        const Vec3 area((hiking ? crew.hiking_front : crew.sitting_front) * people,
                        (hiking ? crew.hiking_side : crew.sitting_side) * people, 0.0);
        box(WindageGroup::crew, crew.position, area, Vec3(crew.cd, crew.cd, 0.0));
    }

    for (const Wrench& g : last_.groups) last_.wrench += g;
    wrench += last_.wrench;
}

}  // namespace sail
