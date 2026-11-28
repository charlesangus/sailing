#include "sail/boat_data.hpp"

#include <cmath>
#include <sstream>
#include <utility>

#include <Eigen/Dense>

#include "dossier_json.hpp"

namespace sail {

BoatDataError::BoatDataError(std::filesystem::path file, const std::string& message)
    : std::runtime_error(file.string() + ": " + message), file_(std::move(file)) {}

namespace {

using dossier::Document;
using dossier::Node;
using dossier::read_document;

std::string format_number(double v) {
    std::ostringstream out;
    out << v;
    return out.str();
}

MassShape parse_shape(const Node& shape) {
    const Node kind = shape.at("value");
    const std::string s = kind.string();
    if (s == "point") return MassShape::point;
    if (s == "box") return MassShape::box;
    if (s == "shell_box") return MassShape::shell_box;
    kind.fail("unknown shape '" + s + "' (expected point, box or shell_box)");
}

MassItem parse_item(const Node& node, const std::string& name, const Vec3& to_hull) {
    MassItem item;
    item.name = name;
    item.mass = node.positive_value();
    item.in_displacement = node.at("in_displacement").boolean();
    item.residual = node.has("residual") && node.at("residual").boolean();
    item.cg = node.at("cg").at("value").vec3() + to_hull;
    if (node.has("shape")) {
        const Node shape = node.at("shape");
        item.shape = parse_shape(shape);
        if (item.shape != MassShape::point) {
            const Node dims = shape.at("dims");
            item.dims = dims.vec3();
            if (!(item.dims.minCoeff() > 0.0)) dims.fail("expected positive extents");
        }
    }
    return item;
}

AddedMassCoefficients parse_added_mass(const Node& entry) {
    AddedMassCoefficients c;
    const Node value = entry.at("value");
    if (!value.json().is_object()) value.fail("expected an object");
    for (const auto& field : value.json().items()) {
        const std::string& key = field.key();
        const double v = value.at(key).number();
        if (key == "surge_fraction") c.surge_fraction = v;
        else if (key == "sway_scale") c.sway_scale = v;
        else if (key == "heave_scale") c.heave_scale = v;
        else if (key == "roll_scale") c.roll_scale = v;
        else if (key == "pitch_scale") c.pitch_scale = v;
        else if (key == "yaw_scale") c.yaw_scale = v;
        else if (key == "length_factor") c.length_factor = v;
        else value.at(key).fail("unknown added-mass coefficient");
        if (v < 0.0) value.at(key).fail("expected a non-negative number");
    }
    return c;
}

// Least-squares quadratic z(x) through the sheer points.
std::array<double, 3> fit_sheer(const Node& points) {
    const std::size_t n = points.size();
    if (n < 3) points.fail("expected at least 3 points");
    Eigen::MatrixXd a(n, 3);
    Eigen::VectorXd b(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Node p = points[i];
        if (p.size() != 2) p.fail("expected [x, z]");
        const double x = p[0].number();
        const auto row = static_cast<Eigen::Index>(i);
        a(row, 0) = 1.0;
        a(row, 1) = x;
        a(row, 2) = x * x;
        b(row) = p[1].number();
    }
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(a);
    if (qr.rank() < 3) points.fail("points do not determine a quadratic");
    const Eigen::Vector3d c = qr.solve(b);
    return {c[0], c[1], c[2]};
}

// Beyond this the longitudinal balance is being forced by something other than the hull.
constexpr double kMaxResidualXShift = 0.5;

}  // namespace

BoatData load_boat_data(const std::filesystem::path& boat_dir) {
    const Document mass_doc = read_document(boat_dir / "mass.json");
    const Document part_doc = read_document(boat_dir / "particulars.json");
    const Document hull_doc = read_document(boat_dir / "hull_hydro.json");
    const Node mass = mass_doc.root();
    const Node part = part_doc.root();
    const Node hydro = hull_doc.root();

    BoatData data;
    data.directory = boat_dir;
    data.name = part.at("name").at("value").string();

    const Node transform = hydro.at("transform");
    data.stem_x_in_body = transform.at("stem_x_in_body").number();
    data.flotation_z_in_body = transform.at("flotation_z_in_body").number();
    const Vec3 to_hull(data.stem_x_in_body, 0.0, data.flotation_z_in_body);

    data.displacement = part.at("displacement").positive_value();
    data.vcgm = part.at("vcgm").value();
    data.density = 1000.0 * part.at("flotation_sg").positive_value();
    data.sink = part.at("sink").positive_value();
    data.righting_moment_1deg = part.at("righting_moment_1deg").positive_value();

    data.sheer = fit_sheer(hydro.at("sheer").at("points"));
    const Node result = hydro.at("result");
    data.hull.lwl = result.at("lwl").number();
    data.hull.bwl = result.at("bwl").number();
    data.hull.tc = result.at("tc").number();
    data.hull.cp = result.at("cp").number();
    const Node cw = result.at("cw");
    data.hull.cw = cw.number();
    if (!(data.hull.cw > 0.0)) cw.fail("expected a positive number");
    data.hull.lcb_x_canoe = result.at("lcb_x_canoe").number();
    data.hull.lcb_x_total = result.at("lcb_x_total").number();

    data.crew.sailing_mass = mass.at("crew_weight_max_class").positive_value();
    data.crew.orc_stability_mass = mass.at("crew_weight_default_orc").positive_value() +
                                   mass.at("orc_stability_gear").value();
    data.crew.x = mass.at("crew_cg_x").value() + data.stem_x_in_body;
    data.crew.height_above_sheer = mass.at("crew_cg_above_sheer").value();
    data.crew.orc_stability_z = mass.at("orc_stability_crew_gear_z").value();

    if (mass.has("added_mass")) data.added_mass = parse_added_mass(mass.at("added_mass"));

    const Node items = mass.at("items");
    if (!items.json().is_object() || items.json().empty()) items.fail("expected a non-empty object");
    for (const auto& entry : items.json().items()) {
        data.items.push_back(parse_item(items.at(entry.key()), entry.key(), to_hull));
    }

    MassItem* residual = nullptr;
    double other_mass = 0.0;
    double other_moment_x = 0.0;
    double other_moment_z = 0.0;
    for (MassItem& item : data.items) {
        if (item.residual) {
            if (residual) {
                items.at(item.name).fail("more than one item is marked residual");
            }
            if (!item.in_displacement) {
                items.at(item.name).fail("the residual item must be in the displacement");
            }
            residual = &item;
        } else if (item.in_displacement) {
            other_mass += item.mass;
            other_moment_x += item.mass * item.cg.x();
            other_moment_z += item.mass * item.cg.z();
        }
    }
    if (!residual) items.fail("no item is marked residual");

    const Node residual_node = items.at(residual->name);
    residual->mass = data.displacement - other_mass;
    if (!(residual->mass > 0.0)) {
        residual_node.fail("solved mass " + format_number(residual->mass) +
                           " kg is not positive: the other in-displacement items weigh " +
                           format_number(other_mass) + " kg against a displacement of " +
                           format_number(data.displacement) + " kg");
    }
    residual->cg.x() = (data.displacement * data.hull.lcb_x_total - other_moment_x) / residual->mass;
    if (std::abs(residual->cg.x() - data.hull.lcb_x_canoe) > kMaxResidualXShift) {
        residual_node.fail("solved cg x " + format_number(residual->cg.x()) + " m is more than " +
                           format_number(kMaxResidualXShift) + " m from the canoe-body LCB " +
                           format_number(data.hull.lcb_x_canoe) + " m");
    }
    residual->cg.z() = (data.displacement * data.vcgm - other_moment_z) / residual->mass;

    const Node bounds = residual_node.at("cg_z_bounds").at("value");
    if (bounds.size() != 2) bounds.fail("expected [low, high]");
    const double height = residual->cg.z();
    if (!(height >= bounds[0].number() && height <= bounds[1].number())) {
        residual_node.fail("solved cg height " + format_number(height) +
                           " m above the flotation plane is outside cg_z_bounds [" +
                           format_number(bounds[0].number()) + ", " +
                           format_number(bounds[1].number()) + "]");
    }
    return data;
}

}  // namespace sail
