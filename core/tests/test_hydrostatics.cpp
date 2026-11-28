#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <numbers>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "sail/equilibrium.hpp"
#include "sail/gravity.hpp"
#include "sail/hydrostatics.hpp"
#include "sail/mesh.hpp"

namespace {

using sail::compute_hydrostatics;
using sail::HydrostaticParams;
using sail::HydrostaticResult;
using sail::Quat;
using sail::State;
using sail::TriMesh;
using sail::Vec3;

constexpr double kRho = 1025.0;
constexpr double kG = sail::kGravity;
constexpr double kTol = 0.005;

double deg(double d) { return d * std::numbers::pi / 180.0; }

State heeled_pose(double heel_deg, double centre_z) {
    State s;
    s.position = Vec3(0.0, 0.0, centre_z);
    s.orientation = Quat(Eigen::AngleAxisd(deg(heel_deg), Vec3::UnitX()));
    return s;
}

// Rewinds each triangle of a convex mesh so its normal points away from `centre`.
void orient_outward(TriMesh& mesh, const Vec3& centre) {
    for (auto& t : mesh.triangles) {
        const Vec3& a = mesh.vertices[t[0]];
        const Vec3 n = (mesh.vertices[t[1]] - a).cross(mesh.vertices[t[2]] - a);
        if (n.dot((a + mesh.vertices[t[1]] + mesh.vertices[t[2]]) / 3.0 - centre) < 0.0) {
            std::swap(t[1], t[2]);
        }
    }
}

TriMesh make_unit_box() {
    TriMesh mesh;
    for (int i = 0; i < 8; ++i) {
        mesh.vertices.emplace_back((i & 1) - 0.5, ((i >> 1) & 1) - 0.5, ((i >> 2) & 1) - 0.5);
    }
    for (int axis = 0; axis < 3; ++axis) {
        const int u = 1 << ((axis + 1) % 3);
        const int v = 1 << ((axis + 2) % 3);
        for (int side = 0; side < 2; ++side) {
            const auto base = static_cast<std::uint32_t>(side << axis);
            const std::uint32_t q[4] = {base, base + static_cast<std::uint32_t>(u),
                                        base + static_cast<std::uint32_t>(u + v),
                                        base + static_cast<std::uint32_t>(v)};
            mesh.triangles.push_back({q[0], q[1], q[2]});
            mesh.triangles.push_back({q[0], q[2], q[3]});
        }
    }
    orient_outward(mesh, Vec3::Zero());
    return mesh;
}

TriMesh make_icosphere(int subdivisions, double radius) {
    const double p = (1.0 + std::sqrt(5.0)) / 2.0;
    std::vector<Vec3> v = {{-1, p, 0}, {1, p, 0}, {-1, -p, 0}, {1, -p, 0},
                           {0, -1, p}, {0, 1, p}, {0, -1, -p}, {0, 1, -p},
                           {p, 0, -1}, {p, 0, 1}, {-p, 0, -1}, {-p, 0, 1}};
    for (auto& x : v) x.normalize();
    std::vector<std::array<std::uint32_t, 3>> f = {
        {0, 11, 5}, {0, 5, 1},  {0, 1, 7},   {0, 7, 10}, {0, 10, 11}, {1, 5, 9}, {5, 11, 4},
        {11, 10, 2}, {10, 7, 6}, {7, 1, 8},  {3, 9, 4},  {3, 4, 2},   {3, 2, 6}, {3, 6, 8},
        {3, 8, 9},  {4, 9, 5},  {2, 4, 11},  {6, 2, 10}, {8, 6, 7},   {9, 8, 1}};
    for (int s = 0; s < subdivisions; ++s) {
        std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> cache;
        auto midpoint = [&](std::uint32_t a, std::uint32_t b) {
            const auto key = std::minmax(a, b);
            const auto it = cache.find(key);
            if (it != cache.end()) return it->second;
            v.push_back((v[a] + v[b]).normalized());
            const auto idx = static_cast<std::uint32_t>(v.size() - 1);
            cache.emplace(key, idx);
            return idx;
        };
        std::vector<std::array<std::uint32_t, 3>> next;
        for (const auto& t : f) {
            const std::uint32_t ab = midpoint(t[0], t[1]);
            const std::uint32_t bc = midpoint(t[1], t[2]);
            const std::uint32_t ca = midpoint(t[2], t[0]);
            next.push_back({t[0], ab, ca});
            next.push_back({t[1], bc, ab});
            next.push_back({t[2], ca, bc});
            next.push_back({ab, bc, ca});
        }
        f = std::move(next);
    }
    TriMesh mesh;
    for (const auto& x : v) mesh.vertices.push_back(radius * x);
    mesh.triangles = std::move(f);
    orient_outward(mesh, Vec3::Zero());
    return mesh;
}

struct Reference {
    double volume = 0.0;
    Vec3 centroid = Vec3::Zero();
};

// Exact volume and centroid of the part of a polyhedron below z = level, from signed tetrahedra
// with their apex on the water plane, so the unmeshed cap adds nothing.
Reference polyhedral_reference(const TriMesh& mesh, const State& pose, double level) {
    std::vector<Vec3> w;
    for (const auto& x : mesh.vertices) w.push_back(pose.position + pose.orientation * x);
    const Vec3 apex(0.0, 0.0, level);
    Reference ref;
    Vec3 moment = Vec3::Zero();
    auto add = [&](const Vec3& a, const Vec3& b, const Vec3& c) {
        const double vol = (a - apex).dot((b - apex).cross(c - apex)) / 6.0;
        ref.volume += vol;
        moment += vol * (apex + a + b + c) / 4.0;
    };
    for (const auto& t : mesh.triangles) {
        std::vector<Vec3> poly;
        for (int i = 0; i < 3; ++i) {
            const Vec3& a = w[t[i]];
            const Vec3& b = w[t[(i + 1) % 3]];
            const bool ina = a.z() < level;
            if (ina) poly.push_back(a);
            if (ina != (b.z() < level)) poly.push_back(a + (level - a.z()) / (b.z() - a.z()) * (b - a));
        }
        for (std::size_t k = 1; k + 1 < poly.size(); ++k) add(poly[0], poly[k], poly[k + 1]);
    }
    if (ref.volume > 0.0) ref.centroid = moment / ref.volume;
    return ref;
}

// Unit box heeled about x: the immersed part is its square section, rotated and clipped by the
// water plane, extruded along x. Area and centroid by the shoelace formula.
Reference box_reference(double heel_deg, double centre_z) {
    const double c = std::cos(deg(heel_deg));
    const double s = std::sin(deg(heel_deg));
    const double corners[4][2] = {{-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}};
    std::vector<std::array<double, 2>> sq;
    for (const auto& k : corners) sq.push_back({c * k[0] - s * k[1], s * k[0] + c * k[1] + centre_z});
    std::vector<std::array<double, 2>> poly;
    for (std::size_t i = 0; i < 4; ++i) {
        const auto& a = sq[i];
        const auto& b = sq[(i + 1) % 4];
        const bool ina = a[1] < 0.0;
        if (ina) poly.push_back(a);
        if (ina != (b[1] < 0.0)) {
            const double t = a[1] / (a[1] - b[1]);
            poly.push_back({a[0] + t * (b[0] - a[0]), 0.0});
        }
    }
    double area = 0.0;
    double cy = 0.0;
    double cz = 0.0;
    for (std::size_t i = 0; i < poly.size(); ++i) {
        const auto& a = poly[i];
        const auto& b = poly[(i + 1) % poly.size()];
        const double cross = a[0] * b[1] - b[0] * a[1];
        area += cross / 2.0;
        cy += (a[0] + b[0]) * cross / 6.0;
        cz += (a[1] + b[1]) * cross / 6.0;
    }
    Reference ref;
    ref.volume = area;
    if (area > 0.0) ref.centroid = Vec3(0.0, cy / area, cz / area);
    return ref;
}

void check_against(const HydrostaticResult& r, const Reference& ref, const State& pose,
                   double length) {
    REQUIRE(ref.volume > 0.0);
    const double weight = kRho * kG * ref.volume;
    CHECK(std::abs(r.volume - ref.volume) / ref.volume < kTol);
    CHECK(std::abs(r.force_world.z() - weight) / weight < kTol);
    CHECK(r.force_world.head<2>().norm() / r.force_world.norm() < 1e-9);
    CHECK((r.centre_of_buoyancy - ref.centroid).norm() / length < kTol);
    const Vec3 expected_moment = (ref.centroid - pose.position).cross(r.force_world);
    CHECK((r.moment_world - expected_moment).norm() / (weight * length) < kTol);
}

}  // namespace

TEST_CASE("unit box matches the analytic buoyancy and centre of buoyancy") {
    const TriMesh box = make_unit_box();
    REQUIRE(box.triangle_count() == 12);
    for (const double heel : {0.0, 30.0, 90.0, 150.0}) {
        for (const double centre_z : {0.0, 0.25, -3.0}) {
            CAPTURE(heel);
            CAPTURE(centre_z);
            const State pose = heeled_pose(heel, centre_z);
            const HydrostaticResult r = compute_hydrostatics(box, pose, 0.0);
            check_against(r, box_reference(heel, centre_z), pose, 1.0);
        }
    }
}

TEST_CASE("icosphere matches its exact polyhedral buoyancy and centre of buoyancy") {
    const TriMesh sphere = make_icosphere(3, 1.0);
    REQUIRE(sphere.triangle_count() == 1280);
    const Reference whole = polyhedral_reference(sphere, heeled_pose(0.0, -10.0), 0.0);
    CHECK(whole.volume < 4.0 / 3.0 * std::numbers::pi);
    for (const double heel : {0.0, 30.0, 90.0, 150.0}) {
        for (const double centre_z : {0.0, 0.5, -3.0}) {
            CAPTURE(heel);
            CAPTURE(centre_z);
            const State pose = heeled_pose(heel, centre_z);
            const HydrostaticResult r = compute_hydrostatics(sphere, pose, 0.0);
            const Reference ref = polyhedral_reference(sphere, pose, 0.0);
            check_against(r, ref, pose, 1.0);
            if (centre_z < -1.0) CHECK(std::abs(r.volume - whole.volume) / whole.volume < 1e-9);
            if (centre_z == 0.0) CHECK(std::abs(r.volume - whole.volume / 2.0) / whole.volume < 1e-9);
        }
    }
}

TEST_CASE("fully emerged mesh feels no force") {
    const TriMesh sphere = make_icosphere(3, 1.0);
    const HydrostaticResult r = compute_hydrostatics(sphere, heeled_pose(40.0, 1.5), 0.0);
    CHECK(r.volume == 0.0);
    CHECK(r.force_world.norm() == 0.0);
    CHECK(r.moment_world.norm() == 0.0);
    CHECK(r.waterplane_area == 0.0);
    CHECK(r.submerged_triangles == 0);
    CHECK(r.cut_triangles == 0);
}

TEST_CASE("fully submerged closed mesh has no moment about its own centroid") {
    TriMesh sphere = make_icosphere(3, 0.8);
    const Vec3 shift(0.3, -0.2, 0.1);
    for (auto& v : sphere.vertices) {
        v = Vec3(1.5 * v.x(), v.y(), 0.7 * v.z()) + shift;
    }
    const Reference centroid = polyhedral_reference(sphere, heeled_pose(0.0, -20.0), 0.0);
    HydrostaticParams params;
    params.reference_point = centroid.centroid - Vec3(0.0, 0.0, -20.0);
    State pose;
    pose.position = Vec3(4.0, -1.0, -5.0);
    pose.orientation = Quat(Eigen::AngleAxisd(0.7, Vec3(1.0, 2.0, -0.5).normalized()));
    const HydrostaticResult r = compute_hydrostatics(sphere, pose, 0.0, params);
    const double weight = kRho * kG * centroid.volume;
    CHECK(std::abs(r.force_world.z() - weight) / weight < 1e-9);
    CHECK(r.moment_world.norm() / weight < 1e-9);
    CHECK((r.centre_of_buoyancy - pose.position).norm() < 1e-9);
    CHECK(r.waterplane_area == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("waterplane area of a box is its sectional width over cos(heel)") {
    const TriMesh box = make_unit_box();
    for (const double heel : {0.0, 30.0}) {
        const HydrostaticResult r = compute_hydrostatics(box, heeled_pose(heel, 0.0), 0.0);
        CHECK(r.waterplane_area == doctest::Approx(1.0 / std::cos(deg(heel))));
    }
}

TEST_CASE("module reports the wrench in the body frame about the CG") {
    const TriMesh box = make_unit_box();
    HydrostaticParams at_centre;
    HydrostaticParams at_cg;
    at_cg.reference_point = Vec3(0.1, 0.0, 0.3);
    sail::HydrostaticsModule module(box, at_cg);

    const Quat q(Eigen::AngleAxisd(deg(30.0), Vec3::UnitX()));
    State pose;
    pose.orientation = q;
    pose.position = Vec3(1.0, 2.0, 0.1) + q * at_cg.reference_point;

    sail::Wrench wrench;
    wrench.force = Vec3(1.0, 0.0, 0.0);
    module.accumulate(pose, 0.0, wrench);
    const HydrostaticResult& r = module.last_result();
    CHECK((wrench.force - Vec3(1.0, 0.0, 0.0) - q.conjugate() * r.force_world).norm() < 1e-6);
    CHECK((wrench.moment - q.conjugate() * r.moment_world).norm() < 1e-6);

    State centre_pose = pose;
    centre_pose.position = Vec3(1.0, 2.0, 0.1);
    const HydrostaticResult c = compute_hydrostatics(box, centre_pose, 0.0, at_centre);
    const Vec3 lever = pose.position - centre_pose.position;
    CHECK((r.force_world - c.force_world).norm() < 1e-6);
    CHECK((r.moment_world - (c.moment_world - lever.cross(c.force_world))).norm() < 1e-6);
    CHECK((r.centre_of_buoyancy - c.centre_of_buoyancy).norm() < 1e-9);
}

TEST_CASE("wave surface path agrees with flat water at a constant height") {
    struct ConstantSurface final : sail::WaterSurface {
        double height(double, double, double) const override { return 0.2; }
    };
    const TriMesh sphere = make_icosphere(3, 1.0);
    const State pose = heeled_pose(30.0, 0.0);
    sail::HydrostaticScratch scratch;
    const HydrostaticResult wave =
        compute_hydrostatics(sphere, pose, ConstantSurface{}, 0.0, HydrostaticParams{}, scratch);
    const HydrostaticResult flat = compute_hydrostatics(sphere, pose, 0.2);
    CHECK(wave.volume == doctest::Approx(flat.volume).epsilon(1e-12));
    CHECK((wave.moment_world - flat.moment_world).norm() < 1e-6);
}

TEST_CASE("volume corrections are subtracted only while submerged") {
    const TriMesh box = make_unit_box();
    HydrostaticParams params;
    params.corrections.push_back({Vec3(0.0, 0.0, -0.4), 0.01});
    const HydrostaticResult wet = compute_hydrostatics(box, heeled_pose(0.0, 0.0), 0.0, params);
    CHECK(wet.correction_volume == doctest::Approx(0.01));
    CHECK(wet.volume == doctest::Approx(0.49));
    const HydrostaticResult dry = compute_hydrostatics(box, heeled_pose(180.0, 0.0), 0.0, params);
    CHECK(dry.correction_volume == 0.0);
    CHECK(dry.volume == doctest::Approx(0.5));
}

TEST_CASE("OBJ loader reads objects, index forms and polygons") {
    const auto path = std::filesystem::temp_directory_path() / "sail_test_hydrostatics.obj";
    {
        std::ofstream out(path);
        out << "# comment\n"
               "mtllib none.mtl\n"
               "\n"
               "v 0 0 0\n"
               "v 1 0 0\n"
               "v 1 1 0  # trailing comment\n"
               "v 0 1 0\n"
               "vt 0 0\n"
               "vn 0 0 1\n"
               "o first\n"
               "f 1/1/1 2/1/1 3/1/1 4/1/1\n"
               "o second\n"
               "v 0 0 1\n"
               "f -1 1//1 2/1\n";
    }
    const TriMesh all = sail::load_obj(path);
    CHECK(all.vertex_count() == 5);
    CHECK(all.triangle_count() == 3);

    const std::vector<std::string> names = {"second"};
    const TriMesh second = sail::load_obj(path, names);
    REQUIRE(second.vertex_count() == 3);
    REQUIRE(second.triangle_count() == 1);
    CHECK((second.vertices[second.triangles[0][0]] == Vec3(0.0, 0.0, 1.0)));
    CHECK((second.vertices[second.triangles[0][1]] == Vec3(0.0, 0.0, 0.0)));

    const std::vector<std::string> missing = {"third"};
    CHECK_THROWS_AS(sail::load_obj(path, missing), sail::ObjError);

    {
        std::ofstream out(path);
        out << "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 9\n";
    }
    try {
        (void)sail::load_obj(path);
        FAIL("expected ObjError");
    } catch (const sail::ObjError& e) {
        CHECK(e.line() == 4);
        CHECK((e.file() == path));
    }
    std::filesystem::remove(path);

    CHECK_THROWS_AS(sail::load_obj(path), sail::ObjError);
}

TEST_CASE("J/80 physics meshes float at the design waterline") {
    const std::filesystem::path dir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";
    const TriMesh hull = sail::load_obj(dir / "hull_physics.obj");
    const TriMesh appendages = sail::load_obj(dir / "appendages.obj");
    const std::vector<std::string> keel_name = {"keel"};
    const std::vector<std::string> rudder_name = {"rudder"};
    const TriMesh keel = sail::load_obj(dir / "appendages.obj", keel_name);
    const TriMesh rudder = sail::load_obj(dir / "appendages.obj", rudder_name);
    CHECK(hull.triangle_count() == 1598);
    CHECK(keel.triangle_count() == 3076);
    CHECK(rudder.triangle_count() == 1686);
    CHECK(appendages.triangle_count() == 4762);

    const sail::BoatHydrostatics loaded = sail::load_boat_hydrostatics(dir);
    const TriMesh& boat = loaded.mesh;
    CHECK(boat.triangle_count() == hull.triangle_count() + appendages.triangle_count());

    std::ifstream in(dir / "appendages.json");
    REQUIRE(in);
    const nlohmann::json foils = nlohmann::json::parse(in).at("foils").at("keel");
    const double root_overlap = foils.at("root_overlap").at("value").get<double>();
    const double root_depth = foils.at("root_overlap_depth").at("value").get<double>();

    // The overlap sits in the top root_overlap_depth of the keel mesh, inside the hull.
    double top = -1e9;
    for (const auto& v : keel.vertices) top = std::max(top, v.z());
    REQUIRE(loaded.corrections.size() == 1);
    const sail::VolumeCorrection& overlap = loaded.corrections.front();
    CHECK(overlap.volume == doctest::Approx(root_overlap));
    CHECK(overlap.centroid.z() == doctest::Approx(top - root_depth / 2.0));

    HydrostaticParams params;
    params.corrections = loaded.corrections;
    const HydrostaticResult r = compute_hydrostatics(boat, State{}, 0.0, params);
    const HydrostaticResult raw = compute_hydrostatics(boat, State{}, 0.0);

    // hull_hydro.json result.total_volume
    constexpr double kTotalVolume = 1.40293;
    CHECK(r.correction_volume == doctest::Approx(root_overlap));
    CHECK(raw.volume - r.volume == doctest::Approx(root_overlap));
    CHECK(std::abs(r.volume - kTotalVolume) / kTotalVolume < 0.02);
    CHECK(r.force_world.z() == doctest::Approx(kRho * kG * r.volume));
}
