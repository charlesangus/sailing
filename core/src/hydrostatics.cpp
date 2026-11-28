#include "sail/hydrostatics.hpp"

#include <utility>

namespace sail {

namespace {

// Surface integrals over the wetted (sub)triangles, all positions relative to the reference
// point's world position. With d the depth and n the outward normal:
//   pressure_area   = sum of  integral d n dA          (force   = -rho g pressure_area)
//   pressure_moment = sum of  integral d (r x n) dA    (moment  = -rho g pressure_moment)
//   volume, first_moment follow from the divergence theorem with fields chosen to vanish where
//   d = 0, so the unmeshed free-surface cap contributes nothing.
struct Accumulator {
    Vec3 pressure_area = Vec3::Zero();
    Vec3 pressure_moment = Vec3::Zero();
    Vec3 first_moment = Vec3::Zero();
    double volume = 0.0;
    double cap_area = 0.0;
};

// Exact for d linear over the triangle. For linear f, g on a triangle of area A with vertex values
// f_i, g_i:  integral f g dA = A/12 (sum f_i g_i + sum f_i * sum g_i).
//   volume:        div(-d z^)            = 1   ->  V        = -integral d n_z dA
//   x, y moments:  div(-x d z^)          = x   ->  int x dV = -integral x d n_z dA
//   z moment:      div(-(z d + d^2/2) z^) = z  ->  int z dV = -integral (z d + d^2/2) n_z dA
// using dd/dz = -1, which holds for any surface height field.
inline void add_triangle(const Vec3& a, const Vec3& b, const Vec3& c, double da, double db,
                         double dc, Accumulator& acc) {
    const Vec3 area = 0.5 * (b - a).cross(c - a);
    const double sum_d = da + db + dc;
    const double mean_d = sum_d / 3.0;
    const Vec3 mean_rd = (da * a + db * b + dc * c + sum_d * (a + b + c)) / 12.0;
    const double mean_dd = (da * da + db * db + dc * dc + sum_d * sum_d) / 12.0;

    acc.pressure_area += mean_d * area;
    acc.pressure_moment += mean_rd.cross(area);
    acc.volume -= mean_d * area.z();
    acc.first_moment.x() -= mean_rd.x() * area.z();
    acc.first_moment.y() -= mean_rd.y() * area.z();
    acc.first_moment.z() -= (mean_rd.z() + 0.5 * mean_dd) * area.z();
    acc.cap_area -= area.z();
}

// Clips a triangle with at least one vertex above and one below the surface to its submerged
// part, the polygon on the d > 0 side. That polygon is a triangle (one vertex under) or a
// quadrilateral (two under), the two cases of Kerner's waterline split; the quadrilateral is fanned
// into two triangles. The waterline points have d = 0 exactly.
inline void add_cut_triangle(const Vec3* p, const double* d, Accumulator& acc) {
    Vec3 poly[4];
    double depth[4];
    int n = 0;
    for (int i = 0; i < 3; ++i) {
        const int j = i == 2 ? 0 : i + 1;
        const bool in_i = d[i] > 0.0;
        if (in_i) {
            poly[n] = p[i];
            depth[n] = d[i];
            ++n;
        }
        if (in_i != (d[j] > 0.0)) {
            const double s = d[i] / (d[i] - d[j]);
            poly[n] = p[i] + s * (p[j] - p[i]);
            depth[n] = 0.0;
            ++n;
        }
    }
    for (int k = 1; k + 1 < n; ++k) {
        add_triangle(poly[0], poly[k], poly[k + 1], depth[0], depth[k], depth[k + 1], acc);
    }
}

template <typename DepthAt>
HydrostaticResult integrate(const TriMesh& mesh, const State& pose, const HydrostaticParams& params,
                            HydrostaticScratch& scratch, DepthAt depth_at) {
    const Mat3 rot = pose.rotation();
    const std::size_t nv = mesh.vertices.size();
    scratch.offsets.resize(nv);
    scratch.depths.resize(nv);
    for (std::size_t i = 0; i < nv; ++i) {
        const Vec3 r = rot * (mesh.vertices[i] - params.reference_point);
        scratch.offsets[i] = r;
        scratch.depths[i] = depth_at(r);
    }

    HydrostaticResult result;
    Accumulator acc;
    for (const auto& tri : mesh.triangles) {
        const double d[3] = {scratch.depths[tri[0]], scratch.depths[tri[1]], scratch.depths[tri[2]]};
        if (d[0] <= 0.0 && d[1] <= 0.0 && d[2] <= 0.0) continue;
        const Vec3 p[3] = {scratch.offsets[tri[0]], scratch.offsets[tri[1]], scratch.offsets[tri[2]]};
        if (d[0] >= 0.0 && d[1] >= 0.0 && d[2] >= 0.0) {
            add_triangle(p[0], p[1], p[2], d[0], d[1], d[2], acc);
            ++result.submerged_triangles;
        } else {
            add_cut_triangle(p, d, acc);
            ++result.cut_triangles;
        }
    }

    const double rho_g = params.density * params.gravity;
    Vec3 force = -rho_g * acc.pressure_area;
    Vec3 moment = -rho_g * acc.pressure_moment;
    double volume = acc.volume;
    Vec3 first_moment = acc.first_moment;

    for (const VolumeCorrection& c : params.corrections) {
        const Vec3 r = rot * (c.centroid - params.reference_point);
        if (depth_at(r) <= 0.0) continue;
        const Vec3 lift(0.0, 0.0, rho_g * c.volume);
        force -= lift;
        moment -= r.cross(lift);
        volume -= c.volume;
        first_moment -= c.volume * r;
        result.correction_volume += c.volume;
    }

    result.volume = volume;
    result.centre_of_buoyancy =
        volume > 0.0 ? Vec3(pose.position + first_moment / volume) : pose.position;
    result.waterplane_area = acc.cap_area;
    result.force_world = force;
    result.moment_world = moment;
    result.wrench.force = rot.transpose() * force;
    result.wrench.moment = rot.transpose() * moment;
    return result;
}

}  // namespace

HydrostaticResult compute_hydrostatics(const TriMesh& mesh, const State& pose,
                                       const WaterSurface& water, double t,
                                       const HydrostaticParams& params,
                                       HydrostaticScratch& scratch) {
    if (const auto* flat = dynamic_cast<const FlatWater*>(&water)) {
        const double head = flat->level() - pose.position.z();
        return integrate(mesh, pose, params, scratch, [head](const Vec3& r) { return head - r.z(); });
    }
    const Vec3 origin = pose.position;
    return integrate(mesh, pose, params, scratch, [&water, &origin, t](const Vec3& r) {
        return water.height(origin.x() + r.x(), origin.y() + r.y(), t) - (origin.z() + r.z());
    });
}

HydrostaticResult compute_hydrostatics(const TriMesh& mesh, const State& pose, double water_level,
                                       const HydrostaticParams& params) {
    HydrostaticScratch scratch;
    return compute_hydrostatics(mesh, pose, FlatWater(water_level), 0.0, params, scratch);
}

HydrostaticsModule::HydrostaticsModule(TriMesh mesh, HydrostaticParams params,
                                       std::shared_ptr<const WaterSurface> water)
    : mesh_(std::move(mesh)),
      params_(std::move(params)),
      water_(water ? std::move(water) : std::make_shared<const FlatWater>(0.0)) {
    scratch_.offsets.resize(mesh_.vertices.size());
    scratch_.depths.resize(mesh_.vertices.size());
}

void HydrostaticsModule::accumulate(const State& state, double t, Wrench& wrench) {
    last_ = compute_hydrostatics(mesh_, state, *water_, t, params_, scratch_);
    wrench += last_.wrench;
}

}  // namespace sail
