#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "sail/mesh.hpp"
#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace sail {

// Free-surface elevation, world z in metres, at world (x, y) and time t seconds.
class WaterSurface {
public:
    virtual ~WaterSurface() = default;
    virtual double height(double x, double y, double t) const = 0;
};

// Still water at a constant level. compute_hydrostatics recognises it and skips the per-vertex
// height query.
class FlatWater final : public WaterSurface {
public:
    explicit FlatWater(double level = 0.0) : level_(level) {}
    double height(double, double, double) const override { return level_; }
    double level() const { return level_; }

private:
    double level_;
};

// A small closed volume that the mesh counts twice, such as an appendage root overlapping the
// hull it joins. Its buoyancy is subtracted, as a point volume at `centroid`, whenever that point
// is below the surface.
struct VolumeCorrection {
    // Mesh frame, m.
    Vec3 centroid = Vec3::Zero();
    // m^3
    double volume = 0.0;
};

struct HydrostaticParams {
    // kg/m^3
    double density = 1025.0;
    // m/s^2
    double gravity = 9.81;
    // Mesh-frame location of the body's reference point (the CG). State::position is where this
    // point is in the world, and moments are taken about it.
    Vec3 reference_point = Vec3::Zero();
    std::vector<VolumeCorrection> corrections;
};

struct HydrostaticResult {
    // Submerged volume, m^3, net of `correction_volume`.
    double volume = 0.0;
    // World frame. Equal to the reference point's world position when nothing is submerged.
    Vec3 centre_of_buoyancy = Vec3::Zero();
    // Area of the submerged volume's free-surface cap projected on the horizontal, m^2. Zero when
    // fully submerged or emerged. Assumes a closed mesh.
    double waterplane_area = 0.0;
    // Hydrostatic force and its moment about the reference point, world frame.
    Vec3 force_world = Vec3::Zero();
    Vec3 moment_world = Vec3::Zero();
    // The same, body frame, as a ForceModule adds it.
    Wrench wrench;
    // Volume removed by HydrostaticParams::corrections at this pose, m^3.
    double correction_volume = 0.0;
    // Triangles wholly below the surface, and triangles cut by it.
    std::size_t submerged_triangles = 0;
    std::size_t cut_triangles = 0;
};

// Per-vertex working storage. Reuse one across calls on the same mesh to avoid allocating.
struct HydrostaticScratch {
    std::vector<Vec3> offsets;
    std::vector<double> depths;
};

// Integrates hydrostatic pressure rho*g*depth over the wetted part of a closed, outward-wound
// mesh. `pose.position` is the world position of params.reference_point and `pose.orientation`
// rotates mesh-frame vectors into the world; velocities are ignored.
//
// Depth is taken at the vertices and is linear across each (sub)triangle, which is exact for flat
// water and approximates a wave surface by its chord between vertices. Triangles cut by the
// surface are clipped at the interpolated waterline.
HydrostaticResult compute_hydrostatics(const TriMesh& mesh, const State& pose,
                                       const WaterSurface& water, double t,
                                       const HydrostaticParams& params,
                                       HydrostaticScratch& scratch);

// Flat water at z = water_level. Allocates its own scratch.
HydrostaticResult compute_hydrostatics(const TriMesh& mesh, const State& pose, double water_level,
                                       const HydrostaticParams& params = {});

// Hydrostatic buoyancy on one closed mesh. Allocates nothing per step.
class HydrostaticsModule final : public ForceModule {
public:
    // A null `water` means flat water at z = 0.
    HydrostaticsModule(TriMesh mesh, HydrostaticParams params,
                       std::shared_ptr<const WaterSurface> water = nullptr);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    // Result of the latest accumulate() call.
    const HydrostaticResult& last_result() const { return last_; }
    const TriMesh& mesh() const { return mesh_; }
    const HydrostaticParams& params() const { return params_; }

private:
    TriMesh mesh_;
    HydrostaticParams params_;
    std::shared_ptr<const WaterSurface> water_;
    HydrostaticScratch scratch_;
    HydrostaticResult last_;
};

}  // namespace sail
