#pragma once

#include <filesystem>
#include <optional>
#include <vector>

#include "sail/hydrostatics.hpp"
#include "sail/mass_model.hpp"
#include "sail/mesh.hpp"
#include "sail/rigid_body.hpp"

namespace sail {

// The floating geometry of a boat dossier: canoe body and appendages as one mesh in the hull
// frame, with the corrections for volume those meshes count twice.
struct BoatHydrostatics {
    TriMesh mesh;
    std::vector<VolumeCorrection> corrections;

    // Parameters for `model`: its water density, its CG as the reference point, and the
    // corrections above.
    HydrostaticParams params(const MassModel& model, double gravity = 9.81) const;
};

// Loads hull_physics.obj and appendages.obj from `boat_dir`, and the keel-root overlap volume
// from appendages.json (foils.keel.root_overlap). Throws BoatDataError on a bad appendages.json
// and ObjError on a bad mesh.
BoatHydrostatics load_boat_hydrostatics(const std::filesystem::path& boat_dir);

// Body-to-world rotation for roll `heel` about the body x axis followed by pitch `trim` about the
// world y axis (Euler roll-pitch with zero yaw), radians. Positive heel puts starboard down and
// positive trim puts the bow down.
Quat heel_trim_orientation(double heel, double trim);

struct EquilibriumRequest {
    // rad, always held.
    double heel = 0.0;
    // rad. Held when free_trim is false, otherwise the starting guess.
    double trim = 0.0;
    bool free_trim = true;
    // Starting world z of the reference point; params.reference_point.z() (hull-frame origin on
    // the water) when unset.
    std::optional<double> z;
};

struct Equilibrium {
    // World x and y of the reference point are its hull-frame x and y; velocities are zero.
    State pose;
    // rad
    double heel = 0.0;
    double trim = 0.0;
    HydrostaticResult hydrostatics;
    // N*m about the world x axis, the horizontal through the reference point under the boat's
    // centreline, positive when it opposes positive heel.
    double righting_moment = 0.0;
    bool converged = false;
    int iterations = 0;
};

// Solves for the flat-water pose (water at z = 0) at which buoyancy carries `mass` at the held
// heel: Newton on the reference point's height, and on trim when request.free_trim, for zero net
// vertical force and, with free trim, zero moment about the world y axis.
Equilibrium solve_equilibrium(const TriMesh& mesh, const HydrostaticParams& params, double mass,
                              const EquilibriumRequest& request = {});

}  // namespace sail
