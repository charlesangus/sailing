#pragma once

#include <array>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "sail/rigid_body.hpp"

namespace sail {

// Raised by load_boat_data. file() is the dossier file at fault, or the directory when the
// problem spans files.
class BoatDataError : public std::runtime_error {
public:
    BoatDataError(std::filesystem::path file, const std::string& message);

    const std::filesystem::path& file() const { return file_; }

private:
    std::filesystem::path file_;
};

// The hull frame is the frame of the dossier meshes: x forward, y to port, z up, origin on the
// centreline at mid-LWL on the measurement-trim flotation plane (the dossier's "body frame").
// All positions below are in it.

enum class MassShape {
    point,
    // Solid uniform box.
    box,
    // Thin uniform shell over all six faces of a box.
    shell_box,
};

struct MassItem {
    std::string name;
    // kg
    double mass = 0.0;
    // Hull frame, m.
    Vec3 cg = Vec3::Zero();
    MassShape shape = MassShape::point;
    // Box extents along x, y, z, centred on `cg`; zero for a point.
    Vec3 dims = Vec3::Zero();
    // Part of the certificate's measurement-trim displacement.
    bool in_displacement = true;
    // Mass and height were solved by the loader to close the displacement and VCG.
    bool residual = false;
};

// Strip-theory added mass, see strip_added_mass in mass_model.hpp.
struct AddedMassCoefficients {
    // Surge added mass as a fraction of the rigid-body mass.
    double surge_fraction = 0.05;
    // Multipliers on the strip-theory values.
    double sway_scale = 1.0;
    double heave_scale = 1.0;
    double roll_scale = 1.0;
    double pitch_scale = 1.0;
    double yaw_scale = 1.0;
    // Equivalent-prism length as a fraction of LWL; the hull's prismatic coefficient when unset.
    std::optional<double> length_factor;
};

struct HullDims {
    // m
    double lwl = 0.0;
    double bwl = 0.0;
    // Canoe-body draft.
    double tc = 0.0;
    // Prismatic coefficient.
    double cp = 0.0;
    // Waterplane coefficient at the DWL.
    double cw = 0.0;
    // Canoe-body LCB, hull frame x.
    double lcb_x_canoe = 0.0;
    // LCB of the floating meshes (canoe, keel, fairing, rudder) at the DWL, hull frame x.
    double lcb_x_total = 0.0;
};

struct CrewData {
    // kg
    double sailing_mass = 0.0;
    // ORC default crew plus the gear ORC adds for the stability calculation, as one mass.
    double orc_stability_mass = 0.0;
    // Hull frame x of the crew in every configuration.
    double x = 0.0;
    // Default sailing crew height above the sheer.
    double height_above_sheer = 0.0;
    // Hull frame z of the ORC stability crew and gear mass.
    double orc_stability_z = 0.0;
};

struct BoatData {
    std::filesystem::path directory;
    std::string name;
    // Provisional (stem) frame to hull frame: p_hull = p_prov + (stem_x_in_body, 0, flotation_z_in_body).
    double stem_x_in_body = 0.0;
    double flotation_z_in_body = 0.0;
    // Certificate measurement-trim displacement, kg.
    double displacement = 0.0;
    // Certificate VCGM, m above the flotation plane (ORC z up; negative is below it), which is
    // also its hull-frame z.
    double vcgm = 0.0;
    // Water density, kg/m^3.
    double density = 1025.0;
    // Certificate immersion at measurement trim, kg per mm of sinkage.
    double sink = 0.0;
    // Certificate righting moment at 1 degree heel in measurement trim, N*m.
    double righting_moment_1deg = 0.0;
    // Sheer fit z = c0 + c1 x + c2 x^2 in the hull frame, through the dossier's sheer points.
    std::array<double, 3> sheer = {0.0, 0.0, 0.0};
    HullDims hull;
    CrewData crew;
    AddedMassCoefficients added_mass;
    // Every mass item, including the solved residual, in file order.
    std::vector<MassItem> items;

    // Sheer height at hull frame x.
    double sheer_z(double x) const { return sheer[0] + x * (sheer[1] + x * sheer[2]); }
};

// Loads mass.json, particulars.json and hull_hydro.json from a boat dossier directory.
//
// Converts item CGs to the hull frame, then solves the one item marked "residual": its mass is
// the certificate displacement minus the other in-displacement items, its x makes the
// measurement-trim LCG equal the total LCB and its z makes the measurement-trim VCG equal the
// certificate VCGM. Heights the dossier gives above the flotation plane (VCGM, the ORC crew
// height, cg_z_bounds) are hull-frame z as they stand. Throws BoatDataError on a missing or
// malformed file, a missing or mistyped field, a non-positive sink, righting moment or Cw, a
// non-positive residual mass, a solved x more than 0.5 m from the canoe-body LCB, or a solved
// height outside the item's cg_z_bounds.
BoatData load_boat_data(const std::filesystem::path& boat_dir);

}  // namespace sail
