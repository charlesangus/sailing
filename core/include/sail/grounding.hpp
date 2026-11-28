#pragma once

#include <cstddef>
#include <filesystem>
#include <limits>
#include <memory>
#include <span>
#include <vector>

#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace sail {

// Seabed height and its slope at one world (x, y).
struct SeabedSample {
    // World z of the seabed, m.
    double height = -std::numeric_limits<double>::infinity();
    // dz/dx and dz/dy.
    double dzdx = 0.0;
    double dzdy = 0.0;

    // Upward unit normal of the seabed surface, world frame.
    Vec3 normal() const { return Vec3(-dzdx, -dzdy, 1.0).normalized(); }
};

// Seabed elevation on a regular grid, world frame, sampled bilinearly between pixel centres.
// Pixel (col, row) is centred on world (origin_x + col * pixel_size, origin_y - row * pixel_size):
// columns run east and rows south, row 0 the northern edge. Outside the grid col and row are
// clamped to its edge, so the edge heights extend outwards and the slope across them is zero.
//
// A default-constructed Bathymetry has no seabed: height() is -infinity everywhere.
class Bathymetry {
public:
    Bathymetry() = default;
    // `heights` is row-major, width * rows values in metres. Throws std::invalid_argument on an
    // empty grid, a size mismatch, a non-positive or non-finite pixel size or a non-finite height.
    Bathymetry(std::size_t width, std::size_t rows, double pixel_size, double origin_x,
               double origin_y, std::vector<float> heights);

    // Reads a scene's meta.json (data/scenes/<name>/meta.json) and the 16-bit greyscale PNG it
    // names, beside it, decoding each sample with its height_encoding. Throws BoatDataError on a
    // bad meta.json and std::runtime_error on an unreadable or mismatched image.
    static Bathymetry load(const std::filesystem::path& meta_json);

    bool has_seabed() const { return !heights_.empty(); }
    std::size_t width() const { return width_; }
    std::size_t rows() const { return rows_; }
    double pixel_size() const { return pixel_; }
    double origin_x() const { return origin_x_; }
    double origin_y() const { return origin_y_; }

    // World z of the seabed at world (x, y), m.
    double height(double x, double y) const { return sample(x, y).height; }
    SeabedSample sample(double x, double y) const;
    // An upper bound on height() over the world rectangle [x_min, x_max] x [y_min, y_max]: the
    // highest grid node of the cells it touches, which bounds the bilinear surface there.
    double max_height(double x_min, double x_max, double y_min, double y_max) const;

private:
    double node(std::size_t col, std::size_t row) const {
        return static_cast<double>(heights_[row * width_ + col]);
    }

    std::size_t width_ = 0;
    std::size_t rows_ = 0;
    double pixel_ = 1.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;
    std::vector<float> heights_;
};

// Keeps, of the points falling in each cube of side `cell` (aligned on the origin), the one
// lowest in z, so a decimated hull still reaches as deep as the full one. Output in first-seen
// cell order. Throws std::invalid_argument on a non-positive cell.
std::vector<Vec3> decimate_lowest(std::span<const Vec3> points, double cell);

// The hull-frame points a boat dossier touches the seabed with: hull_physics.obj decimated at
// 0.3 m, and the keel and rudder of appendages.obj at 0.08 m, because the keel's bulb and tip
// touch first. Throws ObjError on a bad mesh.
std::vector<Vec3> load_contact_points(const std::filesystem::path& boat_dir);

struct GroundingParams {
    // Hull-frame point the state's position refers to (the CG); moments are taken about it.
    Vec3 reference_point = Vec3::Zero();
    // kg, the mass the contact's stiffness and damping are sized for.
    double mass = 1.0;
    // Hz. The undamped frequency of `mass` on the full contact stiffness, kept well below the
    // fixed-step rate (1/20 of it) because a contact low on the keel moves a smaller effective
    // mass than the whole boat, once the roll and pitch it drives are counted.
    double natural_frequency = 0.05 / Simulation::kFixedDt;
    // Damping as a fraction of critical for `mass` on the full stiffness.
    double damping_ratio = 0.5;
    // Coulomb friction coefficient, wet sand.
    double friction = 0.5;
    // m/s. Friction is mu N v / sqrt(v^2 + slip_speed^2) of the tangential slip v, so it has no
    // discontinuity at rest. Its slope there, mu N / slip_speed, is a damper the explicit step
    // has to stay stable with, which sets the floor on this.
    double slip_speed = 0.1;
    // The number of fully pressed contacts that share the full stiffness: fewer each get
    // stiffness / shared_contacts, more share it, so the total never exceeds the stiffness.
    double shared_contacts = 4.0;
    // m. A contact counts towards shared_contacts, and its damping fades in, over this depth.
    double engage_depth = 0.005;

    // N/m and N*s/m, the full contact's.
    double stiffness() const;
    double damping() const;
};

struct GroundingResult {
    // False when the broad phase ruled out any contact.
    bool tested = false;
    std::size_t contacts = 0;
    // m, the deepest penetration along the seabed normal.
    double max_penetration = 0.0;
    // N, sum of the normal forces' magnitudes.
    double normal_force = 0.0;
    // World frame, about the reference point.
    Vec3 force_world = Vec3::Zero();
    Vec3 moment_world = Vec3::Zero();
    Wrench wrench;
};

// Contact between the boat and the seabed. Each contact point below the seabed gets a spring and
// damper along the seabed normal and Coulomb friction against its tangential velocity.
//
// Broad phase: the contact points' hull-frame bounding box is rotated into the world each step,
// and when the seabed's max_height() under its footprint is below its lowest corner no point can
// touch, so nothing else is done. Allocates nothing per step.
class GroundingModule final : public ForceModule {
public:
    // A null `seabed` means none: the module adds nothing. Throws std::invalid_argument on
    // non-positive mass, frequency, slip speed, shared contacts or engage depth, or a negative
    // damping ratio or friction.
    GroundingModule(std::vector<Vec3> points, GroundingParams params,
                    std::shared_ptr<const Bathymetry> seabed = nullptr);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    const GroundingResult& last_result() const { return last_; }
    const GroundingParams& params() const { return params_; }
    std::span<const Vec3> points() const { return points_; }
    const std::shared_ptr<const Bathymetry>& seabed() const { return seabed_; }
    void set_seabed(std::shared_ptr<const Bathymetry> seabed) { seabed_ = std::move(seabed); }

    // Moves the reference point (hull frame); see Simulation::set_body.
    void set_reference_point(const Vec3& point) { params_.reference_point = point; }

private:
    std::vector<Vec3> points_;
    Vec3 box_min_ = Vec3::Zero();
    Vec3 box_max_ = Vec3::Zero();
    GroundingParams params_;
    double stiffness_ = 0.0;
    double damping_ = 0.0;
    std::shared_ptr<const Bathymetry> seabed_;
    GroundingResult last_;
};

}  // namespace sail
