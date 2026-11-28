#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "sail/mass_model.hpp"
#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"
#include "sail/wind.hpp"

namespace sail {

enum class WindageGroup : std::size_t {
    mast,
    boom,
    rigging,
    hull,
    crew,
};
inline constexpr std::size_t kWindageGroupCount = 5;

// A box of three axis-aligned face areas about one point, hull frame. Its drag area against the
// unit apparent wind n (body frame) is |nx| cd.x area.x + |ny| cd.y area.y + |nz| cd.z area.z, so
// the head and side Cd of ORC's table 5.10 blend with the wind angle as in its eq. 5.26.
struct WindageBox {
    std::string name;
    WindageGroup group = WindageGroup::mast;
    Vec3 centre = Vec3::Zero();
    // m^2 seen along body x, y and z.
    Vec3 area = Vec3::Zero();
    Vec3 cd = Vec3::Zero();
};

// A straight wire of diameter d and length |b - a|, drag area d |b - a| sin(angle to the wind),
// at its midpoint.
struct WindageWire {
    std::string name;
    Vec3 a = Vec3::Zero();
    Vec3 b = Vec3::Zero();
    // m
    double diameter = 0.0;
    // Includes ORC's spreader factor.
    double cd = 0.0;
};

// A flat one-sided panel. It takes drag only while its outward normal faces the wind, on its
// area above the water, projected normal to the wind.
struct WindagePanel {
    std::string name;
    // Hull frame, in order around the outline.
    std::array<Vec3, 4> corners;
    // Hull-frame axes, unit.
    Vec3 outward = Vec3::UnitY();
    double cd = 0.0;
};

enum class CrewPosture {
    sitting,
    hiking,
};

struct WindageCrew {
    // People aboard; 0 for none.
    int count = 0;
    // Hull frame, the crew's centre.
    Vec3 position = Vec3::Zero();
    CrewPosture posture = CrewPosture::sitting;
    double cd = 0.0;
    // m^2 per person seen along body x (front) and y (side), sitting and hiking.
    double sitting_front = 0.0;
    double sitting_side = 0.0;
    double hiking_front = 0.0;
    double hiking_side = 0.0;
};

struct WindageParams {
    // kg/m^3, the SailSet default.
    double air_density = 1.225;
    // Calibration factor on every element's drag (Calibration::windage).
    double drag_scale = 1.0;
    // Hull-frame location of the body's reference point (the CG).
    Vec3 reference_point = Vec3::Zero();
    std::vector<WindageBox> boxes;
    std::vector<WindageWire> wires;
    std::vector<WindagePanel> panels;
    WindageCrew crew;
};

// Reads windage.json, rig.json, particulars.json, hull_hydro.json, mass.json and the optional
// calibration.json of a boat dossier directory; windage.json describes how the elements are
// built. No crew is aboard.
// Throws BoatDataError on a missing or malformed file or field.
WindageParams load_windage_params(const std::filesystem::path& boat_dir,
                                  const Vec3& reference_point);

// The same with the CG, crew position and crew presence (crew_mass() > 0) of `model`.
WindageParams load_windage_params(const MassModel& model);

// What the latest accumulate() computed.
struct WindageResult {
    // Body frame, about the CG, N and N*m.
    std::array<Wrench, kWindageGroupCount> groups;
    Wrench wrench;
    // m^2 above the water, of the panels facing outboard on each side whether or not the wind
    // reaches them, and of the bow and stern panels.
    double topsides_port = 0.0;
    double topsides_starboard = 0.0;
    double topsides_bow = 0.0;
    double topsides_stern = 0.0;
};

// Drag of the mast, boom, standing rigging, topsides and crew in the apparent wind at each
// element's own centre, F = 0.5 rho Cd A |V| V with V the air velocity relative to the element.
// The water surface is world z = 0. Allocates nothing per step.
class WindageModule final : public ForceModule {
public:
    // Throws std::invalid_argument on a null `wind` or a non-positive density.
    WindageModule(WindageParams params, std::shared_ptr<const WindField> wind);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    const WindageResult& last_result() const { return last_; }
    const WindageParams& params() const { return params_; }

    void set_reference_point(const Vec3& point) { params_.reference_point = point; }
    void set_wind(std::shared_ptr<const WindField> wind);
    // Throws std::invalid_argument on a negative count.
    void set_crew(int count, const Vec3& position, CrewPosture posture);

private:
    WindageParams params_;
    std::shared_ptr<const WindField> wind_;
    WindageResult last_;
};

}  // namespace sail
