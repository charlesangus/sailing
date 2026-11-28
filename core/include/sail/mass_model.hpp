#pragma once

#include <filesystem>
#include <span>

#include "sail/boat_data.hpp"
#include "sail/rigid_body.hpp"

namespace sail {

struct MassProperties {
    // kg
    double mass = 0.0;
    // Hull frame, m.
    Vec3 cg = Vec3::Zero();
    // kg*m^2, about `cg`, hull-frame axes.
    Mat3 inertia = Mat3::Zero();
};

// Inertia of one item about its own CG, from its shape and dims.
Mat3 item_inertia(const MassItem& item);

// Total mass, CG and inertia about that CG, by the parallel-axis theorem. Throws
// std::invalid_argument if the total mass is not positive.
MassProperties combine(std::span<const MassItem> items);

// Diagonal 6x6 added mass in (surge, sway, heave, roll, pitch, yaw).
//
// Surge is coeffs.surge_fraction * rigid_mass. The others are strip theory in the high-frequency
// limit with every section a half-ellipse of half-beam b = BWL/2 and draft T = Tc, so each is half
// the double-body ellipse value: per metre, sway rho pi T^2/2, heave rho pi b^2/2 and roll about
// the waterline centre rho pi (b^2 - T^2)^2/16. They are integrated over an equivalent prism of
// length L = length_factor * LWL (default Cp * LWL), with pitch and yaw from heave and sway times
// L^2/12. Rotational terms are about the waterline centre, not the CG, and leave out the keel.
Mat6 strip_added_mass(const HullDims& hull, const AddedMassCoefficients& coeffs, double density,
                      double rigid_mass);

enum class MassConfiguration {
    // Certificate measurement trim: in-displacement items only, no crew.
    measurement_trim,
    // Every item plus the class-maximum crew at its default seated position.
    sailing,
    // In-displacement items plus ORC's default crew and gear as one centreline point mass.
    orc_stability,
};

// Mass properties of a boat with one movable crew point mass. Crew is never part of the
// measurement-trim displacement.
class MassModel {
public:
    explicit MassModel(BoatData data,
                       MassConfiguration configuration = MassConfiguration::measurement_trim);

    static MassModel load(const std::filesystem::path& boat_dir,
                          MassConfiguration configuration = MassConfiguration::measurement_trim);

    // Selects the items carried and resets the crew mass and position to the configuration's
    // defaults.
    void set_configuration(MassConfiguration configuration);
    MassConfiguration configuration() const { return configuration_; }

    // Moves the crew in the hull frame's y-z plane, keeping its x. z is hull-frame z.
    void set_crew_position(double y, double z);
    // kg, zero for no crew. Throws std::invalid_argument if negative.
    void set_crew_mass(double mass);
    double crew_mass() const { return crew_.mass; }
    // Hull frame.
    const Vec3& crew_position() const { return crew_.cg; }

    // Whole boat including crew.
    const MassProperties& properties() const { return total_; }
    double total_mass() const { return total_.mass; }
    // Hull frame; pass as HydrostaticParams::reference_point.
    const Vec3& cg() const { return total_.cg; }
    const Mat3& inertia() const { return total_.inertia; }
    Mat6 added_mass() const;
    // Mass, inertia about the CG and added mass, for the integrator.
    BodyProperties body_properties() const;

    const BoatData& data() const { return data_; }

private:
    void recompute();

    BoatData data_;
    MassConfiguration configuration_;
    MassItem crew_;
    MassProperties total_;
};

}  // namespace sail
