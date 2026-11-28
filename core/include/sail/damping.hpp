#pragma once

#include <array>
#include <filesystem>

#include "sail/boat_data.hpp"
#include "sail/mass_model.hpp"
#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace sail {

enum class DampingDof { heave = 0, roll = 1, pitch = 2 };

// Force law -(linear * v + quadratic * |v| * v) for one DOF.
struct DampingCoefficient {
    // N*s/m for heave, N*m*s/rad for roll and pitch.
    double linear = 0.0;
    // N*s^2/m^2 for heave, N*m*s^2/rad^2 for roll and pitch.
    double quadratic = 0.0;
};

struct DampingCoefficients {
    DampingCoefficient heave;
    DampingCoefficient roll;
    DampingCoefficient pitch;

    const DampingCoefficient& of(DampingDof dof) const;
    DampingCoefficient& of(DampingDof dof);
};

// One DOF's damping as fractions of critical damping, as stored in damping.json.
struct DampingRatio {
    double zeta_linear = 0.0;
    // Quadratic damping expressed as the damper ratio it matches at `reference_velocity`.
    double zeta_quadratic = 0.0;
    // m/s for heave, rad/s for roll and pitch.
    double reference_velocity = 1.0;
};

struct DampingSpec {
    DampingRatio heave;
    DampingRatio roll;
    DampingRatio pitch;
};

// Restoring stiffness per DOF, from the dossier alone.
struct DampingStiffness {
    // N/m
    double heave = 0.0;
    // N*m/rad
    double roll = 0.0;
    // N*m/rad
    double pitch = 0.0;
};

// Reads damping.json from a boat dossier directory. Throws BoatDataError on a missing or
// malformed file, a missing or mistyped field, a negative ratio or a non-positive reference
// velocity.
DampingSpec load_damping_spec(const std::filesystem::path& boat_dir);

// Heave k = rho g A_wp from the certificate immersion, roll k = RM(1 deg) / sin(1 deg) from the
// certificate righting moment, pitch k = rho g I_L with I_L = Cw^2 BWL LWL^3 / 12, all from the
// measurement-trim values load_boat_data read.
DampingStiffness dossier_stiffness(const BoatData& data);

// b_crit = 2 sqrt(k (M + A)).
double critical_damping(double stiffness, double effective_mass);

// Absolute coefficients: linear = zeta_linear b_crit and quadratic = zeta_quadratic b_crit /
// reference_velocity, with M + A taken from the diagonal of body.generalised_mass() for each
// DOF.
DampingCoefficients derive_damping(const DampingSpec& spec, const DampingStiffness& stiffness,
                                   const BodyProperties& body);

// Loads damping.json from the model's boat directory and derives the coefficients for the
// model's current mass properties.
DampingCoefficients load_damping(const MassModel& model);

// Damping of heave, roll and pitch, measured against the water surface rather than the hull.
//
// Heave is the CG's world-vertical velocity and its force acts along world z; roll and pitch are
// the components of the world angular velocity about the horizontal heading axis x_h and the
// horizontal axis 90 degrees to port of it, y_h, and their moments act about those same axes.
// With yaw-pitch-roll angles these rates are phi_dot cos(theta) and theta_dot. Body-frame rates
// would damp motions with no restoring stiffness behind them: at a trim angle or heeled with
// leeway, steady forward motion has a body-z velocity, and in a heeled turn body q picks up
// r sin(phi). Each force or moment acts along the rate it damps, so the module only ever
// removes energy.
class DampingModule final : public ForceModule {
public:
    explicit DampingModule(const DampingCoefficients& coefficients);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    // Scales both coefficients of one DOF, or of all three. Throws std::invalid_argument if
    // negative or not finite.
    void set_multiplier(DampingDof dof, double multiplier);
    void set_multipliers(double multiplier);
    double multiplier(DampingDof dof) const;

    // Unscaled coefficients.
    const DampingCoefficients& coefficients() const { return coefficients_; }

private:
    DampingCoefficients coefficients_;
    std::array<double, 3> multipliers_ = {1.0, 1.0, 1.0};
};

}  // namespace sail
