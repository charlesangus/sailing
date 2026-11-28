#pragma once

#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"

namespace sail {

// Gravitational acceleration used throughout the core, m/s^2.
inline constexpr double kGravity = 9.81;

// Weight m * g along world -z, applied at the CG.
class GravityModule final : public ForceModule {
public:
    // `mass` in kg, `gravity` in m/s^2.
    explicit GravityModule(double mass, double gravity = kGravity);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    // For a mass change at run time, e.g. crew added or removed.
    void set_mass(double mass) { mass_ = mass; }
    double mass() const { return mass_; }
    double gravity() const { return gravity_; }

private:
    double mass_;
    double gravity_;
};

}  // namespace sail
