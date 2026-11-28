#pragma once

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

// Frames and units used throughout sailcore:
//   world: right-handed, x east, y north, z up.
//   body:  x forward, y to port, z up, origin at the centre of gravity.
// All quantities are SI (m, kg, s, rad, N, N*m).

namespace sail {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Quat = Eigen::Quaterniond;

// Kinematic state of one rigid body.
struct State {
    // CG position, world frame.
    Vec3 position = Vec3::Zero();
    // Rotates body-frame vectors into the world frame.
    Quat orientation = Quat::Identity();
    // CG velocity, body frame.
    Vec3 linear_velocity = Vec3::Zero();
    // Angular velocity, body frame.
    Vec3 angular_velocity = Vec3::Zero();

    // Body-to-world rotation matrix.
    Mat3 rotation() const { return orientation.toRotationMatrix(); }
    // CG velocity in the world frame.
    Vec3 world_linear_velocity() const { return orientation * linear_velocity; }
};

// rad, positive starboard down: the Euler roll atan2(R21, R22) of a body-to-world rotation, the
// heel that heel_trim_orientation (equilibrium.hpp) was given, whatever the trim.
inline double heel_angle(const Mat3& rotation) { return std::atan2(rotation(2, 1), rotation(2, 2)); }

// Force and moment acting on the body, both in the body frame, moment taken about the CG.
struct Wrench {
    Vec3 force = Vec3::Zero();
    Vec3 moment = Vec3::Zero();

    void clear() {
        force.setZero();
        moment.setZero();
    }
    Wrench& operator+=(const Wrench& other) {
        force += other.force;
        moment += other.moment;
        return *this;
    }
};

// Inertial properties. Generalised coordinates are ordered (surge, sway, heave, roll, pitch, yaw)
// in the body frame.
struct BodyProperties {
    // kg
    double mass = 1.0;
    // kg*m^2, body frame, about the CG.
    Mat3 inertia = Mat3::Identity();
    // Hydrodynamic added mass, same ordering and frame.
    Mat6 added_mass = Mat6::Zero();

    // Rigid-body 6x6 mass matrix. No linear/angular coupling because the origin is the CG.
    Mat6 rigid_mass() const;
    // Effective mass M_rb + M_A used by the integrator.
    Mat6 generalised_mass() const;
};

// Advances `state` by one step of length `dt` under `wrench`, which is held over the step.
//
// The velocity nu = (v, omega) solves the body-frame Kirchhoff equations with the full
// generalised mass M = M_rb + M_A, momentum p = M * nu = (p_lin, p_ang), at the step midpoint:
//   M (nu_new - nu) / dt = (F, Q) - C(nu_mid) nu_mid,  nu_mid = (nu + nu_new) / 2,
//   C(nu) nu = (omega x p_lin, omega x p_ang + v x p_lin),
// by Newton iteration. Position and orientation then advance with nu_new, and the quaternion is
// renormalised. With no wrench, the kinetic energy nu . M nu / 2, |p_lin| and, without added
// mass, |p_ang| are conserved to round-off for any spin axis.
//
// The added-mass Coriolis and Munk terms are applied here: force modules must not add them.
void integrate_step(const BodyProperties& body, const Wrench& wrench, double dt, State& state);

// C(nu) nu above for the generalised mass `m`: the Coriolis, centripetal and Munk terms the
// integrator subtracts from the applied wrench.
Vec6 gyroscopic(const Mat6& m, const Vec6& nu);

}  // namespace sail
