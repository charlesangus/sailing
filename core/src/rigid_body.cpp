#include "sail/rigid_body.hpp"

#include <Eigen/Cholesky>
#include <cmath>

namespace sail {

Mat6 BodyProperties::rigid_mass() const {
    Mat6 m = Mat6::Zero();
    m.topLeftCorner<3, 3>() = mass * Mat3::Identity();
    m.bottomRightCorner<3, 3>() = inertia;
    return m;
}

Mat6 BodyProperties::generalised_mass() const { return rigid_mass() + added_mass; }

void integrate_step(const BodyProperties& body, const Wrench& wrench, double dt, State& state) {
    const Vec3& v = state.linear_velocity;
    const Vec3& w = state.angular_velocity;

    Vec6 nu;
    nu << v, w;
    const Mat6 m = body.generalised_mass();
    const Vec6 p = m * nu;
    const Vec3 p_lin = p.head<3>();
    const Vec3 p_ang = p.tail<3>();

    // Coriolis and centripetal terms come from the total momentum, so the added-mass
    // contribution (including the Munk moment v x p_lin) is included rather than neglected:
    // an ideal-fluid body in a rotating frame feels it, and dropping it would let a later
    // hydrodynamic module see an inconsistent inertia. For the rigid part alone the v x m*v
    // term vanishes and this reduces to omega x m*v and omega x I*omega.
    Vec6 rhs;
    rhs.head<3>() = wrench.force - w.cross(p_lin);
    rhs.tail<3>() = wrench.moment - w.cross(p_ang) - v.cross(p_lin);

    const Vec6 nu_dot = m.ldlt().solve(rhs);
    const Vec3 v_new = v + dt * nu_dot.head<3>();
    const Vec3 w_new = w + dt * nu_dot.tail<3>();

    state.position += dt * (state.orientation * v_new);

    // Body-frame angular velocity composes on the right; the exact exponential keeps the
    // update a pure rotation for any step size.
    const double angle = w_new.norm() * dt;
    if (angle > 0.0) {
        const Quat dq(Eigen::AngleAxisd(angle, w_new.normalized()));
        state.orientation = state.orientation * dq;
    }
    state.orientation.normalize();

    state.linear_velocity = v_new;
    state.angular_velocity = w_new;
}

}  // namespace sail
