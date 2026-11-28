#include "sail/rigid_body.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <cmath>

namespace sail {

namespace {

Mat3 skew(const Vec3& a) {
    Mat3 s;
    s << 0.0, -a.z(), a.y(), a.z(), 0.0, -a.x(), -a.y(), a.x(), 0.0;
    return s;
}

Mat6 gyroscopic_jacobian(const Mat6& m, const Vec6& nu) {
    const Vec6 p = m * nu;
    const Mat3 pl = skew(p.head<3>());
    Mat6 by_velocity = Mat6::Zero();
    by_velocity.topRightCorner<3, 3>() = -pl;
    by_velocity.bottomLeftCorner<3, 3>() = -pl;
    by_velocity.bottomRightCorner<3, 3>() = -skew(p.tail<3>());
    Mat6 by_momentum = Mat6::Zero();
    by_momentum.topLeftCorner<3, 3>() = skew(nu.tail<3>());
    by_momentum.bottomLeftCorner<3, 3>() = skew(nu.head<3>());
    by_momentum.bottomRightCorner<3, 3>() = skew(nu.tail<3>());
    return by_velocity + by_momentum * m;
}

constexpr int kMaxNewtonIterations = 8;
constexpr double kNewtonTolerance = 1e-13;

}  // namespace

Vec6 gyroscopic(const Mat6& m, const Vec6& nu) {
    const Vec6 p = m * nu;
    const Vec3 v = nu.head<3>();
    const Vec3 w = nu.tail<3>();
    Vec6 c;
    c.head<3>() = w.cross(p.head<3>());
    c.tail<3>() = w.cross(p.tail<3>()) + v.cross(p.head<3>());
    return c;
}

Mat6 BodyProperties::rigid_mass() const {
    Mat6 m = Mat6::Zero();
    m.topLeftCorner<3, 3>() = mass * Mat3::Identity();
    m.bottomRightCorner<3, 3>() = inertia;
    return m;
}

Mat6 BodyProperties::generalised_mass() const { return rigid_mass() + added_mass; }

void integrate_step(const BodyProperties& body, const Wrench& wrench, double dt, State& state) {
    Vec6 nu;
    nu << state.linear_velocity, state.angular_velocity;
    Vec6 tau;
    tau << wrench.force, wrench.moment;
    const Mat6 m = body.generalised_mass();

    // An explicit gyroscopic term pumps energy into any spin off a principal axis. At the
    // midpoint velocity it does no work, nu_mid . C(nu_mid) nu_mid = 0, so kinetic energy and
    // the momentum norms are conserved to the Newton tolerance.
    Vec6 next = nu + dt * m.ldlt().solve(tau - gyroscopic(m, nu));
    for (int i = 0; i < kMaxNewtonIterations; ++i) {
        const Vec6 mid = 0.5 * (nu + next);
        const Vec6 residual = m * (next - nu) + dt * (gyroscopic(m, mid) - tau);
        const Mat6 jacobian = m + 0.5 * dt * gyroscopic_jacobian(m, mid);
        const Vec6 delta = jacobian.partialPivLu().solve(residual);
        next -= delta;
        if (delta.norm() <= kNewtonTolerance * (1.0 + next.norm())) break;
    }
    const Vec3 v_new = next.head<3>();
    const Vec3 w_new = next.tail<3>();

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
