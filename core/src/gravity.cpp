#include "sail/gravity.hpp"

namespace sail {

GravityModule::GravityModule(double mass, double gravity) : mass_(mass), gravity_(gravity) {}

void GravityModule::accumulate(const State& state, double, Wrench& wrench) {
    wrench.force += state.orientation.conjugate() * Vec3(0.0, 0.0, -mass_ * gravity_);
}

}  // namespace sail
