#pragma once

#include <cmath>

#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <sail/rigid_body.hpp>

// The one place core frames meet Godot's. Every other file in the extension works in core frames
// or in Godot frames, never both.
//
// World. Core: x east, y north, z up. Godot: +Y up and -Z forward, with north taken as -Z so a
// node facing its default -Z faces north:
//   godot.x =  core.x   (east)
//   godot.y =  core.z   (up)
//   godot.z = -core.y   (south)
//
// Body. Core: x forward, y port, z up. Godot node local axes: -Z forward, +X starboard, +Y up:
//   local.x = -body.y   (starboard)
//   local.y =  body.z   (up)
//   local.z = -body.x   (aft)
//
// Both maps are proper rotations, so handedness is kept and a core rotation R (body to world)
// becomes the node basis W * R * B^T, W and B the world and body maps above. A positive core
// heel (starboard down) is a rotation of +heel about the node's forward axis, local -Z, so the
// node's roll about its local +Z (rotation.z in Godot's YXZ Euler order) reads -heel.
//
// At the identity core orientation the bow points east, Godot +X.
namespace sailgd::frames {

inline godot::Vector3 world_to_godot(const sail::Vec3& v) {
    return godot::Vector3(static_cast<godot::real_t>(v.x()), static_cast<godot::real_t>(v.z()),
                          static_cast<godot::real_t>(-v.y()));
}

inline sail::Vec3 godot_to_world(const godot::Vector3& v) {
    return sail::Vec3(static_cast<double>(v.x), -static_cast<double>(v.z), static_cast<double>(v.y));
}

inline godot::Vector3 body_to_local(const sail::Vec3& v) {
    return godot::Vector3(static_cast<godot::real_t>(-v.y()), static_cast<godot::real_t>(v.z()),
                          static_cast<godot::real_t>(-v.x()));
}

inline sail::Mat3 world_map() {
    sail::Mat3 m;
    m << 1.0, 0.0, 0.0,
         0.0, 0.0, 1.0,
         0.0, -1.0, 0.0;
    return m;
}

inline sail::Mat3 body_map() {
    sail::Mat3 m;
    m << 0.0, -1.0, 0.0,
         0.0, 0.0, 1.0,
         -1.0, 0.0, 0.0;
    return m;
}

inline godot::Basis to_basis(const sail::Mat3& g) {
    return godot::Basis(static_cast<godot::real_t>(g(0, 0)), static_cast<godot::real_t>(g(0, 1)),
                        static_cast<godot::real_t>(g(0, 2)), static_cast<godot::real_t>(g(1, 0)),
                        static_cast<godot::real_t>(g(1, 1)), static_cast<godot::real_t>(g(1, 2)),
                        static_cast<godot::real_t>(g(2, 0)), static_cast<godot::real_t>(g(2, 1)),
                        static_cast<godot::real_t>(g(2, 2)));
}

inline godot::Basis rotation_to_godot(const sail::Quat& orientation) {
    return to_basis(world_map() * orientation.toRotationMatrix() * body_map().transpose());
}

// Unit horizontal core world vector towards a compass bearing, clockwise from north.
inline sail::Vec3 compass_to_world(double bearing_rad) {
    return sail::Vec3(std::sin(bearing_rad), std::cos(bearing_rad), 0.0);
}

// The node's forward axis, local -Z, as a core world vector.
inline sail::Vec3 forward_to_world(const godot::Basis& basis) {
    return godot_to_world(-basis.get_column(2));
}

inline godot::Transform3D state_to_godot(const sail::Vec3& position, const sail::Quat& orientation) {
    return godot::Transform3D(rotation_to_godot(orientation), world_to_godot(position));
}

}  // namespace sailgd::frames
