#include "sail_sim.hpp"

#include <godot_cpp/core/class_db.hpp>

#include <sail/version.hpp>

namespace sailgd {

godot::String SailSim::get_core_version() const {
    const std::string_view version = sail::version();
    return godot::String::utf8(version.data(), static_cast<int64_t>(version.size()));
}

void SailSim::_bind_methods() {
    godot::ClassDB::bind_method(godot::D_METHOD("get_core_version"), &SailSim::get_core_version);
}

}  // namespace sailgd
