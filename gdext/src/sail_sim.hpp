#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/string.hpp>

namespace sailgd {

class SailSim : public godot::Node {
    GDCLASS(SailSim, godot::Node)

public:
    godot::String get_core_version() const;

protected:
    static void _bind_methods();
};

}  // namespace sailgd
