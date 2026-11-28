#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "sail_boat.hpp"
#include "sail_sim.hpp"

namespace {

void initialize_sailgd(godot::ModuleInitializationLevel level) {
    if (level != godot::MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    GDREGISTER_CLASS(sailgd::SailSim);
    GDREGISTER_CLASS(sailgd::SailBoat);
}

void uninitialize_sailgd(godot::ModuleInitializationLevel) {}

}  // namespace

extern "C" {

GDExtensionBool GDE_EXPORT sailgd_library_init(GDExtensionInterfaceGetProcAddress get_proc_address,
                                               GDExtensionClassLibraryPtr library,
                                               GDExtensionInitialization* initialization) {
    godot::GDExtensionBinding::InitObject init(get_proc_address, library, initialization);
    init.register_initializer(initialize_sailgd);
    init.register_terminator(uninitialize_sailgd);
    init.set_minimum_library_initialization_level(godot::MODULE_INITIALIZATION_LEVEL_SCENE);
    return init.init();
}

}
