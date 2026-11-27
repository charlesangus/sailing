# Stack: Godot 4 front end, C++20 physics core via godot-cpp, Linux only

The demo uses Godot 4 for rendering, input and UI, and a C++20 physics core (`sailcore`) with no Godot dependency, wrapped by a godot-cpp GDExtension. The core is testable headless through doctest and a `sailsim` CLI harness. Builds and exports are Linux only. Rationale: the user chose Godot plus a native core for open source, light iteration and testable physics, and C++ over Rust for familiarity. Linux only keeps CI and packaging small; the core stays portable.
