#pragma once

#include <memory>
#include <optional>

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <sail/boat_sim.hpp>
#include <sail/controls.hpp>

namespace sailgd {

// A sailing boat driven by the core BoatSim. The node's transform follows the hull's CG, blended
// between the last two fixed steps, with the frame change of frames.hpp.
//
// Every bound method catches what the core throws: an exception must never unwind into Godot,
// whose own code is built without exception support. A failure is reported with push_error and
// get_last_error(), and leaves the node inert until boat_dir or scene_dir is set again.
//
// scene_dir names a scene directory (res://data/scenes/<name>) whose meta.json and heightmap give
// the seabed the boat can run aground on; empty for none.
class SailBoat : public godot::Node3D {
    GDCLASS(SailBoat, godot::Node3D)

public:
    SailBoat();
    ~SailBoat() override;

    void _process(double delta) override;

    void set_boat_dir(const godot::String& dir);
    godot::String get_boat_dir() const { return boat_dir_; }
    void set_scene_dir(const godot::String& dir);
    godot::String get_scene_dir() const { return scene_dir_; }

    void set_wind_speed_kn(double knots);
    double get_wind_speed_kn() const;
    void set_wind_direction_deg(double from_deg);
    double get_wind_direction_deg() const { return wind_from_deg_; }

    void set_tiller(double v);
    double get_tiller() const { return controls_.tiller; }
    void set_mainsheet(double v);
    double get_mainsheet() const { return controls_.mainsheet; }
    void set_traveler(double v);
    double get_traveler() const { return controls_.traveler; }
    void set_backstay(double v);
    double get_backstay() const { return controls_.backstay; }
    void set_jib_sheet_port(double v);
    double get_jib_sheet_port() const { return controls_.jib_sheet_port; }
    void set_jib_sheet_starboard(double v);
    double get_jib_sheet_starboard() const { return controls_.jib_sheet_starboard; }
    void set_jib_car(double v);
    double get_jib_car() const { return controls_.jib_car; }
    void set_reef(double v);
    double get_reef() const { return controls_.reef; }

    bool set_controls_from_dictionary(const godot::Dictionary& controls);
    godot::Dictionary get_controls() const;

    bool reset_boat(const godot::Vector3& position, double heading_deg, double speed_mps = 0.0);
    int step_seconds(double seconds);

    bool is_active() const { return boat_ != nullptr; }
    godot::String get_last_error() const { return last_error_; }
    godot::Dictionary get_state() const;

protected:
    static void _bind_methods();

private:
    void load();
    void apply_wind();
    void apply_controls(const sail::Controls& controls);
    void update_transform();
    void fail(const char* what, const char* detail);
    template <typename F>
    bool guarded(const char* what, F&& body);

    godot::String boat_dir_;
    godot::String scene_dir_;
    std::shared_ptr<const sail::Bathymetry> bathymetry_;
    std::unique_ptr<sail::BoatSim> boat_;
    std::shared_ptr<const sail::WindField> wind_;
    double wind_speed_mps_ = 0.0;
    double wind_from_deg_ = 0.0;
    sail::Controls controls_;
    std::optional<std::size_t> main_index_;
    std::optional<std::size_t> jib_index_;
    godot::String last_error_;
};

}  // namespace sailgd
