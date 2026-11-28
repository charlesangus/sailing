#include "sail_boat.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <numbers>
#include <string>
#include <utility>

#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <sail/foil.hpp>
#include <sail/harness/autopilot.hpp>
#include <sail/wind.hpp>

#include "frames.hpp"

namespace sailgd {

namespace {

constexpr double kKnot = 1852.0 / 3600.0;
constexpr double kDeg = std::numbers::pi / 180.0;

double to_deg(double rad) { return rad / kDeg; }

double wrap_360(double deg) {
    const double w = std::fmod(deg, 360.0);
    return w < 0.0 ? w + 360.0 : w;
}

bool finite(const sail::State& s) {
    return s.position.allFinite() && s.orientation.coeffs().allFinite() &&
           s.linear_velocity.allFinite() && s.angular_velocity.allFinite();
}

std::filesystem::path to_path(const godot::String& s) {
    const godot::CharString utf8 = s.utf8();
    return std::filesystem::path(std::u8string(reinterpret_cast<const char8_t*>(utf8.get_data())));
}

godot::String globalize(const godot::String& dir) {
    return dir.begins_with("res://") || dir.begins_with("user://")
               ? godot::ProjectSettings::get_singleton()->globalize_path(dir)
               : dir;
}

godot::String to_godot(const std::string& s) {
    return godot::String::utf8(s.data(), static_cast<int64_t>(s.size()));
}

}  // namespace

SailBoat::SailBoat() : wind_(std::make_shared<const sail::UniformWind>(0.0, 0.0)) {}

SailBoat::~SailBoat() = default;

template <typename F>
bool SailBoat::guarded(const char* what, F&& body) {
    try {
        std::forward<F>(body)();
        return true;
    } catch (const std::exception& e) {
        fail(what, e.what());
    } catch (...) {
        fail(what, "unknown exception");
    }
    return false;
}

void SailBoat::fail(const char* what, const char* detail) {
    boat_.reset();
    last_error_ = godot::String("SailBoat ") + what + ": " + to_godot(detail);
    godot::UtilityFunctions::push_error(last_error_);
}

void SailBoat::set_boat_dir(const godot::String& dir) {
    boat_dir_ = dir;
    load();
}

void SailBoat::set_scene_dir(const godot::String& dir) {
    scene_dir_ = dir;
    bathymetry_.reset();
    if (!scene_dir_.is_empty()) {
        const bool ok = guarded("load scene", [&] {
            const std::filesystem::path meta = to_path(globalize(scene_dir_)) / "meta.json";
            bathymetry_ = std::make_shared<const sail::Bathymetry>(sail::Bathymetry::load(meta));
        });
        if (!ok) return;
    }
    load();
}

void SailBoat::load() {
    boat_.reset();
    last_error_ = godot::String();
    main_index_.reset();
    jib_index_.reset();
    if (boat_dir_.is_empty()) return;

    const godot::String global = globalize(boat_dir_);
    guarded("load", [&] {
        const std::filesystem::path dir = to_path(global);
        if (!std::filesystem::is_directory(dir)) {
            throw std::runtime_error("boat dir '" + dir.string() + "' is not a directory");
        }
        // BoatSim finds the sail tables at boat_dir/../../sails the same way.
        const std::filesystem::path section_file = dir / ".." / ".." / "foils" / "naca0012.csv";
        auto section =
            std::make_shared<const sail::FoilSection>(sail::FoilSection::load(section_file));
        sail::BoatSimOptions options;
        options.bathymetry = bathymetry_;
        boat_ = sail::BoatSim::load(dir, std::move(section), options);
        const auto& specs = boat_->sails().params().sails;
        for (std::size_t i = 0; i < specs.size(); ++i) {
            if (specs[i].name == "main") main_index_ = i;
            if (specs[i].name == "jib") jib_index_ = i;
        }
    });
    if (!boat_) return;

    const sail::Vec3 forward = frames::forward_to_world(get_transform().basis);
    const double heading =
        std::hypot(forward.x(), forward.y()) > 1e-9 ? to_deg(std::atan2(forward.x(), forward.y()))
                                                    : 0.0;
    reset_boat(get_position(), heading, 0.0);
}

void SailBoat::apply_wind() {
    guarded("set wind", [&] {
        wind_ = std::make_shared<const sail::LogProfileWind>(wind_speed_mps_, wind_from_deg_);
        if (boat_) boat_->set_wind(wind_);
    });
}

void SailBoat::set_wind_speed_kn(double knots) {
    if (!std::isfinite(knots) || knots < 0.0) {
        godot::UtilityFunctions::push_error("SailBoat: wind speed must be finite and >= 0 kn");
        return;
    }
    wind_speed_mps_ = knots * kKnot;
    apply_wind();
}

double SailBoat::get_wind_speed_kn() const { return wind_speed_mps_ / kKnot; }

void SailBoat::set_wind_direction_deg(double from_deg) {
    if (!std::isfinite(from_deg)) {
        godot::UtilityFunctions::push_error("SailBoat: wind direction must be finite");
        return;
    }
    wind_from_deg_ = wrap_360(from_deg);
    apply_wind();
}

void SailBoat::apply_controls(const sail::Controls& controls) {
    controls_ = sail::clamped(controls);
    if (boat_) guarded("set controls", [&] { boat_->set_controls(controls_); });
}

#define SAILGD_CONTROL_SETTER(name)          \
    void SailBoat::set_##name(double v) {    \
        sail::Controls c = controls_;        \
        c.name = v;                          \
        apply_controls(c);                   \
    }

SAILGD_CONTROL_SETTER(tiller)
SAILGD_CONTROL_SETTER(mainsheet)
SAILGD_CONTROL_SETTER(traveler)
SAILGD_CONTROL_SETTER(backstay)
SAILGD_CONTROL_SETTER(jib_sheet_port)
SAILGD_CONTROL_SETTER(jib_sheet_starboard)
SAILGD_CONTROL_SETTER(jib_car)
SAILGD_CONTROL_SETTER(reef)

#undef SAILGD_CONTROL_SETTER

bool SailBoat::set_controls_from_dictionary(const godot::Dictionary& controls) {
    sail::Controls c = controls_;
    bool ok = true;
    const godot::Array keys = controls.keys();
    for (int64_t i = 0; i < keys.size(); ++i) {
        const godot::String key = keys[i];
        const godot::Variant value = controls[keys[i]];
        if (value.get_type() != godot::Variant::FLOAT && value.get_type() != godot::Variant::INT) {
            godot::UtilityFunctions::push_error("SailBoat: control '", key, "' is not a number");
            ok = false;
            continue;
        }
        const double v = value;
        if (key == "tiller") c.tiller = v;
        else if (key == "mainsheet") c.mainsheet = v;
        else if (key == "traveler") c.traveler = v;
        else if (key == "backstay") c.backstay = v;
        else if (key == "jib_sheet") c.jib_sheet_port = c.jib_sheet_starboard = v;
        else if (key == "jib_sheet_port") c.jib_sheet_port = v;
        else if (key == "jib_sheet_starboard") c.jib_sheet_starboard = v;
        else if (key == "jib_car") c.jib_car = v;
        else if (key == "reef") c.reef = v;
        else {
            godot::UtilityFunctions::push_error("SailBoat: unknown control '", key, "'");
            ok = false;
        }
    }
    apply_controls(c);
    return ok;
}

godot::Dictionary SailBoat::get_controls() const {
    godot::Dictionary d;
    d["tiller"] = controls_.tiller;
    d["mainsheet"] = controls_.mainsheet;
    d["traveler"] = controls_.traveler;
    d["backstay"] = controls_.backstay;
    d["jib_sheet_port"] = controls_.jib_sheet_port;
    d["jib_sheet_starboard"] = controls_.jib_sheet_starboard;
    d["jib_car"] = controls_.jib_car;
    d["reef"] = controls_.reef;
    return d;
}

bool SailBoat::reset_boat(const godot::Vector3& position, double heading_deg, double speed_mps) {
    if (!boat_) {
        godot::UtilityFunctions::push_error("SailBoat reset_boat: no boat loaded");
        return false;
    }
    if (!std::isfinite(heading_deg) || !std::isfinite(speed_mps)) {
        godot::UtilityFunctions::push_error("SailBoat reset_boat: heading and speed must be finite");
        return false;
    }
    const bool ok = guarded("reset_boat", [&] {
        boat_->reset();
        boat_->set_wind(wind_);
        boat_->set_controls(controls_);
        sail::State initial = boat_->equilibrium().pose;
        const sail::Vec3 at = frames::godot_to_world(position);
        const double yaw = (90.0 - heading_deg) * kDeg;
        initial.orientation =
            (sail::Quat(Eigen::AngleAxisd(yaw, sail::Vec3::UnitZ())) * initial.orientation)
                .normalized();
        initial.position.x() = at.x();
        initial.position.y() = at.y();
        initial.linear_velocity = sail::Vec3(speed_mps, 0.0, 0.0);
        initial.angular_velocity = sail::Vec3::Zero();
        boat_->start(initial);
    });
    if (ok) update_transform();
    return ok;
}

void SailBoat::_process(double delta) {
    if (!boat_ || godot::Engine::get_singleton()->is_editor_hint()) return;
    const bool ok = guarded("step", [&] {
        boat_->advance(delta);
        if (!finite(boat_->simulation().state())) {
            throw std::runtime_error("the simulation state is not finite");
        }
    });
    if (ok) update_transform();
}

int SailBoat::step_seconds(double seconds) {
    if (!boat_ || !std::isfinite(seconds) || seconds <= 0.0) return 0;
    // Simulation::advance drops what exceeds kMaxSubsteps per call, so feed it no more than that.
    constexpr double kChunk = sail::Simulation::kMaxSubsteps * sail::Simulation::kFixedDt;
    int steps = 0;
    guarded("step_seconds", [&] {
        double left = seconds;
        while (left > 0.0) {
            const double dt = left < kChunk ? left : kChunk;
            steps += boat_->advance(dt);
            left -= dt;
            if (!finite(boat_->simulation().state())) {
                throw std::runtime_error("the simulation state is not finite");
            }
        }
    });
    if (boat_) update_transform();
    return steps;
}

void SailBoat::update_transform() {
    if (!boat_) return;
    guarded("update transform", [&] {
        const sail::Simulation& sim = boat_->simulation();
        const double a = sim.alpha();
        const sail::State& p = sim.previous_state();
        const sail::State& c = sim.state();
        const sail::Vec3 position = p.position + a * (c.position - p.position);
        const sail::Quat orientation = p.orientation.slerp(a, c.orientation);
        set_transform(frames::state_to_godot(position, orientation));
    });
}

godot::Dictionary SailBoat::get_state() const {
    godot::Dictionary d;
    d["active"] = boat_ != nullptr;
    if (!boat_) return d;
    try {
        sail::BoatSim& boat = *boat_;
        const sail::Simulation& sim = boat.simulation();
        const sail::State& s = sim.state();
        const double t = sim.time();
        const sail::Mat3 r = s.rotation();
        const sail::Vec3 v = s.world_linear_velocity();
        const double yaw = std::atan2(r(1, 0), r(0, 0));
        const double along = v.x() * std::cos(yaw) + v.y() * std::sin(yaw);
        const double across = -v.x() * std::sin(yaw) + v.y() * std::cos(yaw);
        const double speed = std::hypot(v.x(), v.y());
        const sail::ApparentWind aw = sail::apparent_wind(s, boat.rig_point(), *wind_, t);

        d["time_s"] = t;
        d["bsp_mps"] = speed;
        d["bsp_kn"] = speed / kKnot;
        d["heading_deg"] = wrap_360(90.0 - to_deg(yaw));
        d["heel_deg"] = to_deg(sail::heel_angle(r));
        d["leeway_deg"] = speed > 1e-6 ? to_deg(std::atan2(across, along)) : 0.0;
        d["yaw_rate_dps"] = to_deg((r * s.angular_velocity).z());
        d["awa_deg"] = to_deg(aw.awa);
        d["aws_mps"] = aw.aws;
        d["aws_kn"] = aw.aws / kKnot;
        d["twa_deg"] = to_deg(sail::harness::true_wind_angle(s, *wind_, t));
        d["course_twa_deg"] = to_deg(sail::harness::course_true_wind_angle(s, *wind_, t));
        d["tws_kn"] = wind_speed_mps_ / kKnot;
        d["twd_deg"] = wind_from_deg_;
        d["rudder_angle_deg"] = to_deg(boat.appendages().rudder_angle());
        d["rudder_hinge_moment_nm"] = boat.appendages().rudder_hinge_moment_Nm();

        const sail::SheetedSailState& main = boat.sheets().main();
        const sail::SheetedSailState& jib = boat.sheets().jib();
        d["boom_angle_deg"] = to_deg(main.angle);
        d["main_angle_deg"] = to_deg(main.angle);
        d["main_aoa_deg"] = to_deg(main.aoa);
        d["main_luffing"] = main.luffing;
        d["jib_angle_deg"] = to_deg(jib.angle);
        d["jib_aoa_deg"] = to_deg(jib.aoa);
        d["jib_luffing"] = jib.luffing;
        const auto& sails = boat.sails().last_result().sails;
        if (main_index_ && *main_index_ < sails.size()) {
            d["main_stalled"] = sails[*main_index_].stalled;
            d["main_backed"] = sails[*main_index_].backed;
        }
        if (jib_index_ && *jib_index_ < sails.size()) {
            d["jib_stalled"] = sails[*jib_index_].stalled;
            d["jib_backed"] = sails[*jib_index_].backed;
        }

        const sail::CrewPosition& crew = boat.crew().position();
        d["crew_to_port_m"] = crew.y;
        d["crew_height_m"] = crew.z;
        d["crew_hiking"] = crew.posture == sail::CrewPosture::hiking;

        const sail::GroundingResult& ground = boat.grounding().last_result();
        d["has_seabed"] = boat.bathymetry() != nullptr;
        d["seabed_contacts"] = static_cast<int64_t>(ground.contacts);
        d["seabed_normal_force_n"] = ground.normal_force;
        if (boat.bathymetry() && boat.bathymetry()->has_seabed()) {
            d["seabed_z_m"] = boat.bathymetry()->height(s.position.x(), s.position.y());
        }
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error("SailBoat get_state: ", to_godot(e.what()));
        d["active"] = false;
    } catch (...) {
        godot::UtilityFunctions::push_error("SailBoat get_state: unknown exception");
        d["active"] = false;
    }
    return d;
}

void SailBoat::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    using godot::PropertyInfo;
    using godot::Variant;

    ClassDB::bind_method(D_METHOD("set_boat_dir", "dir"), &SailBoat::set_boat_dir);
    ClassDB::bind_method(D_METHOD("get_boat_dir"), &SailBoat::get_boat_dir);
    ClassDB::bind_method(D_METHOD("set_scene_dir", "dir"), &SailBoat::set_scene_dir);
    ClassDB::bind_method(D_METHOD("get_scene_dir"), &SailBoat::get_scene_dir);
    ClassDB::bind_method(D_METHOD("set_wind_speed_kn", "knots"), &SailBoat::set_wind_speed_kn);
    ClassDB::bind_method(D_METHOD("get_wind_speed_kn"), &SailBoat::get_wind_speed_kn);
    ClassDB::bind_method(D_METHOD("set_wind_direction_deg", "from_deg"),
                         &SailBoat::set_wind_direction_deg);
    ClassDB::bind_method(D_METHOD("get_wind_direction_deg"), &SailBoat::get_wind_direction_deg);

    ClassDB::bind_method(D_METHOD("set_tiller", "value"), &SailBoat::set_tiller);
    ClassDB::bind_method(D_METHOD("get_tiller"), &SailBoat::get_tiller);
    ClassDB::bind_method(D_METHOD("set_mainsheet", "value"), &SailBoat::set_mainsheet);
    ClassDB::bind_method(D_METHOD("get_mainsheet"), &SailBoat::get_mainsheet);
    ClassDB::bind_method(D_METHOD("set_traveler", "value"), &SailBoat::set_traveler);
    ClassDB::bind_method(D_METHOD("get_traveler"), &SailBoat::get_traveler);
    ClassDB::bind_method(D_METHOD("set_backstay", "value"), &SailBoat::set_backstay);
    ClassDB::bind_method(D_METHOD("get_backstay"), &SailBoat::get_backstay);
    ClassDB::bind_method(D_METHOD("set_jib_sheet_port", "value"), &SailBoat::set_jib_sheet_port);
    ClassDB::bind_method(D_METHOD("get_jib_sheet_port"), &SailBoat::get_jib_sheet_port);
    ClassDB::bind_method(D_METHOD("set_jib_sheet_starboard", "value"),
                         &SailBoat::set_jib_sheet_starboard);
    ClassDB::bind_method(D_METHOD("get_jib_sheet_starboard"), &SailBoat::get_jib_sheet_starboard);
    ClassDB::bind_method(D_METHOD("set_jib_car", "value"), &SailBoat::set_jib_car);
    ClassDB::bind_method(D_METHOD("get_jib_car"), &SailBoat::get_jib_car);
    ClassDB::bind_method(D_METHOD("set_reef", "value"), &SailBoat::set_reef);
    ClassDB::bind_method(D_METHOD("get_reef"), &SailBoat::get_reef);

    ClassDB::bind_method(D_METHOD("set_controls_from_dictionary", "controls"),
                         &SailBoat::set_controls_from_dictionary);
    ClassDB::bind_method(D_METHOD("get_controls"), &SailBoat::get_controls);
    ClassDB::bind_method(D_METHOD("reset_boat", "position", "heading_deg", "speed_mps"),
                         &SailBoat::reset_boat, DEFVAL(0.0));
    ClassDB::bind_method(D_METHOD("step_seconds", "seconds"), &SailBoat::step_seconds);
    ClassDB::bind_method(D_METHOD("is_active"), &SailBoat::is_active);
    ClassDB::bind_method(D_METHOD("get_last_error"), &SailBoat::get_last_error);
    ClassDB::bind_method(D_METHOD("get_state"), &SailBoat::get_state);

    ADD_PROPERTY(PropertyInfo(Variant::STRING, "boat_dir", godot::PROPERTY_HINT_DIR),
                 "set_boat_dir", "get_boat_dir");
    ADD_PROPERTY(PropertyInfo(Variant::STRING, "scene_dir", godot::PROPERTY_HINT_DIR),
                 "set_scene_dir", "get_scene_dir");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_speed_kn"), "set_wind_speed_kn",
                 "get_wind_speed_kn");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "wind_direction_deg"), "set_wind_direction_deg",
                 "get_wind_direction_deg");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "tiller", godot::PROPERTY_HINT_RANGE, "-1,1,0.01"),
                 "set_tiller", "get_tiller");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "mainsheet", godot::PROPERTY_HINT_RANGE, "0,1,0.01"),
                 "set_mainsheet", "get_mainsheet");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "traveler", godot::PROPERTY_HINT_RANGE, "-1,1,0.01"),
                 "set_traveler", "get_traveler");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "backstay", godot::PROPERTY_HINT_RANGE, "0,1,0.01"),
                 "set_backstay", "get_backstay");
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "jib_sheet_port", godot::PROPERTY_HINT_RANGE, "0,1,0.01"),
        "set_jib_sheet_port", "get_jib_sheet_port");
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "jib_sheet_starboard", godot::PROPERTY_HINT_RANGE, "0,1,0.01"),
        "set_jib_sheet_starboard", "get_jib_sheet_starboard");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "jib_car", godot::PROPERTY_HINT_RANGE, "0,1,0.01"),
                 "set_jib_car", "get_jib_car");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "reef", godot::PROPERTY_HINT_RANGE, "0,1,1"),
                 "set_reef", "get_reef");
}

}  // namespace sailgd
