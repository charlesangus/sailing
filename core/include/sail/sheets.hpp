#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "sail/controls.hpp"
#include "sail/rigid_body.hpp"
#include "sail/sails.hpp"
#include "sail/simulation.hpp"
#include "sail/wind.hpp"

namespace sail {

// A sail swinging about its luff as a rigid body, and the sheet that stops it. The swing is the
// rotation about the luff, positive taking the clew to port (body +y); at zero swing the sail
// lies in the centreplane. Any frame will do as long as the lead is given in the same one.
class SheetGeometry {
public:
    // `tack` and `head` set the luff axis, `sheet_point` is where the sheet attaches (the boom
    // strop for a main, the clew for a jib). Throws std::invalid_argument on a zero luff or a
    // clew or sheet point on the luff axis.
    SheetGeometry(const Vec3& tack, const Vec3& head, const Vec3& clew, const Vec3& sheet_point);

    Vec3 clew(double swing) const;
    Vec3 sheet_point(double swing) const;
    // rad: the tack-to-clew chord's angle in the body xy plane from dead aft, positive with the
    // clew to port, the angle SailTrim::angle_of_attack is measured from.
    double chord_angle(double swing) const;
    // The swing in [0, pi] whose chord_angle() is `angle` (clamped to [0, pi]).
    double swing_for_chord_angle(double angle) const;

    // m, straight from the sheet point to `lead`.
    double sheet_length(double swing, const Vec3& lead) const;
    // The swing, in (-pi, pi], that brings the sheet point nearest `lead`: the sheet two-blocked.
    double nearest_swing(const Vec3& lead) const;
    // The largest swing in [nearest_swing, nearest_swing + pi], capped at pi, to which a sheet of
    // `length` from `lead` lets the sail go to port: where the sheet comes taut. nearest_swing()
    // for a sheet shorter than the nearest distance.
    double limit_swing(const Vec3& lead, double length) const;

private:
    Vec3 tack_;
    // Unit, oriented so a positive swing takes the clew to port.
    Vec3 axis_;
    Vec3 clew_;
    Vec3 sheet_point_;
};

// One sheeted sail, hull frame.
struct SheetedSailParams {
    // The SailSet sail it trims.
    std::string sail;
    Vec3 tack = Vec3::Zero();
    Vec3 head = Vec3::Zero();
    Vec3 clew = Vec3::Zero();
    Vec3 sheet_point = Vec3::Zero();
    // rad, chord angle off the centreline at which the sail stops: the main's boom against the
    // shrouds, the jib's sheet run out. It also fixes the sheet's longest length.
    double max_angle = 1.0;
    // ORC flat with the backstay fully on; slack it is 1.
    double flat_at_full_backstay = 1.0;
    // Area once reefed over the area at full hoist; 1 for a sail that does not reef.
    double reefed_area_fraction = 1.0;
};

struct SheetParams {
    // Mainsheet from the boom strop (sheet_point) to the traveler car.
    SheetedSailParams main;
    Vec3 traveler_centre = Vec3::Zero();
    // m, the car's travel across the boat, centred on traveler_centre.
    double traveler_span = 0.0;

    // Jib sheet from the clew (sheet_point) through the car block on the jib track. The track is
    // given on the port side and mirrored for starboard.
    SheetedSailParams jib;
    Vec3 jib_track_forward = Vec3::Zero();
    Vec3 jib_track_aft = Vec3::Zero();
    // Factor on the jib's flat with the car fully aft, which opens the leech; fully forward it is
    // 1.
    double jib_flat_with_car_aft = 1.0;

    // s, from full hoist to reefed and back.
    double reef_transition_time = 10.0;
};

// Reads sheets.json, the sail corners and sheeting hardware of sails.json (main tack, head,
// clew, sheet_point_on_boom, traveler_centre and traveler_span; jib tack, head, clew,
// track_fwd_end and track_aft_end) and the hull_hydro.json transform to the hull frame, from a
// boat dossier directory. Throws BoatDataError on a missing or malformed file or field or a
// value out of range.
SheetParams load_sheet_params(const std::filesystem::path& boat_dir);

// The reef's progress from full hoist (0) to reefed (1). It follows the command at no more than
// 1 / transition_time per second, by step(dt), and lands on it exactly once within reach.
class ReefActuator {
public:
    // Throws std::invalid_argument unless transition_time is positive and finite.
    explicit ReefActuator(double transition_time);

    void set_command(const Controls& controls) { command_ = clamped(controls).reef; }
    // Non-positive or non-finite dt does nothing.
    void step(double dt);
    // Puts the reef at `reefed` (0 or 1, rounded as Controls::reef) and commands it to stay.
    void reset(double reefed);

    // Linear in time.
    double progress() const { return progress_; }
    // The fraction of the reefed area change made: smoothstep(progress), so the area starts and
    // finishes moving gently.
    double depth() const;
    double command() const { return command_; }
    double transition_time() const { return transition_time_; }

private:
    double transition_time_;
    double progress_ = 0.0;
    double command_ = 0.0;
};

// What the latest update found for one sail.
struct SheetedSailState {
    // At the sail's centre of effort.
    ApparentWind wind;
    // The tack the sail is set for. It lies to leeward: to port on starboard tack.
    Tack tack = Tack::Starboard;
    // Hull frame, the sheet's lead: the traveler car or the jib car block on the leeward side.
    Vec3 lead = Vec3::Zero();
    // m, the sheet's length from its control.
    double sheet_length = 0.0;
    // rad, chord angles, positive with the clew to port: aligned with the apparent wind, as far
    // as the sheet and the stop let it go, and the sail's, the smaller of the two in magnitude
    // toward leeward.
    double free_angle = 0.0;
    double sheet_angle = 0.0;
    double angle = 0.0;
    // rad, awa_sail - angle, signed as SailTrim::angle_of_attack.
    double aoa = 0.0;
    // |aoa| below the sail's luff_aoa.
    bool luffing = false;
    // What was given to SailSet::set_trim.
    SailTrim trim;
};

// Turns Controls into each sail's angle, flat and reef, and gives them to a SailSet as SailTrim.
//
// A ForceModule that adds nothing, registered with the Simulation before the SailSet so it runs
// first in each step; the SailSet's own sail model is left as it is. It reads the apparent wind
// at each sail's centre of effort from the SailSet's latest result, one evaluation old, which
// differs from the current one only while flat or reef move it.
//
// Sail angle: on the leeward side the sail weathervanes to the apparent wind (free_angle,
// |awa_sail|) until the sheet comes taut (sheet_angle) or it reaches max_angle. The sheet's
// length runs linearly from its longest at sheet 0 (the sail at max_angle, the main's car
// centred, the jib's car mid-track) to its shortest at sheet 1 (two-blocked at the same lead).
// The traveler moves the main's lead across the boat and the jib car moves the jib's along its
// track, so either changes the angle at a constant sheet; the main can be sheeted to windward of
// the centreline with the car there. A tack or gybe, the wind crossing to the sail's other side,
// moves it to the new leeward side at once.
//
// Shape: flat = 1 - backstay (1 - flat_at_full_backstay) for both sails, the jib's times
// 1 - jib_car (1 - jib_flat_with_car_aft). The ORC flat also lowers the centre of effort by its
// twist function, which is how the car's twist is felt. Reef: area fraction
// 1 - depth (1 - reefed_area_fraction), SailTrim::reef its square root.
//
// Time: each update steps the reef by the time elapsed since the previous one; the first, a
// repeated time or a time that runs backwards steps nothing.
class SheetController final : public ForceModule {
public:
    // `sails` must outlive this and have the two sails `params` names. Throws
    // std::invalid_argument on a null wind, a parameter out of range or degenerate geometry,
    // and std::out_of_range on a sail the SailSet does not have.
    SheetController(SheetParams params, SailSet& sails, std::shared_ptr<const WindField> wind);

    // Calls update(); adds nothing to `wrench`.
    void accumulate(const State& state, double t, Wrench& wrench) override;
    void update(const State& state, double t);

    void set_controls(const Controls& controls);
    const Controls& controls() const { return controls_; }

    // Puts the reef at `reefed` (0 or 1) holding there, forgets the previous update's time and
    // each sail's side, which the next update takes from the wind.
    void reset(double reefed = 0.0);

    void set_wind(std::shared_ptr<const WindField> wind);

    const SheetedSailState& main() const { return main_.state; }
    const SheetedSailState& jib() const { return jib_.state; }
    const ReefActuator& reef() const { return reef_; }
    const SheetParams& params() const { return params_; }

    // m: the sheets' lengths at sheet 1 and sheet 0.
    double main_sheet_range_min() const { return main_.shortest; }
    double main_sheet_range_max() const { return main_.longest; }
    double jib_sheet_range_min() const { return jib_.shortest; }
    double jib_sheet_range_max() const { return jib_.longest; }

private:
    struct Sail {
        Sail(std::size_t i, const SheetGeometry& g) : index(i), geometry(g) {}

        std::size_t index = 0;
        SheetGeometry geometry;
        double stop_swing = 0.0;
        double shortest = 0.0;
        double longest = 0.0;
        double luff_aoa = 0.0;
        std::optional<Tack> tack;
        SheetedSailState state;
    };

    Sail make_sail(const SheetedSailParams& p, const Vec3& reference_lead) const;
    // The sheet on the sail's side applies; `port_lead` is the lead with the sail to port.
    void trim(Sail& sail, const SheetedSailParams& p, const State& state, double t,
              double port_sheet, double starboard_sheet, const Vec3& port_lead, double flat);

    SheetParams params_;
    SailSet& sails_;
    std::shared_ptr<const WindField> wind_;
    Controls controls_;
    ReefActuator reef_;
    Sail main_;
    Sail jib_;
    bool has_time_ = false;
    double last_time_ = 0.0;
};

}  // namespace sail
