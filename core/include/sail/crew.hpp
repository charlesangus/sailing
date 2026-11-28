#pragma once

#include <filesystem>

#include "sail/mass_model.hpp"
#include "sail/rigid_body.hpp"
#include "sail/wind.hpp"
#include "sail/windage.hpp"

namespace sail {

struct CrewParams {
    // Hull-frame z of the crew point mass on the centreline, seated upright.
    double centre_z = 0.0;
    // m, |y| of the rail (the deck edge less the inset) and the extra outboard of full hiking.
    double rail_y = 0.0;
    double hiking_outboard = 0.0;
    // m, height lost between the centreline and full hiking.
    double hiking_drop = 0.0;
    // rad
    double heel_threshold = 0.0;
    double heel_full = 0.0;
    // m/s of apparent wind speed.
    double light_air_aws = 0.0;
    double light_air_ramp = 0.0;
    // rad, of |AWA|.
    double downwind_awa = 0.0;
    double downwind_ramp = 0.0;
    // s, rail to rail.
    double transit_time = 0.0;
    double hiking_posture_fraction = 0.0;
    // m
    double update_min_move = 0.0;

    // |y| of full hiking.
    double hiking_y() const { return rail_y + hiking_outboard; }
    // m/s, the lateral speed limit.
    double max_speed() const { return 2.0 * hiking_y() / transit_time; }
};

// Reads crew.json, and the crew station and sitting height of `model`. Throws BoatDataError on
// a missing or malformed file or field, or values that leave the rail outside the deck.
CrewParams load_crew_params(const std::filesystem::path& boat_dir, const MassModel& model);

struct CrewPosition {
    double y = 0.0;
    double z = 0.0;
    CrewPosture posture = CrewPosture::sitting;
};

// Moves the crew point mass: toward the windward rail in proportion to heel above a threshold,
// full at heel_full, and to the centreline in light air and downwind. The windward side is the
// side the apparent wind comes over: the starboard rail (y < 0) for Tack::Starboard. Heel is
// taken by magnitude, as the windward side is known from the wind. The lateral position is
// rate-limited, so a change of windward side takes about transit_time, and z follows the lateral
// position between the centreline and hiking heights.
class CrewController {
public:
    // Throws std::invalid_argument on a non-positive transit time, a non-positive hiking
    // distance or a heel_full not above heel_threshold.
    explicit CrewController(const CrewParams& params);

    // Puts the crew on the centreline, for the start of a run.
    void reset();
    // Puts the crew at lateral position `y`, clamped to the hiking range.
    void reset(double y);

    // Advances the crew by `dt` seconds toward the target for `heel` (rad) and apparent wind
    // `wind` at the boat. Non-positive or non-finite dt does nothing.
    const CrewPosition& step(double dt, double heel, const ApparentWind& wind);

    // The lateral target, m, for the same inputs, ignoring the rate limit.
    double target_y(double heel, const ApparentWind& wind) const;

    const CrewPosition& position() const { return position_; }
    const CrewParams& params() const { return params_; }

private:
    void set_y(double y);

    CrewParams params_;
    CrewPosition position_;
};

}  // namespace sail
