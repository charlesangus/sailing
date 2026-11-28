#pragma once

#include <filesystem>

namespace sail {

// Dimensionless multipliers that tune a boat's force models to its reference polars, read from
// calibration.json in the boat dossier. Every one is 1, the uncalibrated model, when the file or
// its entry is absent.
struct Calibration {
    // On each sail's section lift coefficient, Cl_max of the ORC table at the optimum.
    double main_lift = 1.0;
    double jib_lift = 1.0;
    // On each sail's parasitic drag coefficient, Cd0 of the ORC table.
    double main_parasitic_drag = 1.0;
    double jib_parasitic_drag = 1.0;
    // On the sail plan's effective height heff, which sets the induced drag A / (pi heff^2).
    double effective_height = 1.0;
    // On the rise of kheff above its reaching value 0.8 at small AWA.
    double kheff_upwind_rise = 1.0;
    // On the headsail's blanketing loss 1 - bk behind the main, the main-jib interaction
    // downwind.
    double headsail_blanketing = 1.0;
    // On the canoe body's residuary and viscous resistance.
    double hull_resistance = 1.0;
    // On the drag of every windage element.
    double windage = 1.0;
};

// Reads `boat_dir` / calibration.json: a top-level object whose entries main_lift, jib_lift,
// main_parasitic_drag, jib_parasitic_drag, effective_height, kheff_upwind_rise,
// headsail_blanketing, hull_resistance and windage, each optional, hold a positive "value".
// Returns all ones when the file does not exist. Throws BoatDataError on a malformed file, a
// non-positive value or an unknown entry that has a "value".
Calibration load_calibration(const std::filesystem::path& boat_dir);

}  // namespace sail
