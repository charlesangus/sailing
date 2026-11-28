#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

namespace sail {

enum class HullResistanceKind {
    orc,
    dsyhs,
};

// Canoe-body resistance at one speed and heel. Forces are magnitudes in N, opposing the motion;
// the caller applies the direction.
struct HullResistance {
    double residuary_N = 0.0;
    double viscous_N = 0.0;
    double total_N = 0.0;
    // Froude number on the length the residuary model uses.
    double fn = 0.0;
    // Length-volume and beam-draft ratios the residuary model used.
    double lvr = 0.0;
    double btr = 0.0;
    // False when any of the flags below is set.
    bool in_range = true;
    // LVR or BTR was outside the residuary data and was clamped to its edge.
    bool lvr_clamped = false;
    bool btr_clamped = false;
    // Fn was above the residuary data and the value was extrapolated.
    bool fn_extrapolated = false;
};

class HullResistanceModel {
public:
    virtual ~HullResistanceModel() = default;

    // Pure and allocation free. Uses |speed_mps| and |heel_rad|, so sternway and either tack give
    // the same magnitudes. Forces are exactly zero at zero speed; the range fields are still
    // filled in, so resistance(0, heel) is the model's range check at that heel.
    virtual HullResistance resistance(double speed_mps, double heel_rad) const = 0;

    virtual HullResistanceKind kind() const = 0;
};

// The ORC canoe-body residuary resistance surfaces: Rr per unit displacement, in N per tonne-force
// (multiply by displacement kg * g / 1000 for N), over Froude number, LVR and BTR.
class OrcDragSurfaces {
public:
    // Parses the ORCi_Drag_Surfaces.csv layout: a note line, then per Froude number a header row
    // holding Fn, a row of BTR values and one row per LVR. Throws BoatDataError on a missing or
    // malformed file or axes that differ between Froude numbers.
    static OrcDragSurfaces load(const std::filesystem::path& file);

    struct Lookup {
        double value = 0.0;
        bool lvr_clamped = false;
        bool btr_clamped = false;
        bool fn_extrapolated = false;
    };

    // Trilinear in (Fn, LVR, BTR) with a zero row at Fn 0. LVR and BTR outside the grid are
    // clamped to its edge; Fn above the grid is extrapolated linearly from the last two rows.
    // Fn below zero is treated as zero.
    Lookup lookup(double fn, double lvr, double btr) const;

    // Fn axis including the leading zero row.
    const std::vector<double>& fn_axis() const { return fn_; }
    const std::vector<double>& lvr_axis() const { return lvr_; }
    const std::vector<double>& btr_axis() const { return btr_; }

private:
    double at(std::size_t i_fn, std::size_t i_lvr, std::size_t i_btr) const {
        return values_[(i_fn * lvr_.size() + i_lvr) * btr_.size() + i_btr];
    }

    std::vector<double> fn_;
    std::vector<double> lvr_;
    std::vector<double> btr_;
    std::vector<double> values_;
};

// Quantities of the ORC 2023 model the dossier may supply in hull_hydro.json
// `conditions.sailing.orc`, each replacing a stand-in when present.
struct OrcHullOverrides {
    // m. LSM1 replaces Lwl as the viscous length and as RRLSM1; with LSM4 and LSM6 as well the
    // residuary model uses the ORC composite dynamic length.
    std::optional<double> lsm1;
    std::optional<double> lsm4;
    std::optional<double> lsm6;
    // Replaces Bwl / Tc.
    std::optional<double> btr_effective;
    // Heeled tables over heel_deg (ascending, degrees). Each column is empty when absent and
    // otherwise replaces the upright value, interpolated linearly in |heel| and held at the ends.
    std::vector<double> heel_deg;
    std::vector<double> wetted_area;
    std::vector<double> btr;
    std::vector<double> lvr;
};

struct OrcHullParameters {
    // Sailing-condition canoe body, from hull_hydro.json `conditions.sailing.result`.
    // kg, scales the residuary surfaces.
    double displacement = 0.0;
    // m^3, canoe body only.
    double canoe_volume = 0.0;
    // m
    double lwl = 0.0;
    double bwl = 0.0;
    double tc = 0.0;
    // m^2, canoe body.
    double wetted_area = 0.0;
    // kg/m^3 and m^2/s, from hull_resistance.json `water`.
    double density = 0.0;
    double kinematic_viscosity = 0.0;
    // Immersed-rail multiplier 1 + coefficient (heel_deg - threshold)^2 above threshold_deg.
    double rail_threshold_deg = 30.0;
    double rail_coefficient = 0.0;
    // hull_resistance.json `orc.drag_surfaces.path`, resolved against the boat dir.
    std::filesystem::path drag_surfaces_file;
    OrcHullOverrides overrides;
};

// ORC VPP Documentation 2023 canoe-body resistance: the drag-surface half of the residuary
// resistance (section 6.3.1) and the viscous resistance of eq. 6.2, plus an immersed-rail factor
// on Rr that is not part of the ORC documentation.
class OrcHullResistance final : public HullResistanceModel {
public:
    OrcHullResistance(OrcHullParameters params, OrcDragSurfaces surfaces);

    HullResistance resistance(double speed_mps, double heel_rad) const override;
    HullResistanceKind kind() const override { return HullResistanceKind::orc; }

    const OrcHullParameters& params() const { return params_; }
    const OrcDragSurfaces& surfaces() const { return surfaces_; }

    // ORC section 6.3.1 composite dynamic length at a speed; LSM1 (or Lwl) unless LSM1, LSM4 and
    // LSM6 are all supplied.
    double dynamic_length(double speed_mps) const;

    // Multiplier on Rr at |heel_deg|.
    double rail_factor(double heel_deg) const;

private:
    OrcHullParameters params_;
    OrcDragSurfaces surfaces_;
};

// Reads hull_hydro.json `conditions.sailing` (result and the optional orc block) and
// hull_resistance.json `water` and `orc` from a boat dossier directory. Throws BoatDataError on a
// missing or malformed file or field, a non-positive quantity, LSM4 or LSM6 without the other
// two lengths, or a heeled table whose columns differ in length from heel_deg or whose heel_deg
// is not strictly ascending.
OrcHullParameters load_orc_hull_parameters(const std::filesystem::path& boat_dir);

// The model hull_resistance.json `model` names ("orc" when absent), built from the boat dir.
std::unique_ptr<HullResistanceModel> load_hull_resistance(const std::filesystem::path& boat_dir);

// The given model, ignoring hull_resistance.json `model`. Throws BoatDataError for a model that
// is not implemented.
std::unique_ptr<HullResistanceModel> load_hull_resistance(const std::filesystem::path& boat_dir,
                                                          HullResistanceKind kind);

}  // namespace sail
