#pragma once

#include <array>
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
    // dsyhs only. A hull parameter lies outside the series' range (DsyhsRangeCheck).
    bool hull_out_of_range = false;
    // dsyhs only. |heel| was above the heeled wetted-area table, which was held at its last row.
    bool heel_clamped = false;
    // dsyhs only. The boat was heeled and Fn was above the heeled-residuary table, which was held
    // at its last row.
    bool heel_fn_clamped = false;
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

// Keuning & Sonnenberg 1998 regression coefficients, as data/hull/dsyhs1998_coefficients.csv
// holds them.
class DsyhsCoefficients {
public:
    struct Table {
        // Ascending and positive; Fn, or heel in degrees for the wetted-area table.
        std::vector<double> x;
        std::vector<std::array<double, 9>> c;
    };

    // Parses a header line, then rows `table,x,c0..c8` with tables `upright` (a0..a8 over Fn),
    // `heel20_x1000` (u0..u5 over Fn, as printed: 1000 times the coefficient) and `wetted_heel`
    // (s0..s3 over heel_deg). Unused trailing columns are empty. Throws BoatDataError on a missing
    // or malformed file, an unknown or missing table, a row with the wrong number of coefficients
    // or an axis that is not strictly ascending and positive.
    static DsyhsCoefficients load(const std::filesystem::path& file);

    const Table& upright() const { return upright_; }
    // Already divided by 1000.
    const Table& heel20() const { return heel20_; }
    const Table& wetted_heel() const { return wetted_heel_; }

private:
    Table upright_;
    Table heel20_;
    Table wetted_heel_;
};

struct DsyhsHullParameters {
    // Upright sailing-condition canoe body, from hull_hydro.json `conditions.sailing.result`.
    // m and m^3.
    double lwl = 0.0;
    double bwl = 0.0;
    double tc = 0.0;
    double canoe_volume = 0.0;
    // m^2
    double waterplane_area = 0.0;
    double wetted_area = 0.0;
    double cp = 0.0;
    double cm = 0.0;
    // m aft of the forward end of the waterline.
    double lcb_fpp = 0.0;
    double lcf_fpp = 0.0;
    // kg/m^3 and m^2/s, from hull_resistance.json `water`.
    double density = 0.0;
    double kinematic_viscosity = 0.0;
    // 1 + k on the ITTC-57 friction; the series itself uses none.
    double form_factor = 1.0;
    // hull_resistance.json `dsyhs.coefficients.path`, resolved against the boat dir.
    std::filesystem::path coefficients_file;
};

// The hull parameters against the range of the series hulls the regressions were fitted to.
struct DsyhsRangeCheck {
    double lwl_bwl = 0.0;
    double bwl_tc = 0.0;
    double lwl_vol13 = 0.0;
    // Percent of Lwl from Lwl/2, positive forward.
    double lcb_pct = 0.0;
    double lcf_pct = 0.0;
    double cp = 0.0;
    double cm = 0.0;
    double aw_vol23 = 0.0;
    bool lwl_bwl_ok = true;
    bool bwl_tc_ok = true;
    bool lwl_vol13_ok = true;
    bool lcb_ok = true;
    bool lcf_ok = true;
    bool cp_ok = true;
    bool cm_ok = true;
    bool aw_vol23_ok = true;
    bool ok() const {
        return lwl_bwl_ok && bwl_tc_ok && lwl_vol13_ok && lcb_ok && lcf_ok && cp_ok && cm_ok &&
               aw_vol23_ok;
    }
};

// Keuning & Sonnenberg 1998 bare-hull resistance from the Delft Systematic Yacht Hull Series:
// the upright residuary regression (Fn 0.10 to 0.60), the change in residuary resistance with
// heel (20 degree regression over Fn 0.25 to 0.55, scaled by 6.0 phi^1.7 with phi in radians),
// the heeled wetted area (5 to 35 degrees) and ITTC-57 friction with Re on 0.7 Lwl. Fn is on
// Lwl; every hull parameter is the upright one.
class DsyhsHullResistance final : public HullResistanceModel {
public:
    DsyhsHullResistance(DsyhsHullParameters params, DsyhsCoefficients coefficients);

    HullResistance resistance(double speed_mps, double heel_rad) const override;
    HullResistanceKind kind() const override { return HullResistanceKind::dsyhs; }

    const DsyhsHullParameters& params() const { return params_; }
    const DsyhsRangeCheck& range_check() const { return range_; }

    // Upright Rr / (canoe_volume rho g). Linear in Fn between the table rows, zero at Fn 0 and
    // extrapolated linearly above the last row.
    double upright_residuary_ratio(double fn) const;
    // Change in Rr with heel over (canoe_volume rho g), at 20 degrees. Linear in Fn, zero at Fn 0
    // and held above the last row.
    double heel20_residuary_ratio(double fn) const;
    // Canoe-body wetted area at |heel_deg|: the upright area at 0, linear between the table rows
    // and held above the last row.
    double wetted_area(double heel_deg) const;
    // ITTC-57 line with Re on 0.7 Lwl, at |speed_mps|.
    double friction_coefficient(double speed_mps) const;

private:
    DsyhsHullParameters params_;
    DsyhsCoefficients coefficients_;
    DsyhsRangeCheck range_;
    std::array<double, 9> upright_terms_{};
    std::array<double, 9> heel20_terms_{};
};

// Reads the upright hull from hull_hydro.json `conditions.sailing.result` (lwl, bwl, tc,
// canoe_volume, waterplane_area, wetted_area_canoe, cp, cm, lcb_x_canoe, lcb_frac_lwl_aft_fp,
// lcf_x) and hull_resistance.json `water` and `dsyhs` (coefficients.path, optional
// form_factor.value). Throws BoatDataError on a missing or malformed file or field or a
// non-positive quantity.
DsyhsHullParameters load_dsyhs_hull_parameters(const std::filesystem::path& boat_dir);

// Reads hull_hydro.json `conditions.sailing` (result and the optional orc block) and
// hull_resistance.json `water` and `orc` from a boat dossier directory. Throws BoatDataError on a
// missing or malformed file or field, a non-positive quantity, LSM4 or LSM6 without the other
// two lengths, or a heeled table whose columns differ in length from heel_deg or whose heel_deg
// is not strictly ascending.
OrcHullParameters load_orc_hull_parameters(const std::filesystem::path& boat_dir);

// The model hull_resistance.json `model` names ("orc" when absent), built from the boat dir.
std::unique_ptr<HullResistanceModel> load_hull_resistance(const std::filesystem::path& boat_dir);

// The given model, ignoring hull_resistance.json `model`.
std::unique_ptr<HullResistanceModel> load_hull_resistance(const std::filesystem::path& boat_dir,
                                                          HullResistanceKind kind);

}  // namespace sail
