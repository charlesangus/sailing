#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sail/rigid_body.hpp"
#include "sail/simulation.hpp"
#include "sail/wind.hpp"

namespace sail {

// The part a sail plays in the ORC combined-rig method (ORC VPP Documentation 2023, 5.4 and 5.6):
// the main sets the rig height and roach, the headsail the fractionality, overlap, parasitic drag
// de-powering (fcdj) and blanketing.
enum class SailRole {
    main,
    headsail,
};

// Not-a-knot cubic spline through (x, y), the interpolant scipy's make_interp_spline(k=3) builds,
// so the tables are read as Python-VPP reads them. Outside [x.front(), x.back()] the end values
// hold.
class CubicSpline {
public:
    CubicSpline() = default;
    // x strictly increasing, 4 or more points. Throws std::invalid_argument otherwise.
    CubicSpline(std::vector<double> x, std::vector<double> y);

    double operator()(double x) const;

private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> m_;
};

struct SailCoefficientParams {
    SailRole role = SailRole::main;
    // Two-dimensional quadratic viscous drag coefficient (ORC kpm, kpj).
    double kp = 0.0;
    // rad. The angle-of-attack form, see SailCoefficients::section().
    double aoa_design = 0.0;
    double max_ease = 0.0;
    double luff_aoa = 0.0;
    double stall_width = 0.0;
    // Normal-force coefficient of the fully stalled sail, a flat plate's Cn.
    double normal_force = 0.0;
};

// Signed section coefficients of one sail in wind axes, on its own area.
struct SailSection {
    // Positive lift points 90 degrees clockwise (seen from above) from where the air goes, which
    // is to leeward on starboard tack.
    double cl = 0.0;
    double cd = 0.0;
    // rad, signed as SailTrim::angle_of_attack.
    double aoa = 0.0;
    bool luffing = false;
    bool stalled = false;
    // The wind is on the face that is to leeward on this tack.
    bool backed = false;
};

// ORC maximum lift and parasitic drag coefficients of one sail type against apparent wind angle,
// for an optimally trimmed sail, plus a lift curve in angle of attack built on them.
//
// The tables cover 0 to 180 degrees. Over (-180, 180] they are mirrored: Cl is odd and Cd even in
// the apparent wind angle, so a sail on port tack is the starboard sail reflected in the
// centreplane. In angle-of-attack form Cl is odd and Cd even in the angle of attack, so a backed
// sail, the wind on its other face, gives reversed lift; head to wind that is reverse drive.
class SailCoefficients {
public:
    // awa_deg strictly increasing from at least 0 to exactly 180, 4 or more rows, and the
    // parameters non-negative with aoa_design and stall_width positive. Throws
    // std::invalid_argument otherwise.
    SailCoefficients(SailCoefficientParams params, std::vector<double> awa_deg,
                     std::vector<double> cl_high, std::vector<double> cd_high,
                     std::vector<double> cl_low, std::vector<double> cd_low);

    // Reads the CSV layout of data/sails/README.md. Throws BoatDataError on a missing or malformed
    // file or a table the constructor rejects.
    static SailCoefficients load(const std::filesystem::path& file);

    const SailCoefficientParams& params() const { return params_; }

    struct Envelope {
        double cl = 0.0;
        double cd = 0.0;
    };
    // ORC Cl_max and Cd0 at |awa| (rad). `high` in [0, 1] blends the low (0) and high (1) sets.
    Envelope envelope(double awa, double high) const;

    // rad: the angle of attack the ORC table assumes at |awa|, max(min(aoa_design, |awa|),
    // |awa| - max_ease): the design angle, less when the sheet is already on the centreline and
    // more once the sail is eased as far as it goes.
    double optimum_aoa(double awa) const;

    // The table convention: the sail at optimum_aoa() on the leeward side. Reproduces envelope()
    // exactly, Cl signed by the tack.
    SailSection optimal(double awa, double high) const;

    // A sail at angle of attack `aoa` (rad, any value, wrapped) in apparent wind `awa`. With a the
    // magnitude of aoa and a_opt = optimum_aoa(awa): below a_opt the lift falls linearly to zero
    // at a = 0 and the drag stays the envelope's; above it the coefficients blend, with weight
    // w = 1 / (1 + ((a - a_opt) / stall_width)^2) on the envelope, into a flat plate's
    // Cl = Cn sin a cos a, Cd = Cn sin^2 a. At a = a_opt this is optimal().
    SailSection section(double awa, double aoa, double high) const;

private:
    SailCoefficientParams params_;
    CubicSpline cl_high_;
    CubicSpline cd_high_;
    CubicSpline cl_low_;
    CubicSpline cd_low_;
};

// One sail. Positions are hull frame.
struct SailSpec {
    std::string name;
    std::string type;
    std::shared_ptr<const SailCoefficients> coefficients;
    // m^2, rated area at reef = 1.
    double area = 0.0;
    Vec3 tack = Vec3::Zero();
    Vec3 head = Vec3::Zero();
    Vec3 clew = Vec3::Zero();
    // m: tack-head, tack-clew and head-clew distances.
    double luff = 0.0;
    double foot = 0.0;
    double leech = 0.0;
    // area / (area of the tack-head-clew triangle) - 1, Python-VPP's mainsail roach.
    double roach = 0.0;
    // m, distance of the clew from the luff line (LP).
    double luff_perpendicular = 0.0;
    // At reef = 1 and flat = 1: the triangle's centroid raised along the luff by roach * luff / 3,
    // which is Python-VPP's mainsail VCE, P / 3 (1 + roach) above the boom.
    Vec3 centre_of_effort = Vec3::Zero();
    // Weight of ORC's high coefficient set against the low one, 1 = high as Python-VPP uses.
    double high_coefficients = 1.0;
    // Calibration factors on the section Cl and on Cd0 (Calibration::main_lift and so on).
    double lift_scale = 1.0;
    double parasitic_drag_scale = 1.0;
};

// Rig dimensions the combined-rig method needs beyond the sails. Heights are hull-frame z.
struct SailRig {
    // The water surface, from which the effective rig height is measured.
    double waterplane_z = 0.0;
    // The base of I, the sheer at the mast: ORC's HBI datum, about which the twist function
    // lowers the centre of effort.
    double base_z = 0.0;
    // IG and J, m.
    double foretriangle_height = 0.0;
    double foretriangle_base = 0.0;
};

struct SailSetParams {
    // In the order sails.json lists them.
    std::vector<SailSpec> sails;
    SailRig rig;
    // kg/m^3
    double air_density = 1.225;
    // Hull-frame location of the body's reference point (the CG).
    Vec3 reference_point = Vec3::Zero();
    // Calibration factors on heff, on the rise of kheff above 0.8 and on the headsail's
    // blanketing loss 1 - bk.
    double effective_height_scale = 1.0;
    double kheff_rise_scale = 1.0;
    double blanketing_scale = 1.0;
};

// Reads sails.json, rig.json, hull_hydro.json and the optional calibration.json (see
// load_calibration) from a boat dossier directory and each sail's
// coefficients from `sail_data_dir` / orc_<type>.csv. Every top-level object of sails.json with
// tack, head, clew and area entries is a sail, of the type its optional "type" entry names, else
// its key. Positions move from the provisional stem frame to the hull frame with hull_hydro.json's
// transform; the waterplane is conditions.sailing.result.sinkage_m. Throws BoatDataError on a
// missing or malformed file or field, a non-positive area or a degenerate sail triangle.
SailSetParams load_sail_set_params(const std::filesystem::path& boat_dir,
                                   const std::filesystem::path& sail_data_dir,
                                   const Vec3& reference_point);

// Per-sail trim. ORC's de-powering parameters, 0..1.
struct SailTrim {
    // Scales Cl, and the parasitic drag of a headsail, with ORC's fcdmult on the lift-dependent
    // drag; lowers the centre of effort by the twist function.
    double flat = 1.0;
    // Shrinks the sail about its tack: area by reef^2, the centre of effort's height above the tack
    // by reef.
    double reef = 1.0;
    // rad. Without it the sail is optimally trimmed at its apparent wind angle, the ORC table
    // convention. With it: awa_sail minus the angle of the tack-to-clew chord measured from dead
    // aft, positive with the clew to port. A sail drawing normally has the sign of awa_sail.
    std::optional<double> angle_of_attack;
};

// What the latest evaluation found for one sail.
struct SailState {
    // At the centre of effort.
    ApparentWind wind;
    // m^2, after reefing.
    double area = 0.0;
    // Hull frame, after reefing and twist.
    Vec3 centre_of_effort = Vec3::Zero();
    // ORC blanketing factor bk.
    double blanketing = 1.0;
    // Coefficients on `area`, blanketing included: lift signed as SailSection::cl, drag total and
    // its parasitic part.
    double cl = 0.0;
    double cd = 0.0;
    double cd_parasitic = 0.0;
    double aoa = 0.0;
    bool luffing = false;
    bool stalled = false;
    bool backed = false;
    // Body frame, N, acting at centre_of_effort.
    Vec3 force = Vec3::Zero();
};

// The sail plan's ORC coefficients on the total area `area`.
struct SailPlanCoefficients {
    // Signed as SailSection::cl.
    double cl = 0.0;
    double cd = 0.0;
    // CD0max, unflattened.
    double cd_parasitic = 0.0;
    double kpp = 0.0;
    // m: heff, and its factors eff_span_corr and kheff.
    double effective_height = 0.0;
    double effective_span = 0.0;
    double kheff = 0.0;
    double fractionality = 1.0;
    double overlap = 0.0;
    // m^2
    double area = 0.0;
};

struct SailSetResult {
    std::vector<SailState> sails;
    SailPlanCoefficients plan;
    // Body frame, about the CG.
    Wrench wrench;
};

// Sail forces from the apparent wind at each sail's centre of effort, by ORC's combined-rig
// method. Allocates nothing per step.
//
// Each sail's Cl and Cd0 come from its table at its own awa_sail, or from its angle of attack.
// The plan coefficients are ORC's area- and blanketing-weighted sums (eqs. 5.35, 5.36), with
//   CD = CD0 [flat fcdmult fcdj + 1 - fcdj] + (KPP + A / (pi heff^2)) CL^2 fcdmult,
//   heff = eff_span_corr * kheff(AWA) * (b + HBI) (eqs. 5.42 to 5.47).
// The plan's drag is split between the sails exactly: sail i carries its own parasitic and
// quadratic drag and the induced share Cl_i CL A / (pi heff^2), so with one apparent wind the
// sails' forces sum to the plan's. Lift and drag lie in the plane normal to the mast, across and
// along the sail's apparent wind, and act at its centre of effort.
class SailSet final : public ForceModule {
public:
    // Throws std::invalid_argument on a null `wind`, an empty plan or a sail without
    // coefficients.
    SailSet(SailSetParams params, std::shared_ptr<const WindField> wind);

    void accumulate(const State& state, double t, Wrench& wrench) override;

    std::size_t size() const { return params_.sails.size(); }
    // Index of the sail named `name`. Throws std::out_of_range if there is none.
    std::size_t index(const std::string& name) const;

    const SailTrim& trim(std::size_t i) const { return trims_.at(i); }
    // Throws std::out_of_range on a bad index and std::invalid_argument on flat or reef outside
    // [0, 1] or a non-finite angle of attack.
    void set_trim(std::size_t i, const SailTrim& trim);

    // The plan coefficients with every sail in apparent wind angle `awa` (rad) at the current
    // trims: the ORC check.
    SailPlanCoefficients coefficients(double awa) const;

    const SailSetResult& last_result() const { return last_; }
    const SailSetParams& params() const { return params_; }

    void set_reference_point(const Vec3& point) { params_.reference_point = point; }
    void set_wind(std::shared_ptr<const WindField> wind);

    // Back to the state the constructor leaves: every trim at its default and last_result() the
    // sails placed at those trims with nothing evaluated.
    void reset();

private:
    struct RigState {
        double fractionality = 1.0;
        double overlap = 0.0;
        double roach = 0.0;
        double top_above_water = 0.0;
        double foretriangle_area = 0.0;
    };

    RigState rig_state() const;
    void place(const RigState& rig, std::vector<SailState>& states) const;
    SailPlanCoefficients combine(const RigState& rig, std::vector<SailState>& states) const;

    SailSetParams params_;
    std::shared_ptr<const WindField> wind_;
    std::vector<SailTrim> trims_;
    std::optional<std::size_t> main_;
    std::optional<std::size_t> headsail_;
    SailSetResult last_;
};

}  // namespace sail
