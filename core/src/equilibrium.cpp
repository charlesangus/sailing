#include "sail/equilibrium.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <nlohmann/json.hpp>

#include "sail/boat_data.hpp"

namespace sail {

namespace {

using Json = nlohmann::ordered_json;

// appendages.json describes the overlap as the root fairing reaching this far into the canoe
// body at every root vertex.
constexpr double kKeelRootDepth = 0.020;

double read_root_overlap(const std::filesystem::path& file) {
    std::ifstream in(file);
    if (!in) throw BoatDataError(file, "cannot open file");
    Json json;
    try {
        json = Json::parse(in);
    } catch (const Json::exception& e) {
        throw BoatDataError(file, std::string("malformed JSON: ") + e.what());
    }
    const Json* node = &json;
    std::string where;
    for (const char* key : {"foils", "keel", "root_overlap", "value"}) {
        where += where.empty() ? key : std::string(".") + key;
        if (!node->is_object() || !node->contains(key)) {
            throw BoatDataError(file, "missing field '" + where + "'");
        }
        node = &(*node)[key];
    }
    if (!node->is_number()) throw BoatDataError(file, where + ": expected a number");
    const double v = node->get<double>();
    if (!std::isfinite(v) || v < 0.0) {
        throw BoatDataError(file, where + ": expected a non-negative number");
    }
    return v;
}

VolumeCorrection keel_root_correction(const TriMesh& keel, double volume,
                                      const std::filesystem::path& file) {
    if (keel.vertices.empty()) throw ObjError(file, 0, "object 'keel' has no vertices");
    double top = keel.vertices.front().z();
    for (const Vec3& v : keel.vertices) top = std::max(top, v.z());
    Vec3 sum = Vec3::Zero();
    int count = 0;
    for (const Vec3& v : keel.vertices) {
        if (v.z() >= top - kKeelRootDepth) {
            sum += v;
            ++count;
        }
    }
    VolumeCorrection c;
    c.centroid = sum / count;
    c.centroid.z() = top - kKeelRootDepth / 2.0;
    c.volume = volume;
    return c;
}

State pose_at(const HydrostaticParams& params, double heel, double trim, double z) {
    State s;
    s.position = Vec3(params.reference_point.x(), params.reference_point.y(), z);
    s.orientation = heel_trim_orientation(heel, trim);
    return s;
}

}  // namespace

HydrostaticParams BoatHydrostatics::params(const MassModel& model, double gravity) const {
    HydrostaticParams p;
    p.density = model.data().density;
    p.gravity = gravity;
    p.reference_point = model.cg();
    p.corrections = corrections;
    return p;
}

BoatHydrostatics load_boat_hydrostatics(const std::filesystem::path& boat_dir) {
    const std::filesystem::path appendages = boat_dir / "appendages.obj";
    const double overlap = read_root_overlap(boat_dir / "appendages.json");

    BoatHydrostatics boat;
    boat.mesh = load_obj(boat_dir / "hull_physics.obj");
    append_mesh(boat.mesh, load_obj(appendages));
    const std::vector<std::string> keel_name = {"keel"};
    const TriMesh keel = load_obj(appendages, keel_name);
    if (overlap > 0.0) boat.corrections.push_back(keel_root_correction(keel, overlap, appendages));
    return boat;
}

Quat heel_trim_orientation(double heel, double trim) {
    return Quat(Eigen::AngleAxisd(trim, Vec3::UnitY()) * Eigen::AngleAxisd(heel, Vec3::UnitX()));
}

Equilibrium solve_equilibrium(const TriMesh& mesh, const HydrostaticParams& params, double mass,
                              const EquilibriumRequest& request) {
    const double weight = mass * params.gravity;
    const FlatWater water(0.0);
    HydrostaticScratch scratch;
    const auto evaluate = [&](double z, double trim) {
        return compute_hydrostatics(mesh, pose_at(params, request.heel, trim, z), water, 0.0,
                                    params, scratch);
    };
    // Force over weight and moment over weight (a lever in metres) are of similar size, so one
    // norm can drive the line search.
    const auto residual = [&](const HydrostaticResult& r) {
        return Eigen::Vector2d((r.force_world.z() - weight) / weight,
                               request.free_trim ? r.moment_world.y() / weight : 0.0);
    };
    constexpr double kForceTol = 1e-9;
    constexpr double kMomentTol = 1e-6;
    constexpr double kDz = 1e-5;
    constexpr double kDtrim = 1e-5;
    constexpr double kMaxDz = 0.25;
    constexpr double kMaxDtrim = 0.2;
    constexpr int kMaxIterations = 60;

    Equilibrium eq;
    eq.heel = request.heel;
    double z = request.z.value_or(params.reference_point.z());
    double trim = request.trim;
    HydrostaticResult current = evaluate(z, trim);
    Eigen::Vector2d f = residual(current);
    int i = 0;
    for (; i < kMaxIterations; ++i) {
        if (std::abs(f.x()) < kForceTol && std::abs(f.y()) < kMomentTol) {
            eq.converged = true;
            break;
        }
        Eigen::Vector2d step;
        const Eigen::Vector2d fz = residual(evaluate(z + kDz, trim));
        const double dfz = (fz.x() - f.x()) / kDz;
        if (request.free_trim) {
            Eigen::Matrix2d jacobian;
            jacobian.col(0) = (fz - f) / kDz;
            jacobian.col(1) = (residual(evaluate(z, trim + kDtrim)) - f) / kDtrim;
            step = jacobian.partialPivLu().solve(f);
        } else {
            step = Eigen::Vector2d(f.x() / dfz, 0.0);
        }
        // Out of the water the force does not change with height: drop the boat in.
        if (!step.allFinite() || dfz == 0.0) {
            step = Eigen::Vector2d(f.x() < 0.0 ? kMaxDz : -kMaxDz, 0.0);
        }
        step.x() = std::clamp(step.x(), -kMaxDz, kMaxDz);
        step.y() = std::clamp(step.y(), -kMaxDtrim, kMaxDtrim);

        double alpha = 1.0;
        HydrostaticResult trial;
        Eigen::Vector2d g;
        for (int halving = 0; halving < 10; ++halving) {
            trial = evaluate(z - alpha * step.x(), trim - alpha * step.y());
            g = residual(trial);
            if (g.norm() < f.norm()) break;
            alpha *= 0.5;
        }
        z -= alpha * step.x();
        trim -= alpha * step.y();
        current = trial;
        f = g;
    }
    if (!eq.converged) eq.converged = std::abs(f.x()) < kForceTol && std::abs(f.y()) < kMomentTol;
    eq.iterations = i;
    eq.trim = trim;
    eq.pose = pose_at(params, request.heel, trim, z);
    eq.hydrostatics = current;
    eq.righting_moment = -current.moment_world.x();
    return eq;
}

}  // namespace sail
