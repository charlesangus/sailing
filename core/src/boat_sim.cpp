#include "sail/boat_sim.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <utility>

namespace sail {

namespace {

std::shared_ptr<const HullResistanceModel> resistance_model(
    const std::filesystem::path& boat_dir, const std::optional<HullResistanceKind>& kind) {
    if (kind) return load_hull_resistance(boat_dir, *kind);
    return load_hull_resistance(boat_dir);
}

TowComponent resolve(const Vec3& force_world, const Vec3& d, const Vec3& n) {
    TowComponent c;
    c.resistance = -force_world.dot(d);
    c.side_force = force_world.dot(n);
    c.vertical = force_world.z();
    return c;
}

bool finite(const State& s) {
    return s.position.allFinite() && s.orientation.coeffs().allFinite() &&
           s.linear_velocity.allFinite() && s.angular_velocity.allFinite();
}

// Algebraic (Kasa) fit about the centroid, then the RMS of the geometric residuals.
std::optional<TurnCircle> fit_circle(const std::vector<TurnSample>& samples, std::size_t begin,
                                     std::size_t end) {
    const std::size_t count = end - begin;
    if (count < 3) return std::nullopt;
    double mx = 0.0;
    double my = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        mx += samples[i].x;
        my += samples[i].y;
    }
    mx /= static_cast<double>(count);
    my /= static_cast<double>(count);
    Eigen::MatrixXd a(count, 3);
    Eigen::VectorXd b(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double u = samples[begin + i].x - mx;
        const double v = samples[begin + i].y - my;
        a.row(static_cast<Eigen::Index>(i)) << u, v, 1.0;
        b(static_cast<Eigen::Index>(i)) = -(u * u + v * v);
    }
    const Eigen::Vector3d p = a.colPivHouseholderQr().solve(b);
    const double r2 = 0.25 * (p(0) * p(0) + p(1) * p(1)) - p(2);
    if (!(r2 > 0.0) || !p.allFinite()) return std::nullopt;
    TurnCircle c;
    c.centre_x = mx - 0.5 * p(0);
    c.centre_y = my - 0.5 * p(1);
    c.radius = std::sqrt(r2);
    double sum = 0.0;
    for (std::size_t i = begin; i < end; ++i) {
        const double e = std::hypot(samples[i].x - c.centre_x, samples[i].y - c.centre_y) - c.radius;
        sum += e * e;
    }
    c.rms = std::sqrt(sum / static_cast<double>(count));
    return c;
}

}  // namespace

BoatSim::BoatSim(const std::filesystem::path& boat_dir, std::shared_ptr<const FoilSection> section,
                 const BoatSimOptions& options)
    : model_(MassModel::load(boat_dir, MassConfiguration::sailing)),
      body_(model_.body_properties()),
      hydro_(load_boat_hydrostatics(boat_dir)),
      params_(hydro_.params(model_)),
      gravity_(body_.mass),
      hydrostatics_(hydro_.mesh, params_),
      damping_(load_damping(model_)),
      hull_(resistance_model(boat_dir, options.resistance),
            load_hull_force_params(model_)),
      appendages_(section, section, boat_dir, model_.cg()),
      crew_(load_crew_params(boat_dir, model_)),
      pushed_crew_(model_.crew_position()) {}

std::unique_ptr<BoatSim> BoatSim::load(const std::filesystem::path& boat_dir,
                                       std::shared_ptr<const FoilSection> section,
                                       const BoatSimOptions& options) {
    return std::make_unique<BoatSim>(boat_dir, std::move(section), options);
}

Equilibrium BoatSim::equilibrium(const EquilibriumRequest& request) const {
    return solve_equilibrium(hydro_.mesh, params_, body_.mass, request);
}

Wrench BoatSim::evaluate(const State& state, double t) {
    Wrench w;
    gravity_.accumulate(state, t, w);
    hydrostatics_.accumulate(state, t, w);
    damping_.accumulate(state, t, w);
    hull_.accumulate(state, t, w);
    appendages_.keel().accumulate(state, t, w);
    appendages_.rudder().accumulate(state, t, w);
    thrust_.accumulate(state, t, w);
    return w;
}

Vec3 BoatSim::apply_crew(double y, double z, CrewPosture posture) {
    const Vec3 old_cg = model_.cg();
    model_.set_crew_position(y, z);
    body_ = model_.body_properties();
    const Vec3 shift = model_.cg() - old_cg;
    const Vec3& cg = model_.cg();
    params_.reference_point = cg;
    gravity_.set_mass(body_.mass);
    hydrostatics_.set_reference_point(cg);
    hull_.set_reference_point(cg);
    appendages_.set_reference_point(cg);
    pushed_crew_ = model_.crew_position();
    pushed_posture_ = posture;
    for (const CrewListener& listener : crew_listeners_) listener(cg, pushed_crew_, posture);
    return shift;
}

void BoatSim::update_crew(double y, double z, CrewPosture posture) {
    const Vec3 shift = apply_crew(y, z, posture);
    if (sim_) sim_->set_body(body_, shift);
}

const CrewPosition& BoatSim::step_crew(double dt, const WindField& wind) {
    Simulation& sim = simulation();
    const State& state = sim.state();
    const ApparentWind apparent = apparent_wind(state, Vec3::Zero(), wind, sim.time());
    const CrewPosition& pos = crew_.step(dt, heel_angle(state.rotation()), apparent);
    const double moved = std::hypot(pos.y - pushed_crew_.y(), pos.z - pushed_crew_.z());
    if (moved >= crew_.params().update_min_move || pos.posture != pushed_posture_) {
        update_crew(pos.y, pos.z, pos.posture);
    }
    return pos;
}

Simulation& BoatSim::start(const State& initial) {
    sim_.reset();
    crew_.reset();
    apply_crew(crew_.position().y, crew_.position().z, crew_.position().posture);
    sim_.emplace(body_, initial);
    sim_->add_module(gravity_);
    sim_->add_module(hydrostatics_);
    sim_->add_module(damping_);
    sim_->add_module(hull_);
    appendages_.add_to(*sim_);
    sim_->add_module(thrust_);
    return *sim_;
}

Simulation& BoatSim::simulation() {
    if (!sim_) throw std::logic_error("BoatSim::simulation: start() has not been called");
    return *sim_;
}

TowResult solve_tow(BoatSim& boat, const TowRequest& request) {
    TowResult out;
    boat.thrust().set_thrust(0.0);
    boat.appendages().rudder().reset();
    EquilibriumRequest still_request;
    still_request.heel = request.heel;
    still_request.free_trim = true;
    out.still = boat.equilibrium(still_request);

    const double weight = boat.body().mass * boat.gravity().gravity();
    const double sign = request.speed < 0.0 ? -1.0 : 1.0;
    const Vec3 d = sign * Vec3(std::cos(request.leeway), std::sin(request.leeway), 0.0);
    const Vec3 n(-d.y(), d.x(), 0.0);
    const Vec3 velocity_world = std::abs(request.speed) * d;

    auto pose = [&](double z, double trim) {
        State s;
        s.position = Vec3(out.still.pose.position.x(), out.still.pose.position.y(), z);
        s.orientation = heel_trim_orientation(request.heel, trim);
        s.linear_velocity = s.orientation.inverse() * velocity_world;
        return s;
    };
    // Normalised so that one unit is the weight, or the weight times a metre.
    auto residual = [&](double z, double trim) {
        const State s = pose(z, trim);
        const Wrench w = boat.evaluate(s, 0.0);
        const Mat3 r = s.rotation();
        return Eigen::Vector2d((r * w.force).z() / weight, (r * w.moment).y() / weight);
    };

    double z = out.still.pose.position.z();
    double trim = out.still.trim;
    constexpr double kTolerance = 1e-7;
    constexpr double kStepZ = 1e-4;
    constexpr double kStepTrim = 1e-4;
    constexpr int kMaxIterations = 50;
    for (int i = 0; i < kMaxIterations; ++i) {
        const Eigen::Vector2d r0 = residual(z, trim);
        out.iterations = i;
        if (r0.cwiseAbs().maxCoeff() <= kTolerance) {
            out.converged = true;
            break;
        }
        Eigen::Matrix2d jac;
        jac.col(0) = (residual(z + kStepZ, trim) - residual(z - kStepZ, trim)) / (2.0 * kStepZ);
        jac.col(1) =
            (residual(z, trim + kStepTrim) - residual(z, trim - kStepTrim)) / (2.0 * kStepTrim);
        if (!std::isfinite(jac.determinant()) || jac.determinant() == 0.0) break;
        const Eigen::Vector2d step = jac.inverse() * r0;
        z -= std::clamp(step.x(), -0.05, 0.05);
        trim -= std::clamp(step.y(), -0.02, 0.02);
    }

    out.state = pose(z, trim);
    out.trim = trim;
    const Wrench all = boat.evaluate(out.state, 0.0);
    const Mat3 r = out.state.rotation();
    const Vec3 force_world = r * all.force;
    const Vec3 moment_world = r * all.moment;
    out.tow_force_world = Vec3(-force_world.x(), -force_world.y(), 0.0);
    out.roll_constraint_Nm = -moment_world.x();
    out.yaw_constraint_Nm = -moment_world.z();

    Wrench damping;
    boat.damping().accumulate(out.state, 0.0, damping);
    out.canoe = resolve(r * boat.hull().last_result().wrench.force, d, n);
    out.keel = resolve(r * boat.appendages().keel().last_wrench().force, d, n);
    out.rudder = resolve(r * boat.appendages().rudder().last_wrench().force, d, n);
    out.damping = resolve(r * damping.force, d, n);
    const Vec3 hydrodynamic_world = r * (boat.hull().last_result().wrench.force +
                                         boat.appendages().keel().last_wrench().force +
                                         boat.appendages().rudder().last_wrench().force +
                                         damping.force);
    out.total = resolve(Vec3(force_world.x(), force_world.y(), hydrodynamic_world.z()), d, n);
    return out;
}

TurnResult run_turn(BoatSim& boat, const TurnRequest& request) {
    TurnResult out;
    TowRequest tow_request;
    tow_request.speed = request.speed;
    const TowResult straight = solve_tow(boat, tow_request);
    if (!straight.converged) return out;
    out.tow_converged = true;
    out.thrust = straight.tow_force_world.x();

    boat.thrust().set_thrust(out.thrust);
    Simulation& sim = boat.start(straight.state);
    boat.appendages().set_controls(Controls{request.tiller});

    const auto steps = static_cast<std::size_t>(std::llround(request.duration / Simulation::kFixedDt));
    out.samples.reserve(steps + 1);

    double heading_unwrapped = 0.0;
    double previous_heading = 0.0;
    auto record = [&](const State& s, double t, bool first) {
        const Mat3 r = s.rotation();
        const Vec3 fwd = r.col(0);
        const double heading = std::atan2(fwd.y(), fwd.x());
        if (first) {
            heading_unwrapped = heading;
        } else {
            heading_unwrapped += std::remainder(heading - previous_heading, 2.0 * std::numbers::pi);
        }
        previous_heading = heading;
        const Vec3 v = s.world_linear_velocity();
        const double along = v.x() * std::cos(heading) + v.y() * std::sin(heading);
        const double across = -v.x() * std::sin(heading) + v.y() * std::cos(heading);
        TurnSample p;
        p.t = t;
        p.x = s.position.x();
        p.y = s.position.y();
        p.heading = heading_unwrapped;
        p.yaw_rate = (r * s.angular_velocity).z();
        p.speed = std::hypot(v.x(), v.y());
        p.heel = heel_angle(r);
        p.drift = p.speed > 1e-6 ? std::atan2(across, along) : 0.0;
        p.rudder = boat.appendages().rudder_angle();
        out.samples.push_back(p);
    };

    record(sim.state(), 0.0, true);
    for (std::size_t i = 0; i < steps; ++i) {
        const double step_time = sim.time();
        sim.step();
        const State& s = sim.state();
        const double hinge = boat.appendages().rudder_hinge_moment_Nm();
        if (!finite(s) || !std::isfinite(hinge) || !std::isfinite(boat.appendages().rudder_angle())) {
            out.finite = false;
            out.first_nonfinite_time = sim.time();
            break;
        }
        if (std::abs(hinge) > std::abs(out.max_rudder_hinge_moment)) {
            out.max_rudder_hinge_moment = hinge;
            out.max_rudder_hinge_moment_time = step_time;
        }
        record(s, sim.time(), false);
    }

    const TurnSample& start = out.samples.front();
    const TurnSample& last = out.samples.back();
    out.heading_change = last.heading - start.heading;
    for (const TurnSample& p : out.samples) {
        if (std::abs(p.heading - start.heading) >= 0.5 * std::numbers::pi) {
            out.time_to_90deg = p.t;
            break;
        }
    }

    std::optional<std::size_t> window_begin;
    const char* window_kind = "";
    if (out.finite && std::abs(out.heading_change) >= 2.0 * std::numbers::pi) {
        std::size_t i = out.samples.size() - 1;
        while (i > 0 && std::abs(last.heading - out.samples[i].heading) < 2.0 * std::numbers::pi) --i;
        window_begin = i;
        window_kind = "last full revolution";
    } else if (out.finite && std::abs(out.heading_change) >= 0.5 * std::numbers::pi) {
        window_begin = out.samples.size() / 2;
        window_kind = "second half of the run";
    }
    if (window_begin) {
        const std::size_t b = *window_begin;
        const std::size_t e = out.samples.size();
        TurnSteady st;
        st.window = window_kind;
        st.t_begin = out.samples[b].t;
        st.t_end = out.samples[e - 1].t;
        st.heading_change = out.samples[e - 1].heading - out.samples[b].heading;
        for (std::size_t i = b; i < e; ++i) {
            st.yaw_rate += out.samples[i].yaw_rate;
            st.speed += out.samples[i].speed;
            st.heel += out.samples[i].heel;
            st.drift += out.samples[i].drift;
            st.rudder += out.samples[i].rudder;
        }
        const double n = static_cast<double>(e - b);
        st.yaw_rate /= n;
        st.speed /= n;
        st.heel /= n;
        st.drift /= n;
        st.rudder /= n;
        st.circle = fit_circle(out.samples, b, e);
        out.steady = std::move(st);
    }
    return out;
}

}  // namespace sail
