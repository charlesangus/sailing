#include "sail/grounding.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <map>
#include <numbers>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>

#include <stb_image.h>

#include "dossier_json.hpp"
#include "sail/mesh.hpp"

namespace sail {

namespace {

struct Axis {
    std::size_t lo = 0;
    double frac = 0.0;
    // False when the coordinate was clamped onto an edge, where the surface is flat.
    bool inside = false;
};

Axis locate(double coord, std::size_t size) {
    Axis a;
    const double last = static_cast<double>(size - 1);
    a.inside = coord > 0.0 && coord < last;
    const double c = std::clamp(coord, 0.0, last);
    if (size < 2) return a;
    a.lo = std::min(static_cast<std::size_t>(c), size - 2);
    a.frac = c - static_cast<double>(a.lo);
    return a;
}

std::pair<std::size_t, std::size_t> node_range(double lo, double hi, std::size_t size) {
    const double last = static_cast<double>(size - 1);
    const auto first = static_cast<std::size_t>(std::floor(std::clamp(lo, 0.0, last)));
    const auto end = static_cast<std::size_t>(std::ceil(std::clamp(hi, 0.0, last)));
    return {first, end};
}

std::vector<unsigned char> read_file(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) throw std::runtime_error("cannot open '" + file.string() + "'");
    return std::vector<unsigned char>(std::istreambuf_iterator<char>(in),
                                      std::istreambuf_iterator<char>());
}

std::vector<Vec3> object_vertices(const std::filesystem::path& file, const std::string& name) {
    const std::vector<std::string> names = {name};
    return load_obj(file, names).vertices;
}

void positive(double v, const char* what) {
    if (!(v > 0.0) || !std::isfinite(v)) {
        throw std::invalid_argument(std::string("GroundingModule: ") + what + " must be positive");
    }
}

}  // namespace

Bathymetry::Bathymetry(std::size_t width, std::size_t rows, double pixel_size, double origin_x,
                       double origin_y, std::vector<float> heights)
    : width_(width),
      rows_(rows),
      pixel_(pixel_size),
      origin_x_(origin_x),
      origin_y_(origin_y),
      heights_(std::move(heights)) {
    if (width_ == 0 || rows_ == 0) throw std::invalid_argument("Bathymetry: empty grid");
    if (heights_.size() != width_ * rows_) {
        throw std::invalid_argument("Bathymetry: heights do not match the grid size");
    }
    if (!(pixel_ > 0.0) || !std::isfinite(pixel_)) {
        throw std::invalid_argument("Bathymetry: pixel size must be positive");
    }
    if (!std::isfinite(origin_x_) || !std::isfinite(origin_y_)) {
        throw std::invalid_argument("Bathymetry: origin must be finite");
    }
    for (const float h : heights_) {
        if (!std::isfinite(h)) throw std::invalid_argument("Bathymetry: non-finite height");
    }
}

Bathymetry Bathymetry::load(const std::filesystem::path& meta_json) {
    const dossier::Document doc = dossier::read_document(meta_json);
    const dossier::Node root = doc.root();
    const dossier::Node map = root.at("heightmap");
    const std::filesystem::path image = meta_json.parent_path() / map.at("file").string();
    const double width_px = map.at("width_px").number();
    const double rows_px = map.at("height_px").number();
    const double pixel = root.at("pixel_size_m").number();
    if (!(pixel > 0.0)) root.at("pixel_size_m").fail("expected a positive number");
    const double ox = root.at("origin").at("x_m").number();
    const double oy = root.at("origin").at("y_m").number();
    const double scale = root.at("height_encoding").at("scale_m").number();
    const double offset = root.at("height_encoding").at("offset_m").number();

    const std::vector<unsigned char> bytes = read_file(image);
    if (bytes.size() > static_cast<std::size_t>(INT32_MAX)) {
        throw std::runtime_error("'" + image.string() + "' is too large");
    }
    int w = 0;
    int h = 0;
    int channels = 0;
    stbi_us* pixels = stbi_load_16_from_memory(bytes.data(), static_cast<int>(bytes.size()), &w,
                                               &h, &channels, 1);
    if (pixels == nullptr) {
        throw std::runtime_error("cannot decode '" + image.string() +
                                 "': " + stbi_failure_reason());
    }
    const std::unique_ptr<stbi_us, void (*)(void*)> owner(pixels, stbi_image_free);
    if (channels != 1) {
        throw std::runtime_error("'" + image.string() + "' is not a greyscale image");
    }
    if (static_cast<double>(w) != width_px || static_cast<double>(h) != rows_px) {
        throw std::runtime_error("'" + image.string() + "' is " + std::to_string(w) + "x" +
                                 std::to_string(h) + ", not the size meta.json gives");
    }
    const auto count = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    std::vector<float> heights(count);
    for (std::size_t i = 0; i < count; ++i) {
        heights[i] = static_cast<float>(static_cast<double>(pixels[i]) * scale + offset);
    }
    return Bathymetry(static_cast<std::size_t>(w), static_cast<std::size_t>(h), pixel, ox, oy,
                      std::move(heights));
}

SeabedSample Bathymetry::sample(double x, double y) const {
    SeabedSample s;
    if (heights_.empty()) return s;
    const Axis c = locate((x - origin_x_) / pixel_, width_);
    const Axis r = locate((origin_y_ - y) / pixel_, rows_);
    const std::size_t c1 = width_ > 1 ? c.lo + 1 : c.lo;
    const std::size_t r1 = rows_ > 1 ? r.lo + 1 : r.lo;
    const double h00 = node(c.lo, r.lo);
    const double h10 = node(c1, r.lo);
    const double h01 = node(c.lo, r1);
    const double h11 = node(c1, r1);
    const double top = h00 + (h10 - h00) * c.frac;
    const double bottom = h01 + (h11 - h01) * c.frac;
    s.height = top + (bottom - top) * r.frac;
    if (c.inside) {
        const double dcol = (h10 - h00) * (1.0 - r.frac) + (h11 - h01) * r.frac;
        s.dzdx = dcol / pixel_;
    }
    if (r.inside) s.dzdy = -(bottom - top) / pixel_;
    return s;
}

double Bathymetry::max_height(double x_min, double x_max, double y_min, double y_max) const {
    if (heights_.empty()) return -std::numeric_limits<double>::infinity();
    if (!std::isfinite(x_min) || !std::isfinite(x_max) || !std::isfinite(y_min) ||
        !std::isfinite(y_max)) {
        return std::numeric_limits<double>::infinity();
    }
    const auto [c0, c1] = node_range((x_min - origin_x_) / pixel_, (x_max - origin_x_) / pixel_,
                                     width_);
    const auto [r0, r1] = node_range((origin_y_ - y_max) / pixel_, (origin_y_ - y_min) / pixel_,
                                     rows_);
    float best = -std::numeric_limits<float>::infinity();
    for (std::size_t row = r0; row <= r1; ++row) {
        const float* line = heights_.data() + row * width_;
        for (std::size_t col = c0; col <= c1; ++col) best = std::max(best, line[col]);
    }
    return static_cast<double>(best);
}

std::vector<Vec3> decimate_lowest(std::span<const Vec3> points, double cell) {
    if (!(cell > 0.0) || !std::isfinite(cell)) {
        throw std::invalid_argument("decimate_lowest: cell must be positive");
    }
    using Key = std::tuple<std::int64_t, std::int64_t, std::int64_t>;
    std::map<Key, std::size_t> slot;
    std::vector<Vec3> out;
    for (const Vec3& p : points) {
        const Key key{static_cast<std::int64_t>(std::floor(p.x() / cell)),
                      static_cast<std::int64_t>(std::floor(p.y() / cell)),
                      static_cast<std::int64_t>(std::floor(p.z() / cell))};
        const auto [it, fresh] = slot.try_emplace(key, out.size());
        if (fresh) {
            out.push_back(p);
        } else if (p.z() < out[it->second].z()) {
            out[it->second] = p;
        }
    }
    return out;
}

std::vector<Vec3> load_contact_points(const std::filesystem::path& boat_dir) {
    constexpr double kHullCell = 0.3;
    constexpr double kAppendageCell = 0.08;
    const std::filesystem::path appendages = boat_dir / "appendages.obj";
    std::vector<Vec3> out =
        decimate_lowest(load_obj(boat_dir / "hull_physics.obj").vertices, kHullCell);
    for (const char* name : {"keel", "rudder"}) {
        const std::vector<Vec3> part =
            decimate_lowest(object_vertices(appendages, name), kAppendageCell);
        out.insert(out.end(), part.begin(), part.end());
    }
    return out;
}

double GroundingParams::stiffness() const {
    const double omega = 2.0 * std::numbers::pi * natural_frequency;
    return mass * omega * omega;
}

double GroundingParams::damping() const {
    return 2.0 * damping_ratio * std::sqrt(stiffness() * mass);
}

GroundingModule::GroundingModule(std::vector<Vec3> points, GroundingParams params,
                                 std::shared_ptr<const Bathymetry> seabed)
    : points_(std::move(points)), params_(params), seabed_(std::move(seabed)) {
    positive(params_.mass, "mass");
    positive(params_.natural_frequency, "natural frequency");
    positive(params_.slip_speed, "slip speed");
    positive(params_.shared_contacts, "shared contacts");
    positive(params_.engage_depth, "engage depth");
    if (!(params_.damping_ratio >= 0.0) || !std::isfinite(params_.damping_ratio)) {
        throw std::invalid_argument("GroundingModule: damping ratio must be >= 0");
    }
    if (!(params_.friction >= 0.0) || !std::isfinite(params_.friction)) {
        throw std::invalid_argument("GroundingModule: friction must be >= 0");
    }
    stiffness_ = params_.stiffness();
    damping_ = params_.damping();
    if (!points_.empty()) {
        box_min_ = box_max_ = points_.front();
        for (const Vec3& p : points_) {
            box_min_ = box_min_.cwiseMin(p);
            box_max_ = box_max_.cwiseMax(p);
        }
    }
}

void GroundingModule::accumulate(const State& state, double, Wrench& wrench) {
    last_ = GroundingResult{};
    if (!seabed_ || !seabed_->has_seabed() || points_.empty()) return;

    const Mat3 r = state.rotation();
    const Vec3& ref = params_.reference_point;
    Vec3 lo = Vec3::Constant(std::numeric_limits<double>::infinity());
    Vec3 hi = -lo;
    for (int corner = 0; corner < 8; ++corner) {
        const Vec3 c((corner & 1) ? box_max_.x() : box_min_.x(),
                     (corner & 2) ? box_max_.y() : box_min_.y(),
                     (corner & 4) ? box_max_.z() : box_min_.z());
        const Vec3 w = r * (c - ref);
        lo = lo.cwiseMin(w);
        hi = hi.cwiseMax(w);
    }
    const Vec3& at = state.position;
    const double ceiling =
        seabed_->max_height(at.x() + lo.x(), at.x() + hi.x(), at.y() + lo.y(), at.y() + hi.y());
    if (ceiling < at.z() + lo.z()) return;
    last_.tested = true;

    const Vec3 vertical = r.row(2).transpose();
    const double k = stiffness_ / params_.shared_contacts;
    const double c = damping_ / params_.shared_contacts;

    // The sum of engagement weights is needed before any force, so the first pass only counts.
    double engaged = 0.0;
    for (const Vec3& p : points_) {
        const Vec3 arm = p - ref;
        const double z = at.z() + vertical.dot(arm);
        if (z >= ceiling) continue;
        const Vec3 world = at + r * arm;
        const SeabedSample s = seabed_->sample(world.x(), world.y());
        const double depth = (s.height - world.z()) * s.normal().z();
        if (depth > 0.0) engaged += std::min(depth / params_.engage_depth, 1.0);
    }
    const double share =
        engaged > params_.shared_contacts ? params_.shared_contacts / engaged : 1.0;

    const double slip2 = params_.slip_speed * params_.slip_speed;
    for (const Vec3& p : points_) {
        const Vec3 arm = p - ref;
        const double z = at.z() + vertical.dot(arm);
        if (z >= ceiling) continue;
        const Vec3 arm_world = r * arm;
        const Vec3 world = at + arm_world;
        const SeabedSample s = seabed_->sample(world.x(), world.y());
        const Vec3 n = s.normal();
        const double depth = (s.height - world.z()) * n.z();
        if (!(depth > 0.0)) continue;
        ++last_.contacts;
        last_.max_penetration = std::max(last_.max_penetration, depth);
        const double weight = std::min(depth / params_.engage_depth, 1.0);
        const Vec3 velocity =
            r * (state.linear_velocity + state.angular_velocity.cross(arm));
        const double closing = -velocity.dot(n);
        const double normal = std::max(share * (k * depth + c * weight * closing), 0.0);
        const Vec3 slip = velocity + closing * n;
        const Vec3 force = normal * n - (params_.friction * normal /
                                         std::sqrt(slip.squaredNorm() + slip2)) * slip;
        last_.normal_force += normal;
        last_.force_world += force;
        last_.moment_world += arm_world.cross(force);
    }
    last_.wrench.force = r.transpose() * last_.force_world;
    last_.wrench.moment = r.transpose() * last_.moment_world;
    wrench += last_.wrench;
}

}  // namespace sail
