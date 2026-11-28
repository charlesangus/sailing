#include "sail/mesh.hpp"

#include <charconv>
#include <fstream>
#include <limits>
#include <string_view>
#include <system_error>
#include <utility>

namespace sail {

namespace {

std::string format_message(const std::filesystem::path& file, std::size_t line,
                           const std::string& message) {
    std::string out = file.string();
    if (line > 0) out += ":" + std::to_string(line);
    out += ": " + message;
    return out;
}

bool is_space(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v'; }

// Splits off the next whitespace-delimited token from `rest`; empty when none remain.
std::string_view next_token(std::string_view& rest) {
    std::size_t begin = 0;
    while (begin < rest.size() && is_space(rest[begin])) ++begin;
    std::size_t end = begin;
    while (end < rest.size() && !is_space(rest[end])) ++end;
    const std::string_view token = rest.substr(begin, end - begin);
    rest.remove_prefix(end);
    return token;
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && is_space(s.front())) s.remove_prefix(1);
    while (!s.empty() && is_space(s.back())) s.remove_suffix(1);
    return s;
}

class Parser {
public:
    Parser(const std::filesystem::path& file, std::span<const std::string> objects, bool all)
        : file_(file), objects_(objects), all_(all), found_(objects.size(), false) {}

    TriMesh run() {
        std::ifstream in(file_);
        if (!in) throw ObjError(file_, 0, "cannot open file");

        std::string line;
        while (std::getline(in, line)) {
            ++line_no_;
            parse_line(line);
        }
        if (in.bad()) throw ObjError(file_, 0, "read error");

        for (std::size_t i = 0; i < objects_.size(); ++i) {
            if (!found_[i]) throw ObjError(file_, 0, "no object named '" + objects_[i] + "'");
        }
        return compact();
    }

private:
    [[noreturn]] void fail(const std::string& message) const {
        throw ObjError(file_, line_no_, message);
    }

    void parse_line(std::string_view rest) {
        const std::size_t hash = rest.find('#');
        if (hash != std::string_view::npos) rest = rest.substr(0, hash);
        const std::string_view keyword = next_token(rest);
        if (keyword.empty()) return;
        if (keyword == "v") {
            parse_vertex(rest);
        } else if (keyword == "f") {
            parse_face(rest);
        } else if (keyword == "o") {
            select_object(trim(rest));
        }
    }

    double parse_double(std::string_view token) const {
        if (token.empty()) fail("missing vertex coordinate");
        if (token.front() == '+') token.remove_prefix(1);
        double value = 0.0;
        const auto [ptr, ec] = std::from_chars(token.data(), token.data() + token.size(), value);
        if (ec != std::errc() || ptr != token.data() + token.size()) {
            fail("invalid number '" + std::string(token) + "'");
        }
        return value;
    }

    void parse_vertex(std::string_view rest) {
        Vec3 v;
        for (int i = 0; i < 3; ++i) v[i] = parse_double(next_token(rest));
        vertices_.push_back(v);
    }

    std::uint32_t resolve_index(std::string_view token) const {
        const std::string_view head = token.substr(0, token.find('/'));
        long long index = 0;
        const auto [ptr, ec] = std::from_chars(head.data(), head.data() + head.size(), index);
        if (head.empty() || ec != std::errc() || ptr != head.data() + head.size()) {
            fail("invalid face index '" + std::string(token) + "'");
        }
        const auto count = static_cast<long long>(vertices_.size());
        const long long zero_based = index > 0 ? index - 1 : count + index;
        if (index == 0 || zero_based < 0 || zero_based >= count) {
            fail("face index " + std::to_string(index) + " out of range (" +
                 std::to_string(count) + " vertices so far)");
        }
        return static_cast<std::uint32_t>(zero_based);
    }

    void parse_face(std::string_view rest) {
        polygon_.clear();
        for (std::string_view token = next_token(rest); !token.empty(); token = next_token(rest)) {
            polygon_.push_back(resolve_index(token));
        }
        if (polygon_.size() < 3) fail("face has fewer than 3 vertices");
        if (!active_) return;
        for (std::size_t i = 1; i + 1 < polygon_.size(); ++i) {
            triangles_.push_back({polygon_[0], polygon_[i], polygon_[i + 1]});
        }
    }

    void select_object(std::string_view name) {
        if (all_) return;
        active_ = false;
        for (std::size_t i = 0; i < objects_.size(); ++i) {
            if (objects_[i] == name) {
                active_ = true;
                found_[i] = true;
            }
        }
    }

    TriMesh compact() const {
        if (vertices_.size() > std::numeric_limits<std::uint32_t>::max()) {
            throw ObjError(file_, 0, "too many vertices");
        }
        constexpr std::uint32_t kUnused = std::numeric_limits<std::uint32_t>::max();
        std::vector<std::uint32_t> remap(vertices_.size(), kUnused);
        for (const auto& tri : triangles_) {
            for (const std::uint32_t i : tri) remap[i] = 0;
        }
        TriMesh mesh;
        for (std::size_t i = 0; i < vertices_.size(); ++i) {
            if (remap[i] == kUnused) continue;
            remap[i] = static_cast<std::uint32_t>(mesh.vertices.size());
            mesh.vertices.push_back(vertices_[i]);
        }
        mesh.triangles.reserve(triangles_.size());
        for (const auto& tri : triangles_) {
            mesh.triangles.push_back({remap[tri[0]], remap[tri[1]], remap[tri[2]]});
        }
        return mesh;
    }

    const std::filesystem::path& file_;
    std::span<const std::string> objects_;
    bool all_;
    bool active_ = all_;
    std::vector<bool> found_;
    std::size_t line_no_ = 0;
    std::vector<Vec3> vertices_;
    std::vector<std::array<std::uint32_t, 3>> triangles_;
    std::vector<std::uint32_t> polygon_;
};

}  // namespace

ObjError::ObjError(std::filesystem::path file, std::size_t line, const std::string& message)
    : std::runtime_error(format_message(file, line, message)), file_(std::move(file)), line_(line) {}

void append_mesh(TriMesh& dst, const TriMesh& src) {
    const auto offset = static_cast<std::uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    dst.triangles.reserve(dst.triangles.size() + src.triangles.size());
    for (const auto& tri : src.triangles) {
        dst.triangles.push_back({tri[0] + offset, tri[1] + offset, tri[2] + offset});
    }
}

TriMesh load_obj(const std::filesystem::path& file) {
    return Parser(file, {}, true).run();
}

TriMesh load_obj(const std::filesystem::path& file, std::span<const std::string> objects) {
    return Parser(file, objects, false).run();
}

}  // namespace sail
