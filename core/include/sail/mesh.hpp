#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "sail/rigid_body.hpp"

namespace sail {

// Indexed triangle mesh in flat arrays. Vertices are in the mesh's own frame; triangles index
// into `vertices` and wind counter-clockwise seen from outside, so the right-hand normal points
// outward.
struct TriMesh {
    std::vector<Vec3> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;

    std::size_t vertex_count() const { return vertices.size(); }
    std::size_t triangle_count() const { return triangles.size(); }
};

// Appends `src` to `dst`, offsetting its indices.
void append_mesh(TriMesh& dst, const TriMesh& src);

// Raised by load_obj. line() is 1-based, or 0 when the error concerns the file as a whole.
class ObjError : public std::runtime_error {
public:
    ObjError(std::filesystem::path file, std::size_t line, const std::string& message);

    const std::filesystem::path& file() const { return file_; }
    std::size_t line() const { return line_; }

private:
    std::filesystem::path file_;
    std::size_t line_;
};

// Loads the triangles of every object in a Wavefront OBJ file.
//
// Reads `v` and `f` records; `f` accepts `v`, `v/vt`, `v//vn` and `v/vt/vn` forms and negative
// (relative) indices, and polygons are fan-triangulated. `o` starts a named object; faces before
// the first `o` belong to an unnamed object. Other records, comments and blank lines are ignored.
// Only vertices referenced by the loaded faces are kept, in file order.
TriMesh load_obj(const std::filesystem::path& file);

// As above, but keeps only the faces of the named objects. Throws if any name is absent.
TriMesh load_obj(const std::filesystem::path& file, std::span<const std::string> objects);

}  // namespace sail
