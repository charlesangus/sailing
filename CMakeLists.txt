cmake_minimum_required(VERSION 3.24)
project(sailing VERSION 0.1.0 LANGUAGES CXX)

option(SAIL_BUILD_TESTS "Build the unit tests" ON)
option(SAIL_BUILD_GDEXT "Build the Godot GDExtension" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# Eigen's own CMake builds tests and docs; only its headers are needed, so skip it.
FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    URL_HASH SHA256=8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72
    SOURCE_SUBDIR sail-headers-only
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(eigen)
add_library(sail_eigen INTERFACE)
target_include_directories(sail_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
add_library(Eigen3::Eigen ALIAS sail_eigen)

# Header-only as well; SYSTEM keeps its headers out of our -Werror.
FetchContent_Declare(
    nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
    URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
    SOURCE_SUBDIR sail-headers-only
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(nlohmann_json)
add_library(sail_nlohmann_json INTERFACE)
target_include_directories(sail_nlohmann_json SYSTEM INTERFACE ${nlohmann_json_SOURCE_DIR}/include)
add_library(nlohmann_json::nlohmann_json ALIAS sail_nlohmann_json)

# stb_image decodes the scenes' 16-bit heightmaps. Single-header, so only its directory is used.
FetchContent_Declare(
    stb
    URL https://github.com/nothings/stb/archive/6b5799bae8a1fe3792691ef4b72ae26434c3e799.tar.gz
    URL_HASH SHA256=87e9ecd2d2e0cb8fa35bf5a93ea6ca3999595ea4f0a4738ed1aac0a34cbd4da9
    SOURCE_SUBDIR sail-headers-only
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(stb)
add_library(sail_stb INTERFACE)
target_include_directories(sail_stb SYSTEM INTERFACE ${stb_SOURCE_DIR})
add_library(stb::image ALIAS sail_stb)

# Applied per target so third-party code fetched above never sees -Werror.
function(sail_enable_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endfunction()

# Eigen at -O0 runs the per-triangle hydrostatics two orders of magnitude slower, which makes
# simulation tests unusable in Debug. -O1 inlines Eigen while keeping asserts and usable debug info.
function(sail_optimise_debug target)
    target_compile_options(${target} PRIVATE $<$<CONFIG:Debug>:-O1>)
endfunction()

# ctest only looks for CTestTestfile.cmake in the build root, so this must not live in core/.
if(SAIL_BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(core)
add_subdirectory(cli)

if(SAIL_BUILD_GDEXT AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/gdext/CMakeLists.txt)
    add_subdirectory(gdext)
endif()
