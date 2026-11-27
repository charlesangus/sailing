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
    SOURCE_SUBDIR sail-headers-only
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(eigen)
add_library(sail_eigen INTERFACE)
target_include_directories(sail_eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
add_library(Eigen3::Eigen ALIAS sail_eigen)

# Applied per target so third-party code fetched above never sees -Werror.
function(sail_enable_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
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
