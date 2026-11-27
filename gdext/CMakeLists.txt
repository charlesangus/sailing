file(STRINGS ${PROJECT_SOURCE_DIR}/tools/godot.lock _sail_api_line REGEX "^GODOT_CPP_API_VERSION=")
string(REGEX REPLACE "^GODOT_CPP_API_VERSION=" "" _sail_api_version "${_sail_api_line}")
set(GODOTCPP_API_VERSION "${_sail_api_version}" CACHE STRING "Godot API version targeted by godot-cpp")
set(GODOTCPP_SYSTEM_HEADERS ON CACHE BOOL "Expose godot-cpp headers as SYSTEM")

add_subdirectory(${PROJECT_SOURCE_DIR}/extern/godot-cpp ${CMAKE_BINARY_DIR}/extern/godot-cpp)

add_library(sailgd SHARED
    src/register_types.cpp
    src/sail_sim.cpp
)
target_link_libraries(sailgd PRIVATE sail::core godot::cpp)
target_compile_features(sailgd PRIVATE cxx_std_20)
sail_enable_warnings(sailgd)

get_target_property(_sail_godotcpp_suffix godot-cpp GODOTCPP_SUFFIX)
set_target_properties(sailgd PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
    PREFIX "lib"
    OUTPUT_NAME "sailgd${_sail_godotcpp_suffix}"
    SUFFIX ".so"
    LIBRARY_OUTPUT_DIRECTORY "$<1:${PROJECT_SOURCE_DIR}/game/bin>"
)
