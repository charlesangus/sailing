configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/version_config.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/version_config.hpp
    @ONLY
)

add_library(sailcore STATIC
    src/appendages.cpp
    src/boat_sim.cpp
    src/boat_data.cpp
    src/crew.cpp
    src/damping.cpp
    src/equilibrium.cpp
    src/foil.cpp
    src/gravity.cpp
    src/harness/autopilot.cpp
    src/harness/autotrim.cpp
    src/hull_forces.cpp
    src/hull_resistance.cpp
    src/hydrostatics.cpp
    src/mass_model.cpp
    src/mesh.cpp
    src/rigid_body.cpp
    src/sails.cpp
    src/sheets.cpp
    src/simulation.cpp
    src/version.cpp
    src/wind.cpp
    src/windage.cpp
)
add_library(sail::core ALIAS sailcore)
target_include_directories(sailcore
    PUBLIC include
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated
)
target_link_libraries(sailcore PUBLIC Eigen3::Eigen PRIVATE nlohmann_json::nlohmann_json)
target_compile_features(sailcore PUBLIC cxx_std_20)
set_target_properties(sailcore PROPERTIES POSITION_INDEPENDENT_CODE ON)
sail_enable_warnings(sailcore)
sail_optimise_debug(sailcore)

if(SAIL_BUILD_TESTS)
    FetchContent_Declare(
        doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG v2.4.11
        GIT_SHALLOW TRUE
    )
    set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
    set(DOCTEST_WITH_MAIN_IN_STATIC_LIB OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(doctest)
    include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)

    add_executable(sailcore_tests
        tests/test_appendages.cpp
        tests/test_bench_budget.cpp
        tests/test_crew.cpp
        tests/test_damping.cpp
        tests/test_foil.cpp
        tests/test_harness.cpp
        tests/test_hull_forces.cpp
        tests/test_hull_resistance.cpp
        tests/test_hydrostatics.cpp
        tests/test_j80_hydrostatics.cpp
        tests/test_j80_tow.cpp
        tests/test_j80_turn.cpp
        tests/test_main.cpp
        tests/test_mass_model.cpp
        tests/test_rigid_body.cpp
        tests/test_sails.cpp
        tests/test_sheets.cpp
        tests/test_version.cpp
        tests/test_wind.cpp
        tests/test_windage.cpp
    )
    target_link_libraries(sailcore_tests PRIVATE sailcore doctest::doctest nlohmann_json::nlohmann_json)
    target_compile_definitions(sailcore_tests PRIVATE
        SAIL_TEST_EXPECTED_VERSION="${PROJECT_VERSION}"
        SAIL_TEST_DATA_DIR="${PROJECT_SOURCE_DIR}/data"
    )
    sail_enable_warnings(sailcore_tests)
    sail_optimise_debug(sailcore_tests)
    doctest_discover_tests(sailcore_tests)
endif()
