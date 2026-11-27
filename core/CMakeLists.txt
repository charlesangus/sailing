configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/version_config.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/version_config.hpp
    @ONLY
)

add_library(sailcore STATIC src/version.cpp)
add_library(sail::core ALIAS sailcore)
target_include_directories(sailcore
    PUBLIC include
    PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated
)
target_link_libraries(sailcore PUBLIC Eigen3::Eigen)
target_compile_features(sailcore PUBLIC cxx_std_20)
sail_enable_warnings(sailcore)

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

    add_executable(sailcore_tests tests/test_main.cpp tests/test_version.cpp)
    target_link_libraries(sailcore_tests PRIVATE sailcore doctest::doctest)
    sail_enable_warnings(sailcore_tests)
    doctest_discover_tests(sailcore_tests)
endif()
