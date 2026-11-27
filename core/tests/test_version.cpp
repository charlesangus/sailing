#include <doctest/doctest.h>

#include <string>

#include "sail/version.hpp"

TEST_CASE("version is a non-empty dotted triple") {
    const std::string v(sail::version());
    CHECK_FALSE(v.empty());
    CHECK(v.find('.') != std::string::npos);
}

TEST_CASE("version matches the project version") { CHECK(sail::version() == SAIL_TEST_EXPECTED_VERSION); }
