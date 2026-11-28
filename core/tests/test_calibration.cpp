#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <numbers>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "sail/boat_data.hpp"
#include "sail/calibration.hpp"
#include "sail/hull_resistance.hpp"
#include "sail/wind.hpp"

namespace {

const std::filesystem::path kBoatDir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";

std::filesystem::path scratch_dir(const std::string& name, const nlohmann::json* calibration) {
    const auto dir = std::filesystem::temp_directory_path() / ("sail_test_calibration_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    if (calibration != nullptr) {
        std::ofstream out(dir / "calibration.json");
        out << calibration->dump(2);
    }
    return dir;
}

class ConstantResistance final : public sail::HullResistanceModel {
public:
    sail::HullResistance resistance(double, double) const override {
        sail::HullResistance r;
        r.residuary_N = 10.0;
        r.viscous_N = 20.0;
        r.total_N = 30.0;
        r.fn = 0.3;
        return r;
    }
    sail::HullResistanceKind kind() const override { return sail::HullResistanceKind::dsyhs; }
};

}  // namespace

TEST_CASE("an absent calibration.json gives all ones") {
    const sail::Calibration c = sail::load_calibration(scratch_dir("absent", nullptr));
    CHECK(c.main_lift == 1.0);
    CHECK(c.jib_lift == 1.0);
    CHECK(c.main_parasitic_drag == 1.0);
    CHECK(c.jib_parasitic_drag == 1.0);
    CHECK(c.effective_height == 1.0);
    CHECK(c.kheff_upwind_rise == 1.0);
    CHECK(c.headsail_blanketing == 1.0);
    CHECK(c.hull_resistance == 1.0);
    CHECK(c.windage == 1.0);
}

TEST_CASE("calibration.json entries set their factors and leave the rest at one") {
    const nlohmann::json doc = {{"description", "test"},
                                {"jib_lift", {{"value", 1.1}, {"rationale", "x"}}},
                                {"windage", {{"value", 0.8}}}};
    const sail::Calibration c = sail::load_calibration(scratch_dir("partial", &doc));
    CHECK(c.jib_lift == doctest::Approx(1.1));
    CHECK(c.windage == doctest::Approx(0.8));
    CHECK(c.main_lift == 1.0);
    CHECK(c.hull_resistance == 1.0);
}

TEST_CASE("calibration.json rejects unknown entries and non-positive values") {
    const nlohmann::json unknown = {{"keel_lift", {{"value", 1.1}}}};
    CHECK_THROWS_AS(sail::load_calibration(scratch_dir("unknown", &unknown)),
                    sail::BoatDataError);
    const nlohmann::json zero = {{"main_lift", {{"value", 0.0}}}};
    CHECK_THROWS_AS(sail::load_calibration(scratch_dir("zero", &zero)), sail::BoatDataError);
}

TEST_CASE("the J/80 calibration file loads") {
    const sail::Calibration c = sail::load_calibration(kBoatDir);
    CHECK(c.kheff_upwind_rise > 0.0);
    CHECK(c.kheff_upwind_rise <= 1.0);
}

TEST_CASE("ScaledHullResistance scales the resistance and keeps the rest") {
    const sail::ScaledHullResistance scaled(std::make_unique<ConstantResistance>(), 1.2);
    const sail::HullResistance r = scaled.resistance(2.0, 0.0);
    CHECK(r.residuary_N == doctest::Approx(12.0));
    CHECK(r.viscous_N == doctest::Approx(24.0));
    CHECK(r.total_N == doctest::Approx(36.0));
    CHECK(r.fn == doctest::Approx(0.3));
    CHECK(scaled.kind() == sail::HullResistanceKind::dsyhs);
    CHECK_THROWS_AS(sail::ScaledHullResistance(nullptr, 1.0), std::invalid_argument);
    CHECK_THROWS_AS(sail::ScaledHullResistance(std::make_unique<ConstantResistance>(), 0.0),
                    std::invalid_argument);
}

TEST_CASE("LogProfileWind follows the ORC log profile about its 10 m reference") {
    const double z0 = 0.005;
    const sail::LogProfileWind wind(5.0, 0.0);
    const sail::Vec3 at10 = wind.sample(sail::Vec3(3.0, -2.0, 10.0), 0.0);
    CHECK(at10.norm() == doctest::Approx(5.0));
    CHECK(at10.y() == doctest::Approx(-5.0));
    const double ratio = std::log(2.0 / z0) / std::log(10.0 / z0);
    CHECK(wind.sample(sail::Vec3(0.0, 0.0, 2.0), 0.0).norm() == doctest::Approx(5.0 * ratio));
    const double floor = 5.0 * std::log(10.0) / std::log(10.0 / z0);
    CHECK(wind.sample(sail::Vec3(0.0, 0.0, 0.01), 0.0).norm() == doctest::Approx(floor));
    CHECK(wind.sample(sail::Vec3(0.0, 0.0, -1.0), 0.0).norm() == doctest::Approx(floor));
    CHECK_THROWS_AS(sail::LogProfileWind(5.0, 0.0, 10.0, 0.0), std::invalid_argument);
    CHECK_THROWS_AS(sail::LogProfileWind(5.0, 0.0, 0.01, 0.005), std::invalid_argument);
}
