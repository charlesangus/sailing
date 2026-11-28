#include <doctest/doctest.h>

#include <Eigen/Eigenvalues>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "sail/boat_data.hpp"
#include "sail/mass_model.hpp"

namespace {

using sail::BoatData;
using sail::BoatDataError;
using sail::MassConfiguration;
using sail::MassItem;
using sail::MassModel;
using sail::MassShape;
using sail::Mat3;
using sail::Mat6;
using sail::Vec3;

const std::filesystem::path kBoatDir = std::filesystem::path(SAIL_TEST_DATA_DIR) / "boats" / "j80";

double sum_mass(const BoatData& data, bool in_displacement) {
    double m = 0.0;
    for (const MassItem& item : data.items) {
        if (item.in_displacement == in_displacement) m += item.mass;
    }
    return m;
}

const MassItem& residual_item(const BoatData& data) {
    for (const MassItem& item : data.items) {
        if (item.residual) return item;
    }
    FAIL("no residual item");
    return data.items.front();
}

void check_symmetric_positive_definite(const Mat3& m) {
    CHECK((m - m.transpose()).norm() <= 1e-9 * m.norm());
    const Eigen::SelfAdjointEigenSolver<Mat3> eig(m);
    CHECK(eig.eigenvalues().minCoeff() > 0.0);
}

std::string read_text(const std::filesystem::path& file) {
    std::ifstream in(file);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void write_text(const std::filesystem::path& file, const std::string& text) {
    std::ofstream out(file);
    out << text;
}

// A scratch copy of the dossier files the loader reads, for corrupting one at a time.
std::filesystem::path scratch_dossier(const std::string& name) {
    const auto dir = std::filesystem::temp_directory_path() / ("sail_test_mass_model_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    for (const char* file : {"mass.json", "particulars.json", "hull_hydro.json"}) {
        std::filesystem::copy_file(kBoatDir / file, dir / file);
    }
    return dir;
}

std::string load_error(const std::filesystem::path& dir) {
    try {
        sail::load_boat_data(dir);
    } catch (const BoatDataError& e) {
        return e.what();
    }
    return {};
}

}  // namespace

TEST_CASE("box and thin-shell inertias match their closed forms") {
    MassItem box{"box", 12.0, Vec3::Zero(), MassShape::box, Vec3(1.0, 2.0, 3.0), true, false};
    const Mat3 ib = sail::item_inertia(box);
    CHECK(ib(0, 0) == doctest::Approx(13.0));
    CHECK(ib(1, 1) == doctest::Approx(10.0));
    CHECK(ib(2, 2) == doctest::Approx(5.0));

    // A thin-shell cube of side s has I = 5/18 m s^2 about every axis.
    MassItem shell{"shell", 18.0, Vec3::Zero(), MassShape::shell_box, Vec3(2.0, 2.0, 2.0), true,
                   false};
    const Mat3 is = sail::item_inertia(shell);
    for (int i = 0; i < 3; ++i) CHECK(is(i, i) == doctest::Approx(5.0 / 18.0 * 18.0 * 4.0));
    CHECK((is - Mat3(is.diagonal().asDiagonal())).norm() == doctest::Approx(0.0));
}

TEST_CASE("combine applies the parallel-axis theorem") {
    std::vector<MassItem> items = {
        {"a", 1.0, Vec3(1.0, 0.0, 0.0), MassShape::point, Vec3::Zero(), true, false},
        {"b", 3.0, Vec3(-1.0, 0.0, 2.0), MassShape::point, Vec3::Zero(), true, false},
    };
    const sail::MassProperties p = sail::combine(items);
    CHECK(p.mass == doctest::Approx(4.0));
    CHECK(p.cg.x() == doctest::Approx(-0.5));
    CHECK(p.cg.z() == doctest::Approx(1.5));
    // Offsets from the CG: a (1.5, 0, -1.5), b (-0.5, 0, 0.5).
    CHECK(p.inertia(0, 0) == doctest::Approx(1.0 * 2.25 + 3.0 * 0.25));
    CHECK(p.inertia(1, 1) == doctest::Approx(1.0 * 4.5 + 3.0 * 0.5));
    CHECK(p.inertia(2, 2) == doctest::Approx(1.0 * 2.25 + 3.0 * 0.25));
    CHECK(p.inertia(0, 2) == doctest::Approx(-(1.0 * 1.5 * -1.5 + 3.0 * -0.5 * 0.5)));
    CHECK(p.inertia(2, 0) == doctest::Approx(p.inertia(0, 2)));
}

TEST_CASE("J/80 total mass matches the dossier in each configuration") {
    MassModel model = MassModel::load(kBoatDir);
    const BoatData& data = model.data();
    CHECK(data.displacement == doctest::Approx(1438.0));

    CHECK(model.configuration() == MassConfiguration::measurement_trim);
    CHECK(model.crew_mass() == 0.0);
    CHECK(model.total_mass() == doctest::Approx(data.displacement).epsilon(1e-12));
    CHECK(sum_mass(data, true) == doctest::Approx(1438.0).epsilon(1e-12));

    model.set_configuration(MassConfiguration::sailing);
    CHECK(model.crew_mass() == doctest::Approx(350.0));
    CHECK(model.total_mass() ==
          doctest::Approx(data.displacement + sum_mass(data, false) + 350.0).epsilon(1e-12));
    CHECK(model.total_mass() == doctest::Approx(1809.0).epsilon(1e-12));

    model.set_configuration(MassConfiguration::orc_stability);
    CHECK(model.crew_mass() == doctest::Approx(411.0 + 65.8));
    CHECK(model.total_mass() == doctest::Approx(1438.0 + 411.0 + 65.8).epsilon(1e-12));
    CHECK(model.crew_position().y() == 0.0);
    CHECK(model.crew_position().z() == doctest::Approx(0.708));
}

TEST_CASE("J/80 measurement-trim VCG equals the certificate VCGM") {
    const MassModel model = MassModel::load(kBoatDir);
    const BoatData& data = model.data();
    CHECK(data.vcgm == doctest::Approx(-0.076));
    CHECK(std::abs(model.cg().z() - data.vcgm) < 1e-9);
    CHECK(std::abs(model.cg().y()) < 1e-12);
}

TEST_CASE("J/80 residual is solved inside its plausible band") {
    const BoatData data = sail::load_boat_data(kBoatDir);
    const MassItem& res = residual_item(data);
    CHECK(res.name == "hull_deck_fittings");
    CHECK(res.in_displacement);
    CHECK(res.mass == doctest::Approx(1438.0 - (sum_mass(data, true) - res.mass)));
    CHECK(res.mass > 0.0);
    const double height = res.cg.z();
    CHECK(height >= 0.3);
    CHECK(height <= 1.1);
    CHECK(std::abs(res.cg.x() - data.hull.lcb_x_canoe) <= 0.5);
}

TEST_CASE("J/80 measurement-trim LCG equals the total LCB") {
    const MassModel model = MassModel::load(kBoatDir);
    const BoatData& data = model.data();
    CHECK(std::abs(model.cg().x() - data.hull.lcb_x_total) < 1e-9);
    CHECK(std::abs(residual_item(data).cg.x() - data.hull.lcb_x_canoe) <= 0.5);
}

TEST_CASE("moving the crew shifts the CG by crew mass over total mass") {
    MassModel model = MassModel::load(kBoatDir, MassConfiguration::sailing);
    const double total = model.total_mass();
    const double crew = model.crew_mass();
    const Vec3 cg0 = model.cg();
    const Vec3 crew0 = model.crew_position();

    const double dy = 1.15;
    const double dz = 0.1;
    model.set_crew_position(crew0.y() + dy, crew0.z() + dz);
    CHECK(model.total_mass() == doctest::Approx(total));
    CHECK(model.crew_position().x() == doctest::Approx(crew0.x()));
    const Vec3 shift = model.cg() - cg0;
    CHECK(shift.x() == doctest::Approx(0.0));
    CHECK(shift.y() == doctest::Approx(crew / total * dy));
    CHECK(shift.z() == doctest::Approx(crew / total * dz));

    model.set_crew_mass(200.0);
    CHECK(model.total_mass() == doctest::Approx(total - crew + 200.0));
    CHECK_THROWS_AS(model.set_crew_mass(-1.0), std::invalid_argument);
}

TEST_CASE("J/80 inertia stays symmetric positive definite") {
    MassModel model = MassModel::load(kBoatDir);
    for (const auto config : {MassConfiguration::measurement_trim, MassConfiguration::sailing,
                              MassConfiguration::orc_stability}) {
        model.set_configuration(config);
        check_symmetric_positive_definite(model.inertia());
        model.set_crew_position(1.2, 0.8);
        check_symmetric_positive_definite(model.inertia());

        const sail::BodyProperties body = model.body_properties();
        CHECK(body.mass == doctest::Approx(model.total_mass()));
        const Mat6 m = body.generalised_mass();
        CHECK((m - m.transpose()).norm() <= 1e-9 * m.norm());
        const Eigen::SelfAdjointEigenSolver<Mat6> eig(m);
        CHECK(eig.eigenvalues().minCoeff() > 0.0);
    }
}

TEST_CASE("J/80 added mass is a positive diagonal with surge at its mass fraction") {
    const MassModel model = MassModel::load(kBoatDir, MassConfiguration::sailing);
    const Mat6 a = model.added_mass();
    CHECK((a - Mat6(a.diagonal().asDiagonal())).norm() == 0.0);
    CHECK(a(0, 0) == doctest::Approx(model.data().added_mass.surge_fraction * model.total_mass()));
    for (int i = 0; i < 6; ++i) CHECK(a(i, i) > 0.0);

    sail::AddedMassCoefficients doubled = model.data().added_mass;
    doubled.heave_scale *= 2.0;
    const Mat6 b = sail::strip_added_mass(model.data().hull, doubled, model.data().density,
                                          model.total_mass());
    CHECK(b(2, 2) == doctest::Approx(2.0 * a(2, 2)));
    CHECK(b(1, 1) == doctest::Approx(a(1, 1)));
}

TEST_CASE("boat data errors name the file and the problem") {
    SUBCASE("missing file") {
        const auto dir = scratch_dossier("missing");
        std::filesystem::remove(dir / "mass.json");
        const std::string msg = load_error(dir);
        CHECK(msg.find("mass.json") != std::string::npos);
        CHECK(msg.find("cannot open") != std::string::npos);
        std::filesystem::remove_all(dir);
    }
    SUBCASE("malformed JSON") {
        const auto dir = scratch_dossier("malformed");
        write_text(dir / "mass.json", "{ \"items\": [1, 2,");
        const std::string msg = load_error(dir);
        CHECK(msg.find("mass.json") != std::string::npos);
        CHECK(msg.find("malformed JSON") != std::string::npos);
        std::filesystem::remove_all(dir);
    }
    SUBCASE("missing field") {
        const auto dir = scratch_dossier("field");
        std::string text = read_text(dir / "particulars.json");
        const auto at = text.find("\"vcgm\"");
        REQUIRE(at != std::string::npos);
        text.replace(at, 6, "\"vcgm_renamed\"");
        write_text(dir / "particulars.json", text);
        const std::string msg = load_error(dir);
        CHECK(msg.find("particulars.json") != std::string::npos);
        CHECK(msg.find("missing field 'vcgm'") != std::string::npos);
        std::filesystem::remove_all(dir);
    }
    SUBCASE("solved residual height outside its bounds") {
        const auto dir = scratch_dossier("bounds");
        std::string text = read_text(dir / "mass.json");
        const std::string bounds = "\"value\": [0.3, 1.1]";
        const auto at = text.find(bounds);
        REQUIRE(at != std::string::npos);
        text.replace(at, bounds.size(), "\"value\": [0.9, 1.1]");
        write_text(dir / "mass.json", text);
        const std::string msg = load_error(dir);
        CHECK(msg.find("hull_deck_fittings") != std::string::npos);
        CHECK(msg.find("cg_z_bounds") != std::string::npos);
        std::filesystem::remove_all(dir);
    }
}
