#include "sail/calibration.hpp"

#include <array>
#include <string>
#include <utility>

#include "dossier_json.hpp"

namespace sail {

Calibration load_calibration(const std::filesystem::path& boat_dir) {
    Calibration out;
    const std::filesystem::path file = boat_dir / "calibration.json";
    if (!std::filesystem::exists(file)) return out;

    const dossier::Document doc = dossier::read_document(file);
    const dossier::Node root = doc.root();
    const std::array<std::pair<const char*, double*>, 9> entries = {{
        {"main_lift", &out.main_lift},
        {"jib_lift", &out.jib_lift},
        {"main_parasitic_drag", &out.main_parasitic_drag},
        {"jib_parasitic_drag", &out.jib_parasitic_drag},
        {"effective_height", &out.effective_height},
        {"kheff_upwind_rise", &out.kheff_upwind_rise},
        {"headsail_blanketing", &out.headsail_blanketing},
        {"hull_resistance", &out.hull_resistance},
        {"windage", &out.windage},
    }};
    for (auto it = root.json().begin(); it != root.json().end(); ++it) {
        if (!it->is_object() || !it->contains("value")) continue;
        double* target = nullptr;
        for (const auto& [name, field] : entries) {
            if (it.key() == name) target = field;
        }
        const dossier::Node entry = root.at(it.key());
        if (!target) entry.fail("unknown calibration entry");
        *target = entry.positive_value();
    }
    return out;
}

}  // namespace sail
