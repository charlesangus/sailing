#include "common.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <numbers>
#include <utility>

namespace sailcli {

Options Options::parse(const Args& args, const std::vector<std::string_view>& allowed) {
    Options options;
    for (std::size_t i = 0; i < args.size(); ++i) {
        std::string_view arg = args[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
            continue;
        }
        if (!arg.starts_with("--")) throw UsageError("unexpected argument '" + std::string(arg) + "'");
        std::string_view name = arg;
        std::optional<std::string_view> value;
        if (const auto eq = arg.find('='); eq != std::string_view::npos) {
            name = arg.substr(0, eq);
            value = arg.substr(eq + 1);
        }
        if (std::find(allowed.begin(), allowed.end(), name) == allowed.end()) {
            throw UsageError("unknown option '" + std::string(name) + "'");
        }
        if (!value) {
            if (i + 1 >= args.size()) throw UsageError("option '" + std::string(name) + "' needs a value");
            value = args[++i];
        }
        options.values.insert_or_assign(std::string(name), std::string(*value));
    }
    return options;
}

std::optional<std::string> Options::get(std::string_view name) const {
    const auto it = values.find(name);
    if (it == values.end()) return std::nullopt;
    return it->second;
}

double Options::number(std::string_view name, double fallback) const {
    const auto text = get(name);
    if (!text) return fallback;
    double v = 0.0;
    const char* end = text->data() + text->size();
    const auto [ptr, ec] = std::from_chars(text->data(), end, v);
    if (ec != std::errc() || ptr != end || !std::isfinite(v)) {
        throw UsageError("option '" + std::string(name) + "' expects a number, got '" + *text + "'");
    }
    return v;
}

ConfigChoice parse_config(const std::optional<std::string>& value) {
    const std::string name = value.value_or("measurement");
    if (name == "measurement") return {name, sail::MassConfiguration::measurement_trim};
    if (name == "sailing") return {name, sail::MassConfiguration::sailing};
    if (name == "orc") return {name, sail::MassConfiguration::orc_stability};
    throw UsageError("--config must be measurement, sailing or orc, got '" + name + "'");
}

LoadedBoat load_boat(const Options& options, sail::MassConfiguration configuration) {
    const auto dir = options.get("--boat");
    if (!dir) throw UsageError("--boat is required");
    if (!std::filesystem::is_directory(*dir)) {
        throw UsageError("--boat '" + *dir + "' is not a directory");
    }
    sail::MassModel model = sail::MassModel::load(*dir, configuration);
    sail::BoatHydrostatics hydro = sail::load_boat_hydrostatics(*dir);
    sail::HydrostaticParams params = hydro.params(model);
    return {std::move(model), std::move(hydro), std::move(params)};
}

double deg(double rad) { return rad * 180.0 / std::numbers::pi; }
double rad(double deg) { return deg * std::numbers::pi / 180.0; }

int guarded(const char* command, const char* usage, int (*body)(const Options&), const Args& args,
            const std::vector<std::string_view>& allowed) {
    try {
        const Options options = Options::parse(args, allowed);
        if (options.help) {
            std::fputs(usage, stdout);
            return 0;
        }
        return body(options);
    } catch (const UsageError& e) {
        std::fprintf(stderr, "sailsim %s: %s\n\n%s", command, e.what(), usage);
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "sailsim %s: %s\n", command, e.what());
        return 1;
    }
}

}  // namespace sailcli
