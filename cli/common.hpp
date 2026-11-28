#pragma once

#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "sail/boat_sim.hpp"
#include "sail/equilibrium.hpp"
#include "sail/hull_resistance.hpp"
#include "sail/mass_model.hpp"

namespace sailcli {

using Args = std::vector<std::string_view>;

// A command-line mistake: reported with the command's usage and exit status 2.
class UsageError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// Parses `--name value` and `--name=value` pairs. Every name must be in `allowed`; `--help` sets
// help instead.
struct Options {
    std::map<std::string, std::string, std::less<>> values;
    bool help = false;

    static Options parse(const Args& args, const std::vector<std::string_view>& allowed);
    std::optional<std::string> get(std::string_view name) const;
    double number(std::string_view name, double fallback) const;
};

struct ConfigChoice {
    std::string name;
    sail::MassConfiguration configuration;
};

// "measurement", "sailing" or "orc"; absent means "measurement".
ConfigChoice parse_config(const std::optional<std::string>& value);

// The boat as the hydrostatics tools float it.
struct LoadedBoat {
    sail::MassModel model;
    sail::BoatHydrostatics hydro;
    sail::HydrostaticParams params;
};

// Throws UsageError when --boat is missing or not a directory.
LoadedBoat load_boat(const Options& options, sail::MassConfiguration configuration);

// m/s per knot.
inline constexpr double kKnot = 0.514444;

// "orc" or "dsyhs"; absent means the dossier's choice.
std::optional<sail::HullResistanceKind> parse_resistance_model(const std::optional<std::string>& value);
// The name parse_resistance_model() reads for `kind`.
const char* resistance_model_name(sail::HullResistanceKind kind);

// The full force model of --boat with the keel and rudder section from --section, by default
// foils/naca0012.csv two directories above the boat dossier. Throws UsageError when --boat is
// missing or not a directory, or the section file does not exist.
std::unique_ptr<sail::BoatSim> load_boat_sim(const Options& options,
                                             const sail::BoatSimOptions& sim_options);

double deg(double rad);
double rad(double deg);

// Runs `body` and maps exceptions to exit codes: UsageError to 2 (after `usage`), others to 1.
int guarded(const char* command, const char* usage, int (*body)(const Options&), const Args& args,
            const std::vector<std::string_view>& allowed);

int run_hydro(const Args& args);
int run_gz(const Args& args);
int run_bench(const Args& args);
int run_tow(const Args& args);
int run_turn(const Args& args);

}  // namespace sailcli
