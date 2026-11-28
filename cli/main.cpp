#include <cstdio>
#include <functional>
#include <string_view>
#include <vector>

#include "common.hpp"
#include "sail/version.hpp"

namespace {

using sailcli::Args;

struct Command {
    std::string_view name;
    std::string_view summary;
    std::function<int(const Args&)> run;
};

int print_version(const Args&) {
    std::printf("%.*s\n", static_cast<int>(sail::version().size()), sail::version().data());
    return 0;
}

const std::vector<Command>& commands();

int print_help(const Args&) {
    std::puts("usage: sailsim <command> [args]\n\ncommands:");
    for (const Command& c : commands()) {
        std::printf("  %-12.*s %.*s\n", static_cast<int>(c.name.size()), c.name.data(),
                    static_cast<int>(c.summary.size()), c.summary.data());
    }
    return 0;
}

const std::vector<Command>& commands() {
    static const std::vector<Command> table = {
        {"--version", "print the version", print_version},
        {"--help", "show this help", print_help},
        {"hydro", "upright flat-water equilibrium as JSON", sailcli::run_hydro},
        {"gz", "righting-moment and GZ curve over 0-180 deg heel as JSON", sailcli::run_gz},
    };
    return table;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        print_help({});
        return 2;
    }
    const std::string_view name = argv[1];
    const Args rest(argv + 2, argv + argc);
    for (const Command& c : commands()) {
        if (c.name == name) {
            return c.run(rest);
        }
    }
    std::fprintf(stderr, "sailsim: unknown command '%s'\n", argv[1]);
    return 1;
}
