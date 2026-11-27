#include "sail/version.hpp"

#include "version_config.hpp"

namespace sail {

std::string_view version() { return SAIL_VERSION_STRING; }

}  // namespace sail
