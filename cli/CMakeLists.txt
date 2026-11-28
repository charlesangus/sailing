add_executable(sailsim main.cpp common.cpp cmd_hydro.cpp cmd_gz.cpp)
target_link_libraries(sailsim PRIVATE sailcore nlohmann_json::nlohmann_json)
sail_enable_warnings(sailsim)
sail_optimise_debug(sailsim)
