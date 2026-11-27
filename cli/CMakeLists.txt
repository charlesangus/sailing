add_executable(sailsim main.cpp)
target_link_libraries(sailsim PRIVATE sailcore)
sail_enable_warnings(sailsim)
