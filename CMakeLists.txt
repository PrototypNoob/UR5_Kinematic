cmake_minimum_required (VERSION 3.0)
project (UR5_KINEMATIC)
 
find_package (Eigen3 3.4 REQUIRED NO_MODULE)

add_library(kinematic 
    include/UR5Kinematic.cpp
)

add_executable (forward_test
    example/forward_test.cpp
)

add_executable (inverse_test
    example/inverse_test.cpp
)

target_link_libraries (forward_test kinematic Eigen3::Eigen)
target_link_libraries (inverse_test kinematic Eigen3::Eigen)