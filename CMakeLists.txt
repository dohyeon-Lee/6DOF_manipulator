cmake_minimum_required (VERSION 2.8)
project (actuate)
find_package(Eigen3 REQUIRED)
add_subdirectory(motorlib)

add_executable (actuate_velocity_traj position_velocity.cpp Robotics.cpp)
add_executable (actuate_basic actuate_basic.cpp)
add_executable (actuate_robotics actuate_robotics.cpp Robotics.cpp)

target_link_libraries(actuate_velocity_traj PUBLIC motorlib ${Eigen3_LIBS})
target_link_libraries(actuate_basic PUBLIC motorlib)
target_link_libraries(actuate_robotics PUBLIC motorlib ${Eigen3_LIBS})

target_include_directories(motorlib PUBLIC ${Eigen3_INCLUDE_DIRS})