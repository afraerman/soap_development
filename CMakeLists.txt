cmake_minimum_required(VERSION 3.10)
project(soap)

add_executable(${PROJECT_NAME}
astrometry.cpp
attitude_controller.cpp
attitude_integrator.cpp
celestial_bodies.cpp
control.cpp
date_time.cpp
filenames.cpp
forces.cpp
input.cpp
integrator.cpp
main.cpp
matrix.cpp
orbit_integrator.cpp
polygon.cpp
position_vector.cpp
reaction_wheel.cpp
satellite.cpp
state_vector.cpp
tests.cpp
torques.cpp
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

target_link_libraries(${PROJECT_NAME}
${CMAKE_SOURCE_DIR}/cspice.a
${CMAKE_SOURCE_DIR}/libsofa_c.a
${JSONCPP_LIBRARIES})

target_include_directories(${PROJECT_NAME}
PRIVATE ${CMAKE_SOURCE_DIR}/
PRIVATE /media/alexey/Disk/Astronomy/boost_1_84_0/)