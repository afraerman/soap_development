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
quaternion.cpp
reaction_wheel.cpp
satellite.cpp
state_vector.cpp
tests.cpp
torques.cpp
)

find_package(jsoncpp CONFIG QUIET)
if(jsoncpp_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE JsonCpp::JsonCpp)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP REQUIRED jsoncpp)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${JSONCPP_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${JSONCPP_INCLUDE_DIRS})
endif()

find_path(SOAP_JSON_H_DIR json.h PATH_SUFFIXES json HINTS ${JSONCPP_INCLUDE_DIRS})
if(NOT SOAP_JSON_H_DIR)
    message(FATAL_ERROR "json.h not found. Install jsoncpp (vcpkg: 'vcpkg install jsoncpp').")
endif()
target_include_directories(${PROJECT_NAME} PRIVATE ${SOAP_JSON_H_DIR})

if(WIN32)
    set(SOAP_CSPICE "${CMAKE_CURRENT_SOURCE_DIR}/cspice.lib")
    set(SOAP_SOFA   "${CMAKE_CURRENT_SOURCE_DIR}/sofa_lib.lib")
else()
    set(SOAP_CSPICE "${CMAKE_CURRENT_SOURCE_DIR}/cspice.a")
    set(SOAP_SOFA   "${CMAKE_CURRENT_SOURCE_DIR}/libsofa_c.a")
endif()

foreach(_lib ${SOAP_CSPICE} ${SOAP_SOFA})
    if(NOT EXISTS ${_lib})
        message(FATAL_ERROR "Required library not found: ${_lib}")
    endif()
endforeach()

target_link_libraries(${PROJECT_NAME} PRIVATE ${SOAP_CSPICE} ${SOAP_SOFA})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /permissive- /bigobj)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE m)
endif()
