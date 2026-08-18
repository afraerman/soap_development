cmake_minimum_required(VERSION 3.10)
project(soap)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(${PROJECT_NAME}
astrometry.cpp
attitude_controller.cpp
attitude_integrator.cpp
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


if(WIN32)
    find_package(jsoncpp CONFIG REQUIRED)

    target_link_libraries(${PROJECT_NAME}
        PRIVATE
        JsonCpp::JsonCpp
    )
else()
    find_package(jsoncpp CONFIG QUIET)
    if(jsoncpp_FOUND)
        target_link_libraries(${PROJECT_NAME} PRIVATE JsonCpp::JsonCpp)
    else()
        find_package(PkgConfig REQUIRED)
        pkg_check_modules(JSONCPP REQUIRED jsoncpp)
        target_link_libraries(${PROJECT_NAME} PRIVATE ${JSONCPP_LIBRARIES})
        target_include_directories(${PROJECT_NAME} PRIVATE ${JSONCPP_INCLUDE_DIRS})
    endif()
endif()

find_path(SOAP_JSON_H_DIR json.h PATH_SUFFIXES json HINTS ${JSONCPP_INCLUDE_DIRS})
if(NOT SOAP_JSON_H_DIR)
    message(FATAL_ERROR "json.h not found. Install jsoncpp (vcpkg: 'vcpkg install jsoncpp').")
endif()
target_include_directories(${PROJECT_NAME} PRIVATE ${SOAP_JSON_H_DIR})

include(FetchContent)
FetchContent_Declare(SRP
    GIT_REPOSITORY https://github.com/paulzap/Real-Time-Solar-Radiation-Pressure-Modeling.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE)

FetchContent_MakeAvailable(SRP)


target_link_libraries(${PROJECT_NAME}
    PRIVATE
    srp
)

if(WIN32)
    set(SOAP_CSPICE "${CMAKE_CURRENT_SOURCE_DIR}/libs/cspice.lib")
    set(SOAP_SOFA   "${CMAKE_CURRENT_SOURCE_DIR}/libs/sofa_lib.lib")
else()
    set(SOAP_CSPICE "${CMAKE_CURRENT_SOURCE_DIR}/libs/cspice.a")
    set(SOAP_SOFA   "${CMAKE_CURRENT_SOURCE_DIR}/libs/libsofa_c.a")
endif()

foreach(_lib ${SOAP_CSPICE} ${SOAP_SOFA})
    if(NOT EXISTS ${_lib})
        message(FATAL_ERROR "Required library not found: ${_lib}")
    endif()
endforeach()

target_link_libraries(${PROJECT_NAME} PRIVATE ${SOAP_CSPICE} ${SOAP_SOFA})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    SOAP_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Files"
)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /permissive- /bigobj)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE m)
endif()
