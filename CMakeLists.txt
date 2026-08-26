cmake_minimum_required(VERSION 3.10)
project(soap)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SOAP_BUILD_GUI "Build BUI version using Qt" OFF)

if(SOAP_BUILD_GUI)
    #set(CMAKE_PREFIX_PATH "D:/Qt/6.11.2/msvc2022_64")
    find_package(Qt6 QUIET COMPONENTS Widgets)

    if(Qt6_FOUND)
        message(STATUS "Building GUI version")
        set(SOAP_WITH_QT ON)
    else()
        message(WARNING "Qt6 not found - falling back to console")
        set(SOAP_WITH_QT OFF)
    endif()
else()
    message(STATUS "GUI disabled - building console version")
    set(SOAP_WITH_QT OFF)
endif()

if(Qt6_FOUND)
    if(WIN32)    
        set(CMAKE_AUTOMOC ON)
        set(CMAKE_AUTORCC ON)
        set(CMAKE_AUTOUIC ON)
    endif()

    qt_standard_project_setup()

    qt_add_executable(${PROJECT_NAME}
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
        map_dialog.cpp
        matrix.cpp
        orbit_integrator.cpp
        polygon.cpp
        position_vector.cpp
        qt_stream_redirector.h
        qt_main_window.cpp
        quaternion.cpp
        reaction_wheel.cpp
        satellite.cpp
        state_vector.cpp
        tests.cpp
        torques.cpp
    )

    target_link_libraries(soap PRIVATE Qt6::Widgets)

else()
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
endif()

if(SOAP_WITH_QT)
    target_compile_definitions(soap PRIVATE SOAP_WITH_QT)
endif()

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

if(WIN32)
    find_path(SOAP_JSON_H_DIR json.h PATH_SUFFIXES json HINTS ${JSONCPP_INCLUDE_DIRS})
    if(NOT SOAP_JSON_H_DIR)
        message(FATAL_ERROR "json.h not found. Install jsoncpp (vcpkg: 'vcpkg install jsoncpp').")
    endif()
    target_include_directories(${PROJECT_NAME} PRIVATE ${SOAP_JSON_H_DIR})
endif()

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
    SOAP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /permissive- /bigobj)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE m)
endif()

# -------------------------------------------------
# Automatic deployment of Qt DLLs (Windows only)
# -------------------------------------------------
if(WIN32 AND SOAP_WITH_QT)
    # Find the windeployqt tool that belongs to the Qt we just found
    get_target_property(QT_QMAKE_EXECUTABLE Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(QT_BIN_DIR "${QT_QMAKE_EXECUTABLE}" DIRECTORY)

    # Choose --debug or --release automatically
    set(DEPLOY_ARGS)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR (DEFINED CMAKE_CONFIGURATION_TYPES AND "$<CONFIG>" STREQUAL "Debug"))
        list(APPEND DEPLOY_ARGS --debug)
    else()
        list(APPEND DEPLOY_ARGS --release)
        list(APPEND DEPLOY_ARGS --debug)
    endif()

    # Run windeployqt after the executable is built
    add_custom_command(TARGET soap POST_BUILD
        COMMAND "${QT_BIN_DIR}/windeployqt.exe"
                ${DEPLOY_ARGS}
                "$<TARGET_FILE:soap>"
        COMMENT "Running windeployqt for $<CONFIG>..."
    )
endif()

if(WIN32 AND SOAP_WITH_QT)
    set_target_properties(soap PROPERTIES
        WIN32_EXECUTABLE ON
    )
endif()
