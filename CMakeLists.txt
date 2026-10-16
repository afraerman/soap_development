cmake_minimum_required(VERSION 3.18)
project(soap LANGUAGES CXX)

# ------------------------------------------------------------------
# Language standards
# ------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Architectures must be known before any CUDA target is created
set(CMAKE_CUDA_ARCHITECTURES "75;86;89" CACHE STRING "" FORCE)

# Force the SRP static library to be built WITHOUT relocatable device code.
# This eliminates the __cudaRegisterLinkedBinary_* linker errors.
set(CMAKE_CUDA_SEPARABLE_COMPILATION OFF CACHE BOOL "" FORCE)

# ------------------------------------------------------------------
# Optional GUI (Qt)
# ------------------------------------------------------------------
option(SOAP_BUILD_GUI "Build GUI version using Qt" OFF)

if(SOAP_BUILD_GUI)
    find_package(Qt6 QUIET COMPONENTS Widgets Concurrent)
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

# ------------------------------------------------------------------
# Main executable
# ------------------------------------------------------------------
if(SOAP_WITH_QT)
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
        srp_manager.cpp
        state_vector.cpp
        torques.cpp
    )
    target_link_libraries(${PROJECT_NAME} PRIVATE Qt6::Widgets Qt6::Concurrent)
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
        srp_manager.cpp
        state_vector.cpp
        torques.cpp
    )
endif()

if(SOAP_WITH_QT)
    target_compile_definitions(${PROJECT_NAME} PRIVATE SOAP_WITH_QT)
endif()

# ------------------------------------------------------------------
# jsoncpp (called only once)
# ------------------------------------------------------------------
if(WIN32)
    find_package(jsoncpp CONFIG REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE JsonCpp::JsonCpp)

    find_path(SOAP_JSON_H_DIR json.h PATH_SUFFIXES json HINTS ${JSONCPP_INCLUDE_DIRS})
    if(NOT SOAP_JSON_H_DIR)
        message(FATAL_ERROR "json.h not found. Install jsoncpp (vcpkg: 'vcpkg install jsoncpp').")
    endif()
    target_include_directories(${PROJECT_NAME} PRIVATE ${SOAP_JSON_H_DIR})
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

# ------------------------------------------------------------------
# SRP library (FetchContent)
# ------------------------------------------------------------------
include(FetchContent)

set(SM3D_ENABLE_CUDA ON CACHE BOOL "" FORCE)
FetchContent_Declare(SRP
    GIT_REPOSITORY https://github.com/paulzap/Real-Time-Solar-Radiation-Pressure-Modeling.git
    GIT_TAG        main
    GIT_SHALLOW    TRUE
    PATCH_COMMAND
        ${CMAKE_COMMAND} -E echo "Patching SRP CMakeLists.txt (disable CUDA separable compilation)"
        COMMAND
        sed -i "s/CUDA_SEPARABLE_COMPILATION ON/CUDA_SEPARABLE_COMPILATION OFF/g"
            <SOURCE_DIR>/CMakeLists.txt
)
FetchContent_MakeAvailable(SRP)

# CUDA settings for the static library + final executable
if(SM3D_HAS_CUDA)
    enable_language(CUDA)

    set_target_properties(srp PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED ON
        CUDA_STANDARD 20
        CUDA_STANDARD_REQUIRED ON
        CUDA_SEPARABLE_COMPILATION OFF
        CUDA_RESOLVE_DEVICE_SYMBOLS OFF
    )

    target_link_libraries(${PROJECT_NAME} PRIVATE CUDA::cudart)
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE srp)

# ------------------------------------------------------------------
# CSPICE / SOFA
# ------------------------------------------------------------------
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

set(SOAP_SOFA_DIR   "${CMAKE_CURRENT_SOURCE_DIR}/libs/sofa")
set(SOAP_CSPICE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libs/cspice/include")

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SOAP_SOFA_DIR}
    ${SOAP_CSPICE_DIR}
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    SOAP_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Files"
    SOAP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /permissive- /bigobj)
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE m)
endif()

# ------------------------------------------------------------------
# Windows Qt deployment
# ------------------------------------------------------------------
if(WIN32 AND SOAP_WITH_QT)
    get_target_property(QT_QMAKE_EXECUTABLE Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(QT_BIN_DIR "${QT_QMAKE_EXECUTABLE}" DIRECTORY)

    set(DEPLOY_ARGS)
    if(CMAKE_BUILD_TYPE STREQUAL "Debug" OR
       (DEFINED CMAKE_CONFIGURATION_TYPES AND "$<CONFIG>" STREQUAL "Debug"))
        list(APPEND DEPLOY_ARGS --debug)
    else()
        list(APPEND DEPLOY_ARGS --release)
    endif()

    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND "${QT_BIN_DIR}/windeployqt.exe"
                ${DEPLOY_ARGS}
                "$<TARGET_FILE:${PROJECT_NAME}>"
        COMMENT "Running windeployqt for $<CONFIG>..."
    )

    set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE ON)
endif()

# ------------------------------------------------------------------
# Unit tests (optional)
# ------------------------------------------------------------------
option(SOAP_BUILD_TESTS "Build unit tests" ON)

if(SOAP_BUILD_TESTS)
    message(STATUS "Building tests")

    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.16.0
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(soap_tests
        tests/date_time_tests.cpp
        tests/matrix_tests.cpp
        tests/position_vector_tests.cpp
        tests/quaternion_tests.cpp
        tests/srp_manager_tests.cpp

        # shared sources needed by the tests
        astrometry.cpp
        attitude_controller.cpp
        control.cpp
        date_time.cpp
        filenames.cpp
        matrix.cpp
        polygon.cpp
        position_vector.cpp
        quaternion.cpp
        reaction_wheel.cpp
        satellite.cpp
        srp_manager.cpp
        state_vector.cpp
    )

    target_link_libraries(soap_tests PRIVATE
        Catch2::Catch2WithMain
        srp
        ${SOAP_SOFA}
        ${SOAP_CSPICE}
    )

    # jsoncpp for tests (reuse the already-found package)
    if(TARGET JsonCpp::JsonCpp)
        target_link_libraries(soap_tests PRIVATE JsonCpp::JsonCpp)
    elseif(JSONCPP_LIBRARIES)
        target_link_libraries(soap_tests PRIVATE ${JSONCPP_LIBRARIES})
        target_include_directories(soap_tests PRIVATE ${JSONCPP_INCLUDE_DIRS})
    endif()

    if(WIN32 AND SOAP_JSON_H_DIR)
        target_include_directories(soap_tests PRIVATE ${SOAP_JSON_H_DIR})
    endif()

    if(SM3D_HAS_CUDA)
        target_link_libraries(soap_tests PRIVATE CUDA::cudart)
    endif()

    target_include_directories(soap_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${SOAP_SOFA_DIR}
        ${SOAP_CSPICE_DIR}
    )

    target_compile_definitions(soap_tests PRIVATE
        SOAP_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/Files"
        SOAP_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    include(CTest)
    include(Catch)
    catch_discover_tests(soap_tests)
endif()