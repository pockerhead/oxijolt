# Native part of oxijolt-sys: builds joltc and Jolt as static libraries from the
# vendored submodules together with this fork's additions to joltc (joltc_ext/),
# compiles the C++ layout checks and installs a minimal prefix (lib/ with both
# archives, include/joltc.h and include/joltc_ext.h) for build.rs to link.

# Jolt 5.6's Build/CMakeLists.txt requires 3.20.
cmake_minimum_required(VERSION 3.20)
project(oxijolt_sys_native C CXX)

# joltc and Jolt set the C++ standard only in their own directory scope; the
# layout checks in this scope need C++17 as well.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

foreach(required_dir JOLTC_SOURCE_DIR JOLT_PHYSICS_ROOT)
    if (NOT DEFINED ${required_dir})
        message(FATAL_ERROR "${required_dir} is not set. build.rs passes it; run `git submodule update --init` if the sources are missing.")
    endif()
endforeach()
if (NOT EXISTS "${JOLTC_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "joltc not found at ${JOLTC_SOURCE_DIR}. Run `git submodule update --init`.")
endif()
if (NOT EXISTS "${JOLT_PHYSICS_ROOT}/Build/CMakeLists.txt")
    message(FATAL_ERROR "Jolt Physics not found at ${JOLT_PHYSICS_ROOT}. Run `git submodule update --init`.")
endif()

# As a subproject joltc builds a static library without samples, tests or
# install rules, and takes Jolt from JOLT_PHYSICS_ROOT instead of downloading it.
add_subdirectory(${JOLTC_SOURCE_DIR} joltc)

if (DOUBLE_PRECISION)
    set(JOLTC_TARGET joltc_double)
else()
    set(JOLTC_TARGET joltc)
endif()

# joltc_ext/joltc_ext.cpp reaches the JPH::PhysicsSystem behind a JPH_PhysicsSystem handle through
# joltc's private wrapper struct. Its definition is copied here, token for token, from the joltc.cpp
# being built, so both translation units define one type; the header is never installed.
set(joltc_cpp "${JOLTC_SOURCE_DIR}/src/joltc.cpp")
set(joltc_physics_system_marker "struct JPH_PhysicsSystem final")
set(joltc_physics_system_header "${CMAKE_CURRENT_BINARY_DIR}/joltc_ext_generated/joltc_physics_system.h")
file(READ "${joltc_cpp}" joltc_source)
string(FIND "${joltc_source}" "${joltc_physics_system_marker}" joltc_physics_system_first)
string(FIND "${joltc_source}" "${joltc_physics_system_marker}" joltc_physics_system_last REVERSE)
if (joltc_physics_system_first EQUAL -1 OR NOT joltc_physics_system_first EQUAL joltc_physics_system_last)
    message(FATAL_ERROR "${joltc_cpp} must define `${joltc_physics_system_marker}` exactly once: joltc_ext/joltc_ext.cpp uses it through the generated ${joltc_physics_system_header}.")
endif()
string(REGEX MATCH "struct JPH_PhysicsSystem final[ \t\r\n]*{[^}]*};" joltc_physics_system "${joltc_source}")
if (joltc_physics_system STREQUAL "")
    message(FATAL_ERROR "Could not extract `${joltc_physics_system_marker}` from ${joltc_cpp} for joltc_ext/joltc_ext.cpp (generated ${joltc_physics_system_header}).")
endif()
# file(CONFIGURE) rewrites the header only when its content changes.
file(CONFIGURE OUTPUT "${joltc_physics_system_header}" CONTENT "#pragma once

// Generated from joltc.cpp at configure time; joltc's private wrapper of JPH::PhysicsSystem, the
// same tokens as joltc's definition so both translation units define one type.

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>

@joltc_physics_system@
" @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${joltc_cpp}")

# Functions this fork adds to joltc in joltc's naming; compiled into the joltc archive so they
# share its compile definitions (precision, object layer bits, floating-point flags).
target_sources(${JOLTC_TARGET} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_body.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_job_system.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_listener.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_material.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_shape.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_shape_state.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_skeleton.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_soft_body_skin.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext_vehicle.cpp)
target_include_directories(${JOLTC_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext ${CMAKE_CURRENT_BINARY_DIR}/joltc_ext_generated)

# Compile-time checks that joltc's C mirrors match Jolt's C++ types. Linking the
# targets brings in Jolt's public definitions (precision, object layer bits,
# instruction set) and joltc's include directory. Part of ALL, so a failing
# static_assert fails the build.
add_library(joltc_layout_checks OBJECT layout_checks.cpp)
target_link_libraries(joltc_layout_checks PRIVATE ${JOLTC_TARGET} Jolt)

install(TARGETS ${JOLTC_TARGET} Jolt ARCHIVE DESTINATION lib)
install(FILES ${JOLTC_SOURCE_DIR}/include/joltc.h ${CMAKE_CURRENT_SOURCE_DIR}/joltc_ext/joltc_ext.h DESTINATION include)
