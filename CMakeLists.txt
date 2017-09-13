cmake_minimum_required(VERSION 3.5)
project(liblinkbot VERSION 2.0.0 LANGUAGES C CXX)

set(preRelease "")
# Examples: dev, rc.0, rc.1, some-feature-branch-name
#
# Pre-release data must be a '.'-delimited list of zero or more identifiers composed of one or more
# [0-9A-Za-z-] characters. When linkbotd sorts versions which differ only in their pre-release
# data, the list of pre-release identifiers is sorted lexicographically. Identifiers composed of
# all digits are interpreted as numbers for comparison purposes. Numbers always compare less than
# alphanumeric identifiers (e.g., "a" < "b" and "2" < "10", but "10" < "2a").
#
# Given two version strings which differ only in the presence of pre-release data, the version
# string WITH pre-release data (i.e., `if(preRelease)` evalutes to true) compares less than the
# version string WITHOUT pre-release data. This is because the lack of pre-release data indicates
# an actual release.

set(PROJECT_VERSION_STRING "${PROJECT_VERSION}")
if(preRelease)
    set(PROJECT_VERSION_STRING "${PROJECT_VERSION_STRING}-${preRelease}")
endif()

include(ExternalProject)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(linkbotrs_build_command cargo build --release)
    set(linkbotrs_install_command cmake -E copy <SOURCE_DIR>/target/release/liblinkbotrs.a <INSTALL_DIR>/lib/liblinkbotrs.a
        COMMAND cmake -E copy_directory <SOURCE_DIR>/include <INSTALL_DIR>)
    set(linkbotrs_lib_dir ${PROJECT_SOURCE_DIR}/liblinkbotrs/target/release/)
else()
    set(linkbotrs_build_command cargo build)
    set(linkbotrs_install_command echo "no install required")
    set(linkbotrs_lib_dir ${PROJECT_SOURCE_DIR}/liblinkbotrs/target/debug/)
endif()

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

# Set the library dependencies
if(UNIX)
    set(linkbotrs_library_deps 
        ssl
        crypto
        util
        dl
        rt
        pthread
        gcc_s
        c
        m
        rt
        pthread
        util
        )
elseif(APPLE)
endif()

ExternalProject_Add(linkbotrs
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/liblinkbotrs
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ${linkbotrs_build_command}
    INSTALL_COMMAND ${linkbotrs_install_command}
    BUILD_IN_SOURCE 1
    BUILD_ALWAYS OFF
    )

find_library(LINKBOTRS linkbotrs 
    PATHS ${linkbotrs_lib_dir}
    )

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/liblinkbotrs/include
    )

add_library(linkbot SHARED 
    src/linkbot.cpp
    src/clinkbot.cpp
    src/clinkboti.cpp
    src/clinkbotigroup.cpp
    src/clinkbotl.cpp
    src/clinkbotlgroup.cpp
    src/rgbhashtable.c.cpp
    )

target_link_libraries(linkbot 
    ${LINKBOTRS}
    ${linkbotrs_library_deps}
    )

add_dependencies(linkbot linkbotrs)

add_executable(linkbot_test
    tests/test.cpp
    )
add_dependencies(linkbot_test linkbot)
target_link_libraries(linkbot_test linkbot)
