cmake_minimum_required(VERSION 3.16)
project(Slime_Mold LANGUAGES C CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "Use an out-of-source build: cmake -S . -B build")
endif()

# Use the already-extracted official release; do not download or rebuild its
# dependencies in the openFrameworks installation.
set(OF_DIRECTORY_BY_USER
    "${CMAKE_CURRENT_SOURCE_DIR}/../of_v0.12.1_linux64_gcc6_release"
    CACHE PATH "Path to the extracted openFrameworks 0.12.1 Linux release")

include(FetchContent)
FetchContent_Declare(ofxcmake
    GIT_REPOSITORY https://github.com/mm318/ofxCMake.git
    GIT_TAG 7a92d86054eb41a75584b2ddedcc0c72a3c2f325
)
# For an offline build, set FETCHCONTENT_SOURCE_DIR_OFXCMAKE to a local checkout.
FetchContent_MakeAvailable(ofxcmake)

set(APP_NAME Slime_Mold)
set(${APP_NAME}_SOURCE_FILES
    src/main.cpp
    src/ofApp.cpp
    src/ofApp.h
)
set(OFX_ADDONS_ACTIVE ofxGui)

# ofxCMake builds the OF core and ofxGui, links the app, and copies bin/data
# beside the executable in the build directory.
include("${ofxcmake_SOURCE_DIR}/modules/main.cmake")
