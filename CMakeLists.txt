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
    src/SlimeSimulation.cpp
    src/SlimeSimulation.h
)
set(OFX_ADDONS_ACTIVE ofxGui)

# ofxCMake builds the OF core and ofxGui, links the app, and copies bin/data
# beside the executable in the build directory.
include("${ofxcmake_SOURCE_DIR}/modules/main.cmake")

# Replace the generated shader directory so edits and deletions reach the app
# even without relinking. Other runtime files, including saved controls, stay.
add_custom_target(Slime_Mold_data ALL
    COMMAND "${CMAKE_COMMAND}" -E remove_directory
        "$<TARGET_FILE_DIR:Slime_Mold>/data/shadersGL3"
    COMMAND "${CMAKE_COMMAND}" -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/bin/data" "$<TARGET_FILE_DIR:Slime_Mold>/data"
    DEPENDS Slime_Mold
    VERBATIM)

include(CTest)
if(BUILD_TESTING)
    add_executable(Slime_Mold_tests tests/simulation.cpp src/SlimeSimulation.cpp)
    target_include_directories(Slime_Mold_tests PRIVATE src)
    target_link_libraries(Slime_Mold_tests PRIVATE of_static)
    set_target_properties(Slime_Mold_tests PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/bin")
    add_dependencies(Slime_Mold_tests Slime_Mold_data)
    find_program(XVFB_RUN xvfb-run)
    if(XVFB_RUN)
        add_test(NAME simulation COMMAND "${XVFB_RUN}" -a "$<TARGET_FILE:Slime_Mold_tests>")
        set_tests_properties(simulation PROPERTIES ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1")
    else()
        add_test(NAME simulation COMMAND Slime_Mold_tests)
    endif()
    set_tests_properties(simulation PROPERTIES TIMEOUT 180)
endif()
