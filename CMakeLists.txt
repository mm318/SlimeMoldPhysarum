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
    src/SmaOptimizer.cpp
    src/SmaOptimizer.h
)
set(OFX_ADDONS_ACTIVE ofxGui)

include("${ofxcmake_SOURCE_DIR}/modules/main.cmake")

# Runtime directory for saved controls; the app needs no bundled assets.
add_custom_command(TARGET Slime_Mold POST_BUILD
    COMMAND "${CMAKE_COMMAND}" -E make_directory "$<TARGET_FILE_DIR:Slime_Mold>/data"
    VERBATIM)

include(CTest)
if(BUILD_TESTING)
    add_executable(Slime_Mold_sma_tests tests/sma.cpp src/SmaOptimizer.cpp)
    target_include_directories(Slime_Mold_sma_tests PRIVATE src)
    target_compile_features(Slime_Mold_sma_tests PRIVATE cxx_std_17)
    add_test(NAME sma COMMAND Slime_Mold_sma_tests)
endif()
