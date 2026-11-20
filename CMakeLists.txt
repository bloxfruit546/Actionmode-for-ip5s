cmake_minimum_required(VERSION 3.16)
project(GyroStabilizer5sCore LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(gf_core STATIC
    CoreCPP/src/Quaternion.cpp
    CoreCPP/src/SidecarIO.cpp
    CoreCPP/src/CalibrationProfile.cpp
    CoreCPP/src/TrajectoryEngine.cpp
    CoreCPP/src/FrameWarpCalculator.cpp
    CoreCPP/src/OpticalSync.cpp
    CoreCPP/src/TranslationalStabilizer2D.cpp
    CoreCPP/src/VirtualGyroEngine.cpp
)

target_include_directories(gf_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/CoreCPP/include
)

if(MSVC)
    target_compile_options(gf_core PRIVATE /W4)
else()
    target_compile_options(gf_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(gf_core_tests
    CoreCPP/tests/test_math.cpp
)
target_link_libraries(gf_core_tests PRIVATE gf_core)

add_executable(gf_sidecar_calib_tests
    CoreCPP/tests/test_sidecar_and_calib.cpp
)
target_link_libraries(gf_sidecar_calib_tests PRIVATE gf_core)

add_executable(gf_stabilizer_tests
    CoreCPP/tests/test_stabilizer.cpp
)
target_link_libraries(gf_stabilizer_tests PRIVATE gf_core)

add_executable(gf_horizon_rs_autosync_tests
    CoreCPP/tests/test_horizon_rs_autosync.cpp
)
target_link_libraries(gf_horizon_rs_autosync_tests PRIVATE gf_core)

add_executable(gf_quality_metrics_tests
    CoreCPP/tests/test_quality_metrics.cpp
)
target_link_libraries(gf_quality_metrics_tests PRIVATE gf_core)

enable_testing()
add_test(NAME gf_core_math_tests COMMAND gf_core_tests)
add_test(NAME gf_sidecar_calib_tests COMMAND gf_sidecar_calib_tests)
add_test(NAME gf_stabilizer_tests COMMAND gf_stabilizer_tests)
add_test(NAME gf_horizon_rs_autosync_tests COMMAND gf_horizon_rs_autosync_tests)
add_test(NAME gf_quality_metrics_tests COMMAND gf_quality_metrics_tests)
