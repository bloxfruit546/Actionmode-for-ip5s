cmake_minimum_required(VERSION 3.16)
project(GyroStabilizer5sCore LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(gf_core STATIC
    CoreCPP/src/Quaternion.cpp
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

enable_testing()
add_test(NAME gf_core_math_tests COMMAND gf_core_tests)
