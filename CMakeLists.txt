cmake_minimum_required(VERSION 3.16)
project(navcore LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(NAVCORE_BUILD_TESTS "Build the NavCore unit test suite" ON)

# Eigen 3.4 (third_party/eigen, git submodule) is header-only — no build/link step,
# just an include path. Not using add_subdirectory(third_party/eigen): that would pull
# in Eigen's own CMake project (its test suite, install rules, CMake package config),
# none of which this repo needs.
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen)

add_library(navcore STATIC
  src/align/hacf_aligner.cpp
)
target_include_directories(navcore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(navcore PUBLIC eigen)
target_compile_options(navcore PRIVATE -Wall -Wextra)

add_executable(calib_cli tools/calib_cli.cpp)
target_link_libraries(calib_cli PRIVATE navcore)

if(NAVCORE_BUILD_TESTS)
  enable_testing()

  include(FetchContent)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
  )
  FetchContent_MakeAvailable(googletest)

  add_executable(navcore_unit_tests
    tests/unit/hacf_aligner_test.cpp
  )
  target_link_libraries(navcore_unit_tests PRIVATE navcore gtest_main)

  include(GoogleTest)
  gtest_discover_tests(navcore_unit_tests)
endif()
