# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

cmake_minimum_required(VERSION 3.23)
project(ptz-paper-mc-analysis LANGUAGES CXX VERSION 1.0.0)

include(cmake/compiler-flags.cmake)
include(cmake/coverage-flags.cmake)

find_package(OpenCV REQUIRED)
find_package(ptcee REQUIRED)
find_package(Sophus REQUIRED)
find_package(Threads REQUIRED)

add_library(mc_analysis)

target_link_libraries(mc_analysis
  PUBLIC opencv::opencv
  PUBLIC ptcee::ptcee
  PUBLIC Sophus::Sophus
  PUBLIC Threads::Threads
)

file(GLOB_RECURSE HEADER_FILES include/*.h)
file(GLOB_RECURSE SRC_FILES src/*.cpp)

target_sources(mc_analysis
  PUBLIC FILE_SET public_headers TYPE HEADERS BASE_DIRS include FILES ${HEADER_FILES}
  PRIVATE ${SRC_FILES}
)

set_target_properties(mc_analysis PROPERTIES
  PUBLIC_HEADER "${HEADER_FILES}"
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 20
)

add_compiler_flags(mc_analysis)
add_coverage_flags(mc_analysis)

add_executable(base_ptcee_mc
  executables/base_ptcee_mc.cpp
)

set_target_properties(base_ptcee_mc PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CXX
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 20
)

target_link_libraries(base_ptcee_mc
  PUBLIC mc_analysis
)

add_executable(soft_ptcee_mc
  executables/soft_ptcee_mc.cpp
)

set_target_properties(soft_ptcee_mc PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CXX
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 20
)

target_link_libraries(soft_ptcee_mc
  PUBLIC mc_analysis
)

set_target_properties(soft_ptcee_mc PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CXX
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 20
)

add_executable(multi_level_ba_mc
  executables/multi_level_ba_mc.cpp
)

target_link_libraries(multi_level_ba_mc
  PUBLIC mc_analysis
)
