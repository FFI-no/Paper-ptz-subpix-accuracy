# Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

cmake_minimum_required(VERSION 3.16)
project(ptz-paper-mc-analysis VERSION 1.0.0)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR})

find_package(ptcee REQUIRED)
find_package(Sophus REQUIRED)
find_package(Threads REQUIRED)

file(GLOB_RECURSE SRC_FILES src/*.cpp)
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(mc_analysis
  ${HEADER_FILES}
  ${SRC_FILES}
  )

target_include_directories(mc_analysis PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

set_target_properties(mc_analysis PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CXX
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 17
  BUILD_RPATH $ORIGIN
  INSTALL_RPATH $ORIGIN
  )

target_link_libraries(mc_analysis
  PUBLIC ptcee::ptcee
  PUBLIC Sophus::Sophus
  PUBLIC Threads::Threads
  )

add_executable(hard_beta_mc
  executables/hard_beta_mc.cpp
  )

set_target_properties(hard_beta_mc PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CXX
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 17
  BUILD_RPATH $ORIGIN
  INSTALL_RPATH $ORIGIN
  )

target_link_libraries(hard_beta_mc
  PUBLIC mc_analysis
  )

add_executable(soft_beta_mc
  executables/soft_beta_mc.cpp
  )

set_target_properties(soft_beta_mc PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CXX
  CXX_STANDARD_REQUIRED ON
  CXX_STANDARD 17
  BUILD_RPATH $ORIGIN
  INSTALL_RPATH $ORIGIN
  )

target_link_libraries(soft_beta_mc
  PUBLIC mc_analysis
  )
