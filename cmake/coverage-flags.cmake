# coverage-flags.cmake
# version 1.0.0

macro(add_coverage_option)
  if(NOT coverage_enabled_already)
    set(coverage_enabled_already TRUE)

    option(WITH_COVERAGE OFF "Whether to build with coverage")
    message(STATUS "* WITH_COVERAGE is ${WITH_COVERAGE}")

    set(gcc_like_cxx    "$<CXX_COMPILER_ID:GNU>")
    set(config_coverage "$<CONFIG:Coverage>")
    set(with_coverage   "$<BOOL:${WITH_COVERAGE}>")
    set(add_coverage    "$<AND:${gcc_like_cxx},$<OR:${config_coverage},${with_coverage}>>")
  endif()
endmacro()

function(add_coverage_flags target_name)
  add_coverage_option()
  target_compile_options(${target_name} PUBLIC "$<${add_coverage}:--coverage;-g;-O0>")
  target_link_options(   ${target_name} PUBLIC "$<${add_coverage}:--coverage>")
endfunction()
