# compiler-flags.cmake
# version 1.0.0

function(add_compiler_flags target_name)
  set(gcc_compiler_flags
    -Wall
    -Wcast-align
    -Wcast-qual
    -Werror
    -Wextra
    -Wfloat-conversion
    -Winit-self
    -Winit-self
    -Wlogical-op
    -Wmissing-declarations
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
    -Wpedantic
    -Wpointer-arith
    -Wshadow
    -Wsuggest-override
    -Wuninitialized
    -Wunknown-pragmas
    -Wunreachable-code
    -Wunused-local-typedefs
    -Wno-unused-function
  )

  set(msvc_compiler_flags
    /W4
    /WX
  )

  set(msvc_cxx        "$<COMPILE_LANG_AND_ID:CXX,MSVC>")
  set(gcc_like_cxx    "$<COMPILE_LANG_AND_ID:CXX,GNU>")

  target_compile_options(${target_name} PRIVATE
    "$<${gcc_like_cxx}:$<BUILD_INTERFACE:${gcc_compiler_flags}>>"
    "$<${msvc_cxx}:$<BUILD_INTERFACE:${msvc_compiler_flags}>>"
  )
endfunction()
