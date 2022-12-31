// Copyright (c) 2022 Norwegian Defence Research Establishment (FFI)

#pragma once

#include <cstddef>
#include <string>

namespace mc
{
struct StandardArgs
{
  size_t num_threads;
  size_t num_iter;
  std::string out_path;
};

StandardArgs readStandardArgs(
  int argc,
  const char** argv
);
}
