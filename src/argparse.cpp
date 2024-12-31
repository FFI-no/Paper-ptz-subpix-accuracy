// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/argparse.h"

#include "sstream"
#include "stdexcept"

namespace mc
{
StandardArgs readStandardArgs(
  const int argc,
  const char** argv
)
{
  if (argc < 4)
  {
    std::ostringstream ss;
    ss << "USAGE: " << argv[0] << " <num threads> <num iter> <path to output file>";

    throw std::runtime_error{ss.str()};
  }

  return {
    std::stoull(argv[1]),
    std::stoull(argv[2]),
    argv[3]
  };
}
}
