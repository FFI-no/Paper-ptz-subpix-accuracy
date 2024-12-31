// Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

#include "mc-analysis/argparse.h"
#include "mc-analysis/triple_cosine_path.h"
#include "mc-analysis/generate_config.h"
#include "mc-analysis/simulation.h"

#include <fstream>

int main(const int argc, const char** argv)
{
  constexpr size_t seed = 73;
  const auto args = mc::readStandardArgs(argc, argv);

  std::ofstream res_out{args.out_path};

  mc::performSimulations(
    seed,
    args.num_threads,
    args.num_iter,
    mc::generateConfig<-2>,
    mc::tripleCosinePath,
    res_out
  );

  return 0;
}
