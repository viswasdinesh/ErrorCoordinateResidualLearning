// Generates a deterministic fixture of 8-D ambient-adapter inputs -> 9-D
// residual predictions computed by the C++ ommpc::residual::ambientForward
// (include/ommpc_ros2/residual_model_ambient.h), for
// test/test_residual_mlp_parity.py to replay through the original PyTorch
// model (models/ambient_model.pt) and compare. This program has no ROS/OSQP
// dependencies -- it only exercises the pure-math generated header. Mirrors
// gen_residual_mlp_fixture.cpp (the ecrl equivalent) exactly, just at 8-D.
#include <cstdio>
#include <random>

#include <Eigen/Eigen>

#include "ommpc_ros2/residual_model_ambient.h"

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::fprintf(stderr, "usage: %s <output_csv_path>\n", argv[0]);
    return 1;
  }

  FILE *f = std::fopen(argv[1], "w");
  if (!f)
  {
    std::fprintf(stderr, "failed to open %s for writing\n", argv[1]);
    return 1;
  }

  std::fprintf(f, "x_0,x_1,x_2,x_3,x_4,x_5,x_6,x_7,"
                  "y_0,y_1,y_2,y_3,y_4,y_5,y_6,y_7,y_8\n");

  std::mt19937 rng(20260806);
  // Wide-ish spread (well beyond the trained input distribution's std) so the
  // fixture exercises both the near-linear (small-signal) and saturating
  // (large-signal) regimes of the hidden tanh units -- this is a pure
  // arithmetic parity check, not a check of in-distribution model behavior,
  // so unrealistic combinations are fine and useful here.
  std::uniform_real_distribution<double> unif(-3.0, 3.0);

  const int kNumRows = 1000;
  for (int i = 0; i < kNumRows; ++i)
  {
    Eigen::VectorXd x(8);
    for (int j = 0; j < 8; ++j)
    {
      x(j) = unif(rng);
    }

    const Eigen::VectorXd y = ommpc::residual::ambientForward(x);

    for (int j = 0; j < 8; ++j)
    {
      std::fprintf(f, "%.17g,", x(j));
    }
    for (int j = 0; j < 9; ++j)
    {
      std::fprintf(f, "%.17g%s", y(j), j == 8 ? "\n" : ",");
    }
  }

  std::fclose(f);
  std::fprintf(stderr, "wrote %d rows to %s\n", kNumRows, argv[1]);
  return 0;
}
