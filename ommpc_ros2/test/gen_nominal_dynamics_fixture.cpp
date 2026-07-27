// Generates a deterministic fixture of (state, control, dt, second-state) ->
// (nominalStep output, boxminus output) pairs computed by the C++
// nominal_dynamics.h, for scripts/test_residuals_parity.py to replay through
// its own Python port and compare. This program has no ROS/OSQP
// dependencies -- it only exercises the pure-math header.
#include <cstdio>
#include <random>

#include <Eigen/Eigen>

#include "ommpc_ros2/nominal_dynamics.h"

using ommpc::NominalInput;
using ommpc::NominalState;

namespace
{

Eigen::Quaterniond randomModestQuat(std::mt19937 &rng, std::uniform_real_distribution<double> &unif)
{
  // Keep rotation angles well away from the Log() branch cut near pi, and
  // away from the small-angle special case in SO3::log/exp, so this is a
  // generic, well-conditioned test point.
  Eigen::Vector3d axis(unif(rng), unif(rng), unif(rng));
  axis.normalize();
  double angle = 0.05 + 1.5 * (unif(rng) + 1.0) / 2.0; // in [0.05, 1.55] rad
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
  q.normalize();
  return q;
}

} // namespace

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

  std::fprintf(f,
               "p_x,p_y,p_z,v_x,v_y,v_z,q_w,q_x,q_y,q_z,"
               "thracc,wx,wy,wz,dt,"
               "p2_x,p2_y,p2_z,v2_x,v2_y,v2_z,q2_w,q2_x,q2_y,q2_z,"
               "step_p_x,step_p_y,step_p_z,step_v_x,step_v_y,step_v_z,step_q_w,step_q_x,step_q_y,step_q_z,"
               "bm_0,bm_1,bm_2,bm_3,bm_4,bm_5,bm_6,bm_7,bm_8\n");

  std::mt19937 rng(20260724);
  std::uniform_real_distribution<double> unif(-1.0, 1.0);

  const int kNumRows = 1000;
  for (int i = 0; i < kNumRows; ++i)
  {
    NominalState x;
    x.p = Eigen::Vector3d(2.0 * unif(rng), 2.0 * unif(rng), 1.0 + unif(rng));
    x.v = Eigen::Vector3d(unif(rng), unif(rng), unif(rng));
    x.q = randomModestQuat(rng, unif);

    NominalInput u;
    u.thracc = ommpc::kGravity * (1.0 + 0.3 * unif(rng));
    u.omega = Eigen::Vector3d(unif(rng), unif(rng), unif(rng));

    double dt = 0.02 + 0.03 * (unif(rng) + 1.0) / 2.0; // [0.02, 0.05] s

    NominalState x2;
    x2.p = Eigen::Vector3d(2.0 * unif(rng), 2.0 * unif(rng), 1.0 + unif(rng));
    x2.v = Eigen::Vector3d(unif(rng), unif(rng), unif(rng));
    x2.q = randomModestQuat(rng, unif);

    const NominalState x_next = ommpc::nominalStep(x, u, dt);
    const Eigen::Matrix<double, 9, 1> bm = ommpc::boxminus(x, x2);

    std::fprintf(f,
                 "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
                 "%.17g,%.17g,%.17g,%.17g,%.17g,"
                 "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
                 "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
                 "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
                 x.p(0), x.p(1), x.p(2), x.v(0), x.v(1), x.v(2), x.q.w(), x.q.x(), x.q.y(), x.q.z(), u.thracc,
                 u.omega(0), u.omega(1), u.omega(2), dt, x2.p(0), x2.p(1), x2.p(2), x2.v(0), x2.v(1), x2.v(2),
                 x2.q.w(), x2.q.x(), x2.q.y(), x2.q.z(), x_next.p(0), x_next.p(1), x_next.p(2), x_next.v(0),
                 x_next.v(1), x_next.v(2), x_next.q.w(), x_next.q.x(), x_next.q.y(), x_next.q.z(), bm(0), bm(1), bm(2),
                 bm(3), bm(4), bm(5), bm(6), bm(7), bm(8));
  }

  std::fclose(f);
  std::fprintf(stderr, "wrote %d rows to %s\n", kNumRows, argv[1]);
  return 0;
}
