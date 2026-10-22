builtin_interfaces/Time timestamp
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Quaternion quaternion
geometry_msgs/Vector3 body_rates
float64 thrust

# Live RLS thrust-to-acceleration estimate (MpcController::thr2acc) at this tick.
float64 thr2acc
# Mirrors OmmpcNode's FsmState enum: 0=WAIT_OFFBOARD 1=ARMED_WAIT 2=TAKEOFF 3=HOVER 4=TRACKING 5=LAND
uint8 fsm_state
# Raw OSQP work->info->status_val for this tick's solve; -1 if the MPC QP was not invoked this tick
# (e.g. ARMED_WAIT, which commands a fixed setpoint without solving).
int32 osqp_status

# Disturbance-observer trim / ecrl NN correction diagnostics (MpcController::
# last_trim_norm/last_ecrl_norm/last_correction_clipped). trim_norm is the
# leaky-integrated disturbance estimate's norm, tracked unconditionally
# regardless of enable_integral_trim; ecrl_norm is the raw ecrl NN
# prediction's norm (pre-clip), 0 when enable_residual_correction is off;
# correction_clipped is whether the combined ecrl+trim correction hit
# residual_clip_norm this cycle (only meaningful when at least one
# correction path is enabled).
float64 trim_norm
float64 ecrl_norm
bool correction_clipped

# Live per-cycle compute-time diagnostics, microseconds (MpcController::
# last_trim_time_us/last_ecrl_time_us/last_qp_solve_time_us/
# last_total_time_us). total_time_us covers execMPC from entry through the
# OSQP solve; trim/ecrl/qp_solve are disjoint sub-slices of it. For
# compute-cost benchmarking against the 100Hz/10ms control-loop budget.
float64 trim_time_us
float64 ecrl_time_us
float64 qp_solve_time_us
float64 total_time_us
