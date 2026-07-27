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
