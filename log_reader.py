import sys
from pyulog import ULog
import numpy as np

# Convert quaternion to euler angles (roll, pitch, yaw) in radians
def quat_to_euler(w, x, y, z):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def analyze_flight_window(filepath):
    ulog = ULog(filepath)
    
    status_data = ulog.get_dataset('vehicle_status')
    rates_data = ulog.get_dataset('vehicle_rates_setpoint')
    pos_data = ulog.get_dataset('vehicle_local_position')
    att_data = ulog.get_dataset('vehicle_attitude')

    status_t = status_data.data['timestamp'] / 1e6
    nav_state = status_data.data['nav_state']
    
    rates_t = rates_data.data['timestamp'] / 1e6
    thrust_z = rates_data.data.get('thrust_body[2]', rates_data.data.get('thrust[2]', None))
    roll_rate_cmd = rates_data.data['roll']
    pitch_rate_cmd = rates_data.data['pitch']
    yaw_rate_cmd = rates_data.data['yaw']
    
    pos_t = pos_data.data['timestamp'] / 1e6
    alt = -pos_data.data['z']
    
    att_t = att_data.data['timestamp'] / 1e6
    q_w = att_data.data['q[0]']
    q_x = att_data.data['q[1]']
    q_y = att_data.data['q[2]']
    q_z = att_data.data['q[3]']

    print("t(s)  | Nav | Cmd Thr | Alt (m) | Roll Cmd | Pitch Cmd | Actual Roll | Actual Pitch | Actual Yaw")
    print("-" * 105)
    
    for t_query in np.arange(20.0, 32.0, 0.2):
        idx_status = np.abs(status_t - t_query).argmin()
        idx_rates = np.abs(rates_t - t_query).argmin()
        idx_pos = np.abs(pos_t - t_query).argmin()
        idx_att = np.abs(att_t - t_query).argmin()
        
        state = nav_state[idx_status]
        thrust = thrust_z[idx_rates] if thrust_z is not None else float('nan')
        roll_cmd = roll_rate_cmd[idx_rates]
        pitch_cmd = pitch_rate_cmd[idx_rates]
        altitude = alt[idx_pos]
        
        # Get actual attitude
        qw, qx, qy, qz = q_w[idx_att], q_x[idx_att], q_y[idx_att], q_z[idx_att]
        act_roll, act_pitch, act_yaw = quat_to_euler(qw, qx, qy, qz)
        
        # Convert to degrees for easier reading
        act_roll_deg = np.degrees(act_roll)
        act_pitch_deg = np.degrees(act_pitch)
        act_yaw_deg = np.degrees(act_yaw)
        
        print(f"{t_query:5.1f} | {state:3d} | {thrust:7.4f} | {altitude:7.4f} | {roll_cmd:8.4f} | {pitch_cmd:9.4f} | {act_roll_deg:11.1f}° | {act_pitch_deg:12.1f}° | {act_yaw_deg:10.1f}°")

if __name__ == "__main__":
    analyze_flight_window("/home/dinesh/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-07-13/06_05_22.ulg")