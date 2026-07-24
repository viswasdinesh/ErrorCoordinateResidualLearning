#!/usr/bin/env python3

import os
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import math
import random
import json
import time
import numpy as np

import rclpy
from rclpy.node import Node
from ommpc_ros2.msg import PolyTraj
from builtin_interfaces.msg import Time

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        
        self.publisher_ = self.create_publisher(PolyTraj, '/drone_0_planning/trajectory', 10)
        
        # Parameters
        self.declare_parameter('num_trajectories', 300)
        self.declare_parameter('traj_duration_min', 8.0)
        self.declare_parameter('traj_duration_max', 12.0)
        self.declare_parameter('hover_height', 1.5)
        self.declare_parameter('xy_range', 2.0)
        self.declare_parameter('z_min', 0.8)
        self.declare_parameter('z_max', 2.2)
        self.declare_parameter('delay_between_trajectories', 20.0) # seconds
        self.declare_parameter('save_file', '/home/dinesh/ecrl_datasets/generated_trajectories.json')
        self.declare_parameter('time_scales', [1.0, 1.25, 1.5])
        self.declare_parameter('windy', False)
        
        self.num_trajectories = self.get_parameter('num_trajectories').value
        self.traj_duration_min = self.get_parameter('traj_duration_min').value
        self.traj_duration_max = self.get_parameter('traj_duration_max').value
        self.hover_height = self.get_parameter('hover_height').value
        self.xy_range = self.get_parameter('xy_range').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.delay_between_trajectories = self.get_parameter('delay_between_trajectories').value
        self.save_file = self.get_parameter('save_file').value
        self.time_scales = self.get_parameter('time_scales').value
        self.windy = self.get_parameter('windy').value

        if self.windy:
            directory = os.path.dirname(self.save_file)
            filename = os.path.basename(self.save_file)
            if not os.path.basename(directory) == 'windy':
                self.save_file = os.path.join(directory, 'windy', filename)

        self.param_client = self.create_client(SetParameters, '/ecrl_data_collector_node/set_parameters')
        self.get_logger().info("Waiting for /ecrl_data_collector_node/set_parameters service...")
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return
            self.get_logger().info('Waiting for parameter service...')

        self.get_logger().info(f"Initialized Trajectory Generator Node.")
        self.get_logger().info(f"Generating {self.num_trajectories} trajectories using time scales: {self.time_scales}.")

    def bernstein_to_power(self, control_points, T):
        """
        Converts Bernstein control points to power basis coefficients.
        control_points: numpy array of shape (N+1, 3)
        T: duration of the piece
        Returns: coeff: list of floats for x, y, z of size (N+1)
        """
        N = len(control_points) - 1
        coeff_x = [0.0] * (N + 1)
        coeff_y = [0.0] * (N + 1)
        coeff_z = [0.0] * (N + 1)
        
        # a_k = (1 / T^k) * binom(N, k) * sum_{i=0}^k (-1)^(k-i) * binom(k, i) * P_i
        for k in range(N + 1):
            sum_val = np.zeros(3)
            for i in range(k + 1):
                term = ((-1) ** (k - i)) * math.comb(k, i) * control_points[i]
                sum_val += term
            a_k = sum_val * math.comb(N, k) / (T ** k)
            
            # Save in descending order of power (t^N to t^0)
            coeff_x[N - k] = float(a_k[0])
            coeff_y[N - k] = float(a_k[1])
            coeff_z[N - k] = float(a_k[2])
            
        return coeff_x, coeff_y, coeff_z

    def generate_trajectories(self):
        trajectories = []
        p_start = np.array([0.0, 0.0, self.hover_height])
        p_end = np.array([0.0, 0.0, self.hover_height])
        
        num_base = math.ceil(self.num_trajectories / len(self.time_scales))
        traj_counter = 1
        
        for i in range(num_base):
            if traj_counter > self.num_trajectories:
                break
                
            # Degree 7 Bernstein polynomial
            # P0, P1, P2 = p_start for zero velocity and acceleration at start
            # P5, P6, P7 = p_end for zero velocity and acceleration at end
            # P3, P4 are free control points
            
            # Varying "weights" or spatial scale of the control points
            scale_factor = random.uniform(0.5, 1.5)
            xy_lim = self.xy_range * scale_factor
            
            p3 = np.array([
                random.uniform(-xy_lim, xy_lim),
                random.uniform(-xy_lim, xy_lim),
                random.uniform(self.z_min, self.z_max)
            ])
            p4 = np.array([
                random.uniform(-xy_lim, xy_lim),
                random.uniform(-xy_lim, xy_lim),
                random.uniform(self.z_min, self.z_max)
            ])
            
            control_points = np.zeros((8, 3))
            control_points[0] = p_start
            control_points[1] = p_start
            control_points[2] = p_start
            control_points[3] = p3
            control_points[4] = p4
            control_points[5] = p_end
            control_points[6] = p_end
            control_points[7] = p_end
            
            base_duration = random.uniform(self.traj_duration_min, self.traj_duration_max)
            
            for s in self.time_scales:
                if traj_counter > self.num_trajectories:
                    break
                
                # Time scaling: divide duration by s (s > 1 makes the trajectory faster)
                duration = base_duration / s
                
                coeff_x, coeff_y, coeff_z = self.bernstein_to_power(control_points, duration)
                
                traj = {
                    'traj_id': traj_counter,
                    'order': 7,
                    'duration': [float(duration)],
                    'coef_x': coeff_x,
                    'coef_y': coeff_y,
                    'coef_z': coeff_z,
                    'control_points': control_points.tolist(),
                    'scale': scale_factor,
                    'time_scale_factor': float(s)
                }
                trajectories.append(traj)
                traj_counter += 1
            
        # Save to file
        os.makedirs(os.path.dirname(self.save_file), exist_ok=True)
        with open(self.save_file, 'w') as f:
            json.dump(trajectories, f, indent=4)
        self.get_logger().info(f"Saved {self.num_trajectories} generated trajectories to {self.save_file}")
        
        return trajectories

    def run(self):
        trajectories = self.generate_trajectories()
        
        self.get_logger().info("Starting sequential execution in 5 seconds. Make sure PX4 and the ommpc_node are running and armed.")
        time.sleep(5.0)
        
        for idx, traj in enumerate(trajectories):
            if not rclpy.ok():
                break
            
            # Set the parameter on the data collector node dynamically to start a new CSV file
            base_dir = os.path.dirname(self.save_file)
            new_csv = os.path.join(base_dir, f"polynomial{traj['traj_id']}.csv")
            self.get_logger().info(f"Setting data collector CSV path to: {new_csv}")
            
            req = SetParameters.Request()
            param = Parameter()
            param.name = "csv_filepath"
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = new_csv
            req.parameters = [param]
            
            success = False
            for attempt in range(3):
                future = self.param_client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                if future.done():
                    result = future.result()
                    if result.results[0].successful:
                        self.get_logger().info(f"Successfully set csv_filepath to {new_csv} (attempt {attempt+1})")
                        success = True
                        break
                    else:
                        self.get_logger().warn(f"Param update attempt {attempt+1} failed: {result.results[0].reason}, retrying...")
                else:
                    self.get_logger().warn(f"Param update attempt {attempt+1} timed out, retrying...")
                time.sleep(1.0)
                
            if not success:
                self.get_logger().error(f"Failed to set csv_filepath to {new_csv} after 3 attempts.")
                
            self.get_logger().info(f"Publishing trajectory {traj['traj_id']}/{self.num_trajectories} (duration: {traj['duration'][0]:.2f}s, scale: {traj['scale']:.2f})")
            
            msg = PolyTraj()
            msg.drone_id = 0
            msg.traj_id = int(traj['traj_id'])
            msg.order = int(traj['order'])
            msg.coef_x = traj['coef_x']
            msg.coef_y = traj['coef_y']
            msg.coef_z = traj['coef_z']
            msg.duration = traj['duration']
            
            # Set start_time to slightly in the past (e.g. 0.05s) to guarantee now_time >= start_time 
            # and trigger the queue pop_front in feed_from_traj_utils
            now = self.get_clock().now()
            start_time_ns = now.nanoseconds - int(0.05 * 1e9)
            msg.start_time.sec = int(start_time_ns // 1000000000)
            msg.start_time.nanosec = int(start_time_ns % 1000000000)
            
            self.publisher_.publish(msg)
            
            # Wait for execution and hover duration before sending the next one
            sleep_time = traj['duration'][0] + self.delay_between_trajectories
            self.get_logger().info(f"Waiting {sleep_time:.2f} seconds for trajectory execution and data collection...")
            
            # Sleep in small increments to allow ROS callbacks/signals to interrupt if needed
            wake_time = time.time() + sleep_time
            while time.time() < wake_time and rclpy.ok():
                time.sleep(0.5)

        self.get_logger().info("All 100 trajectories have been published and completed.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Trajectory generation interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
