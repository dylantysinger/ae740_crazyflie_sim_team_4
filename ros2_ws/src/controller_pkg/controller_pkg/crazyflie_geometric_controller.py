import numpy as np

from crazyflie_py import *
import rclpy
import rclpy.node

from crazyflie_interfaces.msg import ForceTorqueCmd

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Empty

import tf_transformations

# TODO overview:
#   PART 1: Choose the gains of the geometric controller (self.Kp, self.Kv, self.KR, self.Kw).
# DONE  PART 2: Add the ROS2 subscribers for the state data and the publishers for the command and reference path.
# DONE  PART 3: Parse the pose and twist messages into the state variables used by the controller.
#   PART 4: Implement the 'horizontal_circle' reference trajectory (position, velocity, acceleration, yaw).
#   PART 5: Implement the geometric controller that computes the collective thrust and body torques.
# DONE  PART 6: Implement cmd_force_torque to publish the force-torque command to the Crazyflie.
#   PART 7: Complete the control loop to compute and send the command, and stop the motors after landing.


class CrazyflieGeometricController(rclpy.node.Node):
    def __init__(self, node_name: str, mass: float, inertia: np.ndarray, rate: int):
        super().__init__(node_name)

        name = self.get_name()
        prefix = '/' + name

        self.rate = rate

        # Quadrotor parameters
        self.m = mass
        self.g = 9.81
        self.J = inertia
        self.e3 = np.array([0., 0., 1.])

        ############################################################################################################
        # [TODO] PART 1: Choose the gains of the geometric controller. Make sure to use given variable names.
        #
        # Each gain is a numpy array of size 3 (one gain per axis):
        # - self.Kp -> position gain         (N/m),      axes (x, y, z) of the world frame
        self.Kp = np.array([0.15,0.15,0.25])
        # - self.Kv -> velocity gain         (N s/m),    axes (x, y, z) of the world frame
        self.Kv = np.array([0.1,0.1,0.1])
        # - self.KR -> attitude gain         (Nm/rad),   axes (x, y, z) of the body frame
        self.KR = 8.0e-3*np.array([1.,1.,0.1])
        # - self.Kw -> angular velocity gain (Nm s/rad), axes (x, y, z) of the body frame
        self.Kw = 6.0e-4*np.array([1.,1.,0.1])

        #
        # Hints:
        #   1. Start by tuning the gains for takeoff and hover, then the circular trajectory.
        #   2. The attitude loop runs off-board with state feedback at 50 Hz from the crazyflie_server, so the
        #      attitude gains cannot be arbitrarily large. In the simulator, roll/pitch KR above ~1e-2 oscillates,
        #      and below ~4e-3 the drone cannot hold hover.


        # Command limits
        self.max_thrust = 0.55      # N  (hover thrust is about 0.27 N)
        self.max_torque = 2.0e-3    # Nm

        self.position = []
        self.velocity = []
        self.attitude = []
        self.R_WB = np.eye(3)
        self.omega_B = np.zeros(3)

        self.trajectory_changed = True

        self.flight_mode = 'idle'
        self.trajectory_t0 = self.get_clock().now()
        self.trajectory_type = 'horizontal_circle'
        self.plot_trajectory = True

        self.takeoff_duration = 5.0
        self.land_duration = 5.0

        self.takeoff_height = 1.0

        self.is_flying = False

        self.get_logger().info('Initialization completed...')

        ############################################################################################################
        # [TODO] PART 2: Add ROS2 subscribers for the Crazyflie state data, and publishers for the control command
        #                and the reference trajectory
        #
        # (a) Position subscriber
        # topic type -> PoseStamped
        # topic name -> {prefix}/pose (e.g., '/cf_1/pose')
        # callback -> self._pose_msg_callback

        # (b) Velocity subscriber
        # topic type -> TwistStamped
        # topic name -> {prefix}/twist
        # callback -> self._twist_msg_callback

        # (c) Reference trajectory path publisher
        # topic type -> Path
        # topic name -> {prefix}/geo_reference_path
        # publisher variable -> self.reference_path_pub

        # (d) Force-torque command publisher
        # topic type -> ForceTorqueCmd
        # topic name -> {prefix}/force_torque_cmd
        # publisher variable -> self.force_torque_pub

        self.position_sub       = self.create_subscription(PoseStamped, f'{prefix}/pose', self._pose_msg_callback, 10)
        self.velocity_sub       = self.create_subscription(TwistStamped, f'{prefix}/twist', self._twist_msg_callback, 10)

        self.reference_path_pub = self.create_publisher(Path, f'{prefix}/geo_reference_path', 10)
        self.force_torque_pub   = self.create_publisher(ForceTorqueCmd, f'{prefix}/force_torque_cmd', 10)


        self.takeoffService = self.create_subscription(Empty, f'/all/geo_takeoff', self.takeoff, 10)
        self.landService = self.create_subscription(Empty, f'/all/geo_land', self.land, 10)
        self.trajectoryService = self.create_subscription(Empty, f'/all/geo_trajectory', self.start_trajectory, 10)
        self.hoverService = self.create_subscription(Empty, f'/all/geo_hover', self.hover, 10)

        # Single control loop running at self.rate (Hz)
        self.create_timer(1./self.rate, self._control_loop)


    # [TODO] PART 3: Parse the ROS2 state messages. Make sure to use given variable names.
    #
    # NOTE:
    # - self.position -> numpy array (x, y, z) of p^W
    # - self.attitude -> numpy array of the Euler angles (roll, pitch, yaw), wrapped between -pi and +pi
    # - self.R_WB     -> 3x3 numpy array, rotation matrix R^W_B of the current attitude
    # - self.velocity -> numpy array of v^W (velocity in the WORLD frame)
    # - self.omega_B  -> numpy array of omega^B (angular velocity in the BODY frame, rad/s)
    #
    # Hints:
    #   1. Look at the PoseStamped and TwistStamped message structures at
    #      https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html.
    #   2. tf_transformations has functions for Euler angles and rotation matrices from a quaternion
    #      (quaternion_matrix returns a 4x4 homogeneous matrix).
    #   3. The twist message is filled by the crazyflie_server from the firmware log variables
    #      kalman.statePX/PY/PZ (velocity in the BODY frame) and gyro.x/y/z (angular rates in deg/s).

    def _pose_msg_callback(self, msg: PoseStamped):

        self.position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        quaternion = msg.pose.orientation
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        euler = np.array(tf_transformations.euler_from_quaternion(q))
        self.attitude = (euler+np.pi) % (2*np.pi) - np.pi

        self.R_WB = tf_transformations.euler_matrix(euler[0], euler[1], euler[2]) [:3, :3]

        # return # remove this statement after finishing this part


    def _twist_msg_callback(self, msg: TwistStamped):
        v = msg.twist.linear
        v_B = np.array([v.x, v.y, v.z])

        omega = msg.twist.angular

        self.velocity = self.R_WB @ v_B
        self.omega_B  = np.deg2rad(np.array([omega.x, omega.y, omega.z]))

        # return # remove this statement after finishing this part


    def start_trajectory(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'trajectory'

    def takeoff(self, msg):
        if len(self.position) == 0:
            self.get_logger().warning("No pose received yet, ignoring takeoff.")
            return
        self.trajectory_changed = True
        self.flight_mode = 'takeoff'
        self.go_to_position = np.array([self.position[0],
                                        self.position[1],
                                        self.takeoff_height])

    def hover(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'hover'
        self.go_to_position = np.array([self.position[0],
                                        self.position[1],
                                        self.position[2]])

    def land(self, msg):
        self.trajectory_changed = True
        self.flight_mode = 'land'
        self.go_to_position = np.array([self.position[0],
                                        self.position[1],
                                        0.05])


    # [TODO] PART 4: Implement the trajectory type 'horizontal_circle' starting at self.trajectory_start_position.
    # Instructions:
    # - Use self.trajectory_start_position as the starting position (not the center).
    # - Use a radius a = 1.0 m, and let the angular velocity (the rate of change of the angle along the circle)
    #   ramp up smoothly as omega(t) = 0.75 * tanh(0.1 t).
    # - Compute the reference position (pxr, pyr, pzr), velocity (vxr, vyr, vzr) and acceleration (axr, ayr, azr).
    # - Keep the reference yaw angle yawr = 0.
    # - Return [pxr, pyr, pzr, vxr, vyr, vzr, axr, ayr, azr, yawr].

    def trajectory_function(self, t):
        if self.trajectory_type == 'horizontal_circle':
            radius = 1.0 #m
            theta = 0.75*10.0*np.log(np.abs(np.cosh(0.1*t)))
            omega = np.array([0,0,0.75 * np.tanh(0.1*t)]) #rad/s
            omegaDot = np.array([0, 0, 0.75 * 0.1 / np.cosh(0.1*t)**2]) #rad/s/s

            O_B_A = np.array([[np.cos(theta),np.sin(theta),0],[-np.sin(theta),np.cos(theta),0],[0,0,1]])

            rQuadWRTCenterInB = np.array([radius,0,0])
            rCenterWRTOriginInA = self.trajectory_start_position - rQuadWRTCenterInB
            r = O_B_A.T @ rQuadWRTCenterInB

            p = np.transpose(O_B_A) @ rQuadWRTCenterInB + rCenterWRTOriginInA

            pxr = p[0]
            pyr = p[1]
            pzr = p[2]

            # Transport theorem
            v = self.supercross(omega) @ r 
            vxr = v[0]
            vyr = v[1]
            vzr = v[2]

            # Double transport theorem
            a = self.supercross(omegaDot) @ r + self.supercross(omega) @ v
            axr = a[0]
            ayr = a[1]
            azr = a[2]

            yawr = 0.0

            return np.array([pxr,pyr,pzr,vxr,vyr,vzr,axr,ayr,azr,yawr])

        elif self.trajectory_type == 'wavy_circle':

            
            r = 1.0 # Radius of circle
            omega = 0.5 # angular velocity
            A = 0.3 # Vertical oscillation amplitude
            n = 2 # number of oscillations per revolution

            x0, y0, z0 = self.trajectory_start_position

            # Position
            pxr = x0 + r*(np.cos(omega*t) - 1) # I think this needs a -1 to start the reference at x0,y0,z0
            pyr = y0 + r*np.sin(omega*t)
            pzr = z0 + A*np.sin(n*omega*t)

            # Velocity
            vxr = -r*omega*np.sin(omega*t)
            vyr = r*omega*np.cos(omega*t)
            vzr = A*n*omega*np.cos(n*omega*t)

            # Acceleration
            axr = -r*omega**2*np.cos(omega*t)
            ayr = -r*omega**2*np.sin(omega*t)
            azr = -A*(n*omega)**2*np.sin(n*omega*t)

            yawr = 0.0

            return np.array([pxr,pyr,pzr,vxr,vyr,vzr,axr,ayr,azr,yawr])
        
        else:
            return np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])

    def supercross(self, a):
        super = np.array([[0, -a[2], a[1]],[a[2], 0,-a[0]],[-a[1],a[0],0]])
        return super

    def navigator(self, t):
        # Returns the desired p^W_d, v^W_d, p_ddot^W_d and yaw psi at time t
        if self.flight_mode == 'takeoff' or self.flight_mode == 'land':
            T = self.takeoff_duration if self.flight_mode == 'takeoff' else self.land_duration
            k = 12.0 / T
            s = 1./(1. + np.exp(-(k*(t - T) + 6.0)))       # smooth step from 0 to 1 over 2T seconds
            s_dot = k*s*(1. - s)
            delta = self.go_to_position - self.trajectory_start_position
            ref = np.array([*(self.trajectory_start_position + s*delta), *(s_dot*delta), 0., 0., 0., 0.])
        elif self.flight_mode == 'trajectory':
            ref = self.trajectory_function(t)
        else:   # hover
            ref = np.array([*self.go_to_position, 0., 0., 0., 0., 0., 0., 0.])

        self.tLast = t
        return ref[0:3], ref[3:6], ref[6:9], ref[9]


    # [TODO] PART 5: Implement the geometric controller discussed in class.
    # Inputs:  desired position p_d, velocity v_d, acceleration a_d (all in the world frame) and desired yaw yaw_d.
    # Outputs: collective thrust f_z (scalar, N) and body torques tau (numpy array of size 3, Nm).
    #
    # Instructions:
    # - Use the current state from PART 3 (self.position, self.velocity, self.R_WB, self.omega_B).
    # - Use the gains self.Kp, self.Kv, self.KR, self.Kw and the parameters self.m, self.g, self.J.
    # - You can assume that the desired angular velocity is zero.
    # - Saturate f_z to [0, self.max_thrust] and each torque to [-self.max_torque, self.max_torque].

    # def geometric_controller(self, p_d, v_d, a_d, yaw_d):
    #     ...
    #     return f_z, tau

    def geometric_controller(self, p_d, v_d, a_d, yaw_d):

        # Errors
        e_p = self.position - p_d
        e_v = self.velocity - v_d

        # Desired total force in the inertial (Or I guess world) frame
        F_des = (-self.Kp*e_p - self.Kv*e_v + self.m*self.g*self.e3 + self.m *a_d)

        # Desired body z-axis
        b3_d = F_des / np.linalg.norm(F_des)

        # Desired heading from yaw
        b1_c = np.array([
                        np.cos(yaw_d),
                        np.sin(yaw_d),
                        0.0])

        # Desired rot matrix R_d
        b2_d = np.cross(b3_d, b1_c)
        b2_d = b2_d / np.linalg.norm(b2_d)

        b1_d = np.cross(b2_d, b3_d)

        R_d = np.column_stack((b1_d, b2_d, b3_d))

        # Attitude Error
        e_R_matrix = 0.5* (R_d.T @ self.R_WB - self.R_WB.T @ R_d)
        e_R = np.array([e_R_matrix[2,1], e_R_matrix[0,2], e_R_matrix[1,0]])

        # Angular velocity error
        e_w = self.omega_B 

        # Collective Thrust
        f_z = F_des @ (self.R_WB @ self.e3)

        # Body Torque
        tau = (-self.KR*e_R - self.Kw*e_w + np.cross(self.omega_B, self.J @ self.omega_B))

        # Saturation
        f_z = np.clip(f_z, 0.0, self.max_thrust)

        tau = np.clip(tau, -self.max_torque, self.max_torque)

        return f_z, tau


    # [TODO] PART 6: Implement the cmd_force_torque function to publish the force-torque commands.
    # Instructions:
    # - Create a ForceTorqueCmd message
    # - Set the thrust_si, torque_x, torque_y and torque_z fields from the input parameters
    #   (the message fields are float64, convert numpy values with float())
    # - Publish the message using self.force_torque_pub
    # - See the structure of the message in
    #       ae740_crazyflie_sim/ros2_ws/src/crazyswarm2/crazyflie_interfaces/msg/ForceTorqueCmd.msg
    #
    # def cmd_force_torque(self, thrust, torque):
    def cmd_force_torque(self, thrust, torque):
        msg = ForceTorqueCmd()
        msg.thrust_si = float(thrust)       # [N]
        msg.torque_x  = float(torque[0])    # [N*m]
        msg.torque_y  = float(torque[1])    # [N*m]
        msg.torque_z  = float(torque[2])    # [N*m]
        self.force_torque_pub.publish(msg)


    def publish_reference_path(self, t):
        reference_path = Path()
        reference_path.header.frame_id = 'world'
        reference_path.header.stamp = self.get_clock().now().to_msg()

        for t_ref in np.linspace(t, t + 1.0, 20):
            p_d, _, _, _ = self.navigator(t_ref)
            ref_pose = PoseStamped()
            ref_pose.pose.position.x = p_d[0]
            ref_pose.pose.position.y = p_d[1]
            ref_pose.pose.position.z = p_d[2]
            reference_path.poses.append(ref_pose)

        self.reference_path_pub.publish(reference_path)

    def _control_loop(self):
        if self.flight_mode == 'idle':
            return

        if len(self.position) == 0 or len(self.velocity) == 0 or len(self.attitude) == 0:
            self.get_logger().warning("Empty state message.")
            return

        if not self.is_flying:
            self.is_flying = True

        if self.trajectory_changed:
            self.trajectory_start_position = self.position.copy()
            self.trajectory_t0 = self.get_clock().now()
            self.trajectory_changed = False

        t = (self.get_clock().now() - self.trajectory_t0).nanoseconds / 10.0**9

        # [TODO] PART 7: Compute and send the geometric control command at the current time step
        #
        # p_d, v_d, a_d, yaw_d = ... (reference at time t, see self.navigator(t))
        # f_z, tau = ... (geometric control law, PART 5)
        # ... (publish the command, PART 6)
        #
        # Once the landing is finished (flight_mode == 'land' and t > 2*self.land_duration), send a zero
        # command to stop the motors, and set self.flight_mode = 'idle' and self.is_flying = False.

        # stop once landing is complete
        if self.flight_mode == 'land' and t > 2*self.land_duration:

            self.cmd_force_torque(0.0, np.zeros(3))

            self.flight_mode = 'idle'
            self.is_flying = False

            return

        # desired reference
        p_d, v_d, a_d, yaw_d = self.navigator(t)

        # activate geo controller
        f_z, tau = self.geometric_controller(p_d, v_d, a_d, yaw_d)

        self.cmd_force_torque(f_z, tau)

        if self.plot_trajectory:
            self.publish_reference_path(t)

        self.get_logger().info(
            f'mode={self.flight_mode} t={t:.1f} '
            f'pos={np.round(self.position,3)} p_d={np.round(p_d,3)} '
            f'vel={np.round(self.velocity,3)} f_z={f_z:.3f} tau={np.round(tau,6)}',
            throttle_duration_sec=0.5)

def main():
    rclpy.init()

    # Quadrotor Parameters
    mass = 0.028
    Ixx = 2.3951e-5
    Iyy = 2.3951e-5
    Izz = 3.2347e-5
    inertia = np.diag([Ixx, Iyy, Izz])

    rate = 100 # control update rate (in Hz)
    quad_name = 'cf_1'

    node = CrazyflieGeometricController(quad_name, mass, inertia, rate)

    # Standard node commands
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()
