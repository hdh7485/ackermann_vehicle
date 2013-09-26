#!/usr/bin/env python

"""ackermann_controller.py

Control the wheels of a vehicle with Ackermann steering.

Subscribed Topics:
    ackermann_cmd (ackermann_msgs/AckermannDriveStamped)
        Ackermann command. It contains the vehicle's desired speed and steering
        angle.

Published Topics:
    TODO

Parameters:
    left_front_shock_name
    left_front_shock_spring_constant
    left_front_shock_damping_coefficient
    left_front_shock_equilibrium_position
    left_front_axle_controller_name

    left_steering_controller_name
    right_steering_controller_name

    ~cmd_timeout (float, default: 0.5)
        If this node does not receive a command for more than cmd_timeout
        seconds, vehicle motion is paused until a command is received.
        cmd_timeout must be greater than zero.
    ~publishing_frequency (float, default: 30.0)
        Joint command publishing frequency, measured in hertz. It must
        be greater than zero.

Copyright (c) 2013 Wunderkammer Laboratory
TODO
"""

import math
import threading

from math import pi

import rospy
import tf

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class _AckermannCtrlr(object):
    """Ackermann controller

    An object of class _AckermannCtrlr is a node that controls the wheels of a
    vehicle with Ackermann steering.
    """

    def __init__(self):
        """Initialize this _AckermannCtrlr."""

        rospy.init_node("ackermann_controller")

        # Parameters

        # Left front wheel
        self._left_front_wheel_dia = \
            rospy.get_param("~left_front_wheel/diameter", 1.0)
        self._left_front_circ = pi * self._left_front_wheel_dia
        self._left_steer_ctrlr_name = \
            rospy.get_param("~left_front_wheel/steering_controller_name",
                            self._DEF_LEFT_STEER_CTRLR_NAME)
        self._left_front_axle_ctrlr_name = \
            rospy.get_param("~left_front_wheel/axle_controller_name",
                            self._DEF_LEFT_FRONT_AXLE_CTRLR_NAME)
        self._left_front_steer_joint_name = \
            rospy.get_param("~left_front_wheel/steering_joint_name",
                            self._DEF_LEFT_FRONT_STEER_JOINT_NAME)

        # Right front wheel
        self._right_front_wheel_dia = \
            rospy.get_param("~right_front_wheel/diameter", 1.0)
        self._right_front_circ = pi * self._right_front_wheel_dia
        self._right_steer_ctrlr_name = \
            rospy.get_param("~right_front_wheel/steering_controller_name",
                            self._DEF_RIGHT_STEER_CTRLR_NAME)
        self._right_front_axle_ctrlr_name = \
            rospy.get_param("~right_front_wheel/axle_controller_name",
                            self._DEF_RIGHT_FRONT_AXLE_CTRLR_NAME)
        self._right_front_steer_joint_name = \
            rospy.get_param("~right_front_wheel/steering_joint_name",
                            self._DEF_RIGHT_FRONT_STEER_JOINT_NAME)

        # Left rear wheel
        self._left_rear_wheel_dia = \
            rospy.get_param("~left_rear_wheel/diameter", 1.0)
        self._left_rear_circ = pi * self._left_rear_wheel_dia
        self._left_rear_axle_ctrlr_name = \
            rospy.get_param("~left_rear_wheel/axle_controller_name",
                            self._DEF_LEFT_REAR_AXLE_CTRLR_NAME)
        self._left_rear_axle_joint_name = \
            rospy.get_param("~left_rear_wheel/axle_joint_name",
                            self._DEF_LEFT_REAR_AXLE_JOINT_NAME)

        # Right rear wheel
        self._right_rear_wheel_dia = \
            rospy.get_param("~right_rear_wheel/diameter", 1.0)
        self._right_rear_circ = pi * self._right_rear_wheel_dia
        self._right_rear_axle_ctrlr_name = \
            rospy.get_param("~right_rear_wheel/axle_controller_name",
                            self._DEF_RIGHT_REAR_AXLE_CTRLR_NAME)
        self._right_rear_axle_joint_name = \
            rospy.get_param("~right_rear_wheel/axle_joint_name",
                            self._DEF_RIGHT_REAR_AXLE_JOINT_NAME)

        # Shock absorber controllers
        shock_ctrlrs = rospy.get_param("~shock_absorber_controllers", [])
        self._eq_positions = []    # Equilibrium positions
        self._shock_cmd_pubs = []  # Shock absorber command publishers
        try:
            for shock_ctrlr in shock_ctrlrs:
                try:
                    pub = rospy.Publisher(shock_ctrlr["name"] + "/command",
                                          Float64, latch=True)
                    pos = shock_ctrlr["equilibrium_position"]
                    pub.publish(pos)
                    self._eq_positions.append(pos)
                    self._shock_cmd_pubs.append(pub)
                except:
                    pass
        except:
            pass

        # Command timeout
        try:
            timeout = float(rospy.get_param("~cmd_timeout",
                                            self._DEF_CMD_TIMEOUT))
            if timeout <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified command timeout value is invalid. "
                          "The default timeout value will be used instead.")
            timeout = self._DEF_CMD_TIMEOUT
        self._cmd_timeout = rospy.Duration.from_sec(timeout)

        # Publishing frequency
        try:
            pub_freq = float(rospy.get_param("~publishing_frequency",
                                             self._DEF_PUB_FREQ))
            if pub_freq <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified publishing frequency is invalid. "
                          "The default frequency will be used instead.")
            pub_freq = self._DEF_PUB_FREQ
        self._sleep_timer = rospy.Rate(pub_freq)

        # _ackermann_cmd_lock is used to control access to _steer_ang,
        # _steer_ang_vel, _speed, _accel, and _jerk.
        self._ackermann_cmd_lock = threading.Lock()
        self._steer_ang = 0.0      # Steering angle
        self._steer_ang_vel = 0.0  # Steering angle velocity
        self._speed = 0.0
        self._accel = 0.0          # Acceleration
        self._jerk = 0.0

        self._last_steer_ang = 0.0  # Last steering angle
        self._theta_left = 0.0      # Angle of the left steering joint
        self._theta_right = 0.0     # Angle of the right steering joint

        self._last_speed = 0.0
        self._last_accel_limit = 0.0
        self._left_front_ang_vel = 0.0
        self._right_front_ang_vel = 0.0
        self._left_rear_ang_vel = 0.0
        self._right_rear_ang_vel = 0.0

        # _joint_dist_div_2 is the distance between the steering joints,
        # divided by two. inv_wheelbase_ is the inverse of wheelbase_.
        tf_listener = tf.TransformListener()
        lf_pos = _get_link_pos(tf_listener, self._left_front_steer_joint_name)
        rf_pos = _get_link_pos(tf_listener, self._right_front_steer_joint_name)
        self._joint_dist_div_2 = abs(lf_pos[1] - rf_pos[1]) / 2  # TODO
        lr_pos = _get_link_pos(tf_listener, self._left_rear_axle_joint_name)
        self._wheelbase = abs(lf_pos[0] - lr_pos[0])  # TODO
        self._inv_wheelbase = 1 / self._wheelbase
        self._wheelbase_sqr = pow(self._wheelbase, 2)

        # Subscribers and publishers

        self._ackermann_cmd_sub = \
            rospy.Subscriber("ackermann_cmd", AckermannDriveStamped,
                             self.ackermann_cmd_cb, queue_size=1)

        self._left_steer_cmd_pub = \
            rospy.Publisher(self._left_steer_ctrlr_name + "/command", Float64)
        self._right_steer_cmd_pub = \
            rospy.Publisher(self._right_steer_ctrlr_name + "/command", Float64)

        self._left_front_axle_cmd_pub = \
            rospy.Publisher(self._left_front_axle_ctrlr_name + "/command",
                            Float64)
        # TODO: There can be zero or more axle controllers.
        self._right_front_axle_cmd_pub = \
            rospy.Publisher(self._right_front_axle_ctrlr_name + "/command",
                            Float64)
        self._left_rear_axle_cmd_pub = \
            rospy.Publisher(self._left_rear_axle_ctrlr_name + "/command",
                            Float64)
        self._right_rear_axle_cmd_pub = \
            rospy.Publisher(self._right_rear_axle_ctrlr_name + "/command",
                            Float64)

        # # Get the wheel module positions.
        # self._wheel_mod_positions = []
        # tf_listener = tf.TransformListener()
        # self._wheel_mod_positions.append(_get_wheel_mod_pos(
        #         tf_listener, "front_steer_axle"))
        # self._wheel_mod_positions.append(_get_wheel_mod_pos(
        #         tf_listener, "left_steer_axle"))
        # self._wheel_mod_positions.append(_get_wheel_mod_pos(
        #         tf_listener, "right_steer_axle"))

        # # Compute the robot's maximum angular velocity. _max_ang_vel
        # # unit: rad/s.
        # max_radius = 0.0
        # for pos in self._wheel_mod_positions:
        #     radius = math.sqrt(numpy.dot(pos, pos))
        #     if radius > max_radius:
        #         max_radius = radius
        # circ = (2 * pi) * max_radius
        # self._max_ang_vel = (2 * pi) * (self._MAX_LIN_VEL / circ)
        # #rospy.loginfo("The maximum angular velocity is %f." %
        # #              self._max_ang_vel)

        # # _last_cmd_time is the time at which the most recent velocity
        # # command was received. _wheels_stopped is True if wheel motion has
        # # been commanded to stop.
        # self._vel_cmd = None  # Velocity command
        # self._last_cmd_time = rospy.get_rostime()
        # self._last_lin_vel = numpy.array((0.0, 0.0))
        # self._last_ang_vel = 0.0
        # self._last_vel_time = None
        # self._sleep_timer = rospy.Rate(pub_freq)
        # self._wheels_stopped = False

        # # _joint_states contains joint states received from "joint_states".
        # # _joint_cmd contains a joint command published to
        # # "wheel_joint_command".

        # self._joint_states = None

        # self._joint_cmd = JointState()
        # self._joint_cmd.header.frame_id = "/base_link"  # TODO: tf_prefix
        # self._joint_cmd.name = \
        #     ["front_steering_joint", "front_axle_joint",
        #      "left_steering_joint", "left_axle_joint",
        #      "right_steering_joint", "right_axle_joint"]
        # self._joint_cmd.position = 6 * [0.0]
        # self._joint_cmd.velocity = 6 * [0.0]

        # # _vel_cmd_sub receives velocity commands. _joint_state_sub receives
        # # joint positions and velocities. _joint_cmd_pub publishes wheel joint
        # # commands.
        # self._vel_cmd_sub = rospy.Subscriber("cmd_vel", Twist, self.vel_cmd_cb,
        #                                      queue_size=1)

        # # Subscribers and publishers

        # <!-- Steering controller -->
        # <controller:ackermann_steering_controller name="steering_controller" plugin="libackermann_steering_controller.so">
        #   <alwaysOn>true</alwaysOn>
        #   <updateRate>1000</updateRate>
        #
        #   <leftJointName>left_steering_joint</leftJointName>
        #   <rightJointName>right_steering_joint</rightJointName>
        #   <jointDistance>${hex_hub_dist - 2 * axle_length}</jointDistance>
        #   <wheelbase>${wheelbase}</wheelbase>
        # </controller:ackermann_steering_controller>

    def spin(self):
        """Control the vehicle."""

        last_time = rospy.get_time()

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            if delta_t > 0.0:
                last_time = t
                with self._ackermann_cmd_lock:
                    steer_ang = self._steer_ang
                    steer_ang_vel = self._steer_ang_vel
                    speed = self._speed
                    accel = self._accel
                    jerk = self._jerk
                    steer_ang_changed, x = \
                        self._ctrl_steering(steer_ang, steer_ang_vel, delta_t)
                self._ctrl_axles(speed, accel, jerk, delta_t,
                                 steer_ang_changed, x)
                self._ctrl_shocks()
            self._sleep_timer.sleep()
            # if self._joint_cmd_pub.get_num_connections() > 0:
            #     # If too much time has elapsed since the last velocity command
            #     # was received, stop the robot.
            #     stop_wheels = (self._vel_cmd is None or
            #                    rospy.get_rostime() - self._last_cmd_time >
            #                    self._vel_cmd_timeout)
            #     self._proc_vel_cmd(stop_wheels)

    def ackermann_cmd_cb(self, ackermann_cmd):
        """Ackermann driving command callback

        :Parameters:
          ackermann_cmd: ackermann_msgs.msg.AckermannDriveStamped
            Ackermann driving command.
        """
        with self._ackermann_cmd_lock:
            self._steer_ang = ackermann_cmd.drive.steering_angle
            self._steer_ang_vel = ackermann_cmd.drive.steering_angle_velocity
            self._speed = ackermann_cmd.drive.speed
            self._accel = ackermann_cmd.drive.acceleration
            self._jerk = ackermann_cmd.drive.jerk

    # def vel_cmd_cb(self, vel_cmd):
    #     """Velocity command callback

    #     :Parameters:
    #       vel_cmd: geometry_msgs.msg.Twist
    #         Velocity command.
    #     """
    #     # Compute the linear and angular velocities given acceleration limits.

    #     delta_lin_vel = (numpy.array((vel_cmd.linear.x, vel_cmd.linear.y)) -
    #                      self._last_lin_vel)
    #     delta_ang_vel = vel_cmd.angular.z - self._last_ang_vel
    #     time = rospy.get_rostime()
    #     if self._last_vel_time is None:
    #         self._last_vel_time = time
    #     delta_t = (time - self._last_vel_time).to_sec()

    #     if delta_t > 0.0:
    #         lin_accel = delta_lin_vel / delta_t
    #         # lin_accel_mag: Linear acceleration magnitude. Unit: m/s/s.
    #         lin_accel_mag = math.sqrt(numpy.dot(lin_accel, lin_accel))
    #         if lin_accel_mag > 0.0:
    #             lin_accel_normed = lin_accel / lin_accel_mag
    #             lin_accel_mag = min(lin_accel_mag, self._MAX_LIN_ACCEL)
    #             delta_lin_vel = delta_t * lin_accel_mag * lin_accel_normed
    #         else:
    #             delta_lin_vel = numpy.array((0.0, 0.0))

    #         ang_accel = delta_ang_vel / delta_t
    #         delta_ang_vel = delta_t * max(-self._MAX_ANG_ACCEL,
    #                                        min(ang_accel, self._MAX_ANG_ACCEL))
    #     else:
    #         delta_lin_vel = numpy.array((0.0, 0.0))
    #         delta_ang_vel = 0.0

    #     self._last_lin_vel += delta_lin_vel
    #     self._last_ang_vel += delta_ang_vel
    #     cmd = Twist()
    #     cmd.linear.x = self._last_lin_vel[0]
    #     cmd.linear.y = self._last_lin_vel[1]
    #     cmd.angular.z = self._last_ang_vel

    #     self._vel_cmd = cmd
    #     self._last_vel_time = time
    #     self._last_cmd_time = time

    # def _proc_vel_cmd(self, stop_wheels):
    #     # Process the most recent velocity command. If stop_wheels is True,
    #     # stop wheel motion.

    #     stop_wheels = stop_wheels or (self._vel_cmd.linear.x == 0 and
    #                                   self._vel_cmd.linear.y == 0 and
    #                                   self._vel_cmd.angular.z == 0)
    #     if not stop_wheels:
    #         self._wheels_stopped = False

    #     if stop_wheels:
    #         # Stop wheel motion.

    #         # Set the steering joint positions to the positions they were at
    #         # when wheel motion was stopped.
    #         if not self._wheels_stopped:
    #             if self._joint_states is not None:
    #                 for i in range(0, 6, 2):
    #                     joint_name = self._joint_cmd.name[i]
    #                     if joint_name not in self._joint_states.name:
    #                         break
    #                     index = self._joint_states.name.index(joint_name)
    #                     self._joint_cmd.position[i] = \
    #                         self._joint_states.position[index]
    #                 else:
    #                     self._wheels_stopped = True

    #         # Stop axle rotation.
    #         for i in range(1, 6, 2):
    #             self._joint_cmd.velocity[i] = 0.0
    #     elif ((self._vel_cmd.linear.x != 0 or self._vel_cmd.linear.y != 0) and
    #           self._vel_cmd.angular.z == 0):
    #         # The linear velocity is nonzero and the angular velocity is zero.

    #         vel_vec = numpy.array((self._vel_cmd.linear.x,
    #                                self._vel_cmd.linear.y))
    #         lin_vel = math.sqrt(numpy.dot(vel_vec, vel_vec))  # Linear velocity
    #         vel_vec_normed = vel_vec / lin_vel
    #         lin_vel = min(lin_vel, self._MAX_LIN_VEL)

    #         # Point the wheels in the same direction. theta_desired is the
    #         # desired steering angle.
    #         theta_desired = \
    #             math.copysign(math.acos(numpy.dot(vel_vec_normed,
    #                                               self._X_DIR)),
    #                           vel_vec_normed[1])
    #         vel_gains = []
    #         for i in range(0, 6, 2):
    #             joint_name = self._joint_cmd.name[i]
    #             theta, vel_gain = self._comp_theta(joint_name, theta_desired)
    #             vel_gains.append(vel_gain)
    #             self._joint_cmd.position[i] = theta

    #         # Specify the angular velocities of the axles.
    #         ang_vel = (lin_vel / self._WHEEL_CIRC) * (2 * pi)
    #         i_2 = 0
    #         for i in range(1, 6, 2):
    #             self._joint_cmd.velocity[i] = vel_gains[i_2] * ang_vel
    #             i_2 += 1
    #     elif (self._vel_cmd.linear.x == 0 and self._vel_cmd.linear.y == 0 and
    #           self._vel_cmd.angular.z != 0):
    #         # The linear velocity is zero and the angular velocity is nonzero.

    #         # Align the wheels so that they are tangent to circles centered
    #         # at the origin of the base frame.
    #         radii = []
    #         vel_gains = []
    #         i_2 = 0
    #         for i in range(0, 6, 2):
    #             pos = self._wheel_mod_positions[i_2]
    #             radius = math.sqrt(numpy.dot(pos, pos))
    #             radii.append(radius)
    #             pos_normed = pos / radius
    #             theta_desired = \
    #                 math.copysign(math.acos(numpy.dot(pos_normed,
    #                                                   self._X_DIR)),
    #                               pos_normed[1]) + (pi / 2)
    #             joint_name = self._joint_cmd.name[i]
    #             theta, vel_gain = self._comp_theta(joint_name, theta_desired)
    #             vel_gains.append(vel_gain)
    #             self._joint_cmd.position[i] = theta
    #             i_2 += 1

    #         # Specify the angular velocities of the axles.
    #         max_lin_speed = 0.0  # Maximum linear speed
    #         i_2 = 0
    #         for i in range(1, 6, 2):
    #             lin_vel = self._vel_cmd.angular.z * radii[i_2]
    #             lin_speed = abs(lin_vel)
    #             if lin_speed > max_lin_speed:
    #                 max_lin_speed = lin_speed
    #             ang_vel = (lin_vel / self._WHEEL_CIRC) * 2 * pi
    #             self._joint_cmd.velocity[i] = vel_gains[i_2] * ang_vel
    #             i_2 += 1
    #         if max_lin_speed > self._MAX_LIN_VEL:
    #             gain = self._MAX_LIN_VEL / max_lin_speed
    #             for i in range(3, 6):
    #                 self._joint_cmd.velocity[i] *= gain
    #     else:
    #         # The linear and angular velocities are nonzero.

    #         vel_vec = numpy.array((self._vel_cmd.linear.x,
    #                                self._vel_cmd.linear.y))
    #         lin_vel = math.sqrt(numpy.dot(vel_vec, vel_vec))  # Linear velocity
    #         vel_vec_normed = vel_vec / lin_vel
    #         lin_vel = min(lin_vel, self._MAX_LIN_VEL)

    #         try:
    #             # This code should not be executed when
    #             # self._vel_cmd.angular.z is zero, but a division by zero
    #             # exception can occur here. Perhaps a sufficiently small z
    #             # value can cause the exception.
    #             radius = lin_vel / self._vel_cmd.angular.z
    #         except:
    #             radius = 1000000000.0
    #         ortho_vec = numpy.array((-vel_vec_normed[1], vel_vec_normed[0]))
    #         center = radius * ortho_vec

    #         # Align the wheels so that they are tangent to circles centered
    #         # at "center".
    #         radii = []
    #         vel_gains = []
    #         i_2 = 0
    #         for i in range(0, 6, 2):
    #             pos = self._wheel_mod_positions[i_2]
    #             vec = pos - center
    #             radius = math.sqrt(numpy.dot(vec, vec))
    #             radii.append(radius)
    #             vec_normed = vec / radius
    #             theta_desired = \
    #                 math.copysign(math.acos(numpy.dot(vec_normed,
    #                                                   self._X_DIR)),
    #                               vec_normed[1]) + (pi / 2)
    #             joint_name = self._joint_cmd.name[i]
    #             theta, vel_gain = self._comp_theta(joint_name, theta_desired)
    #             vel_gains.append(vel_gain)
    #             self._joint_cmd.position[i] = theta
    #             i_2 += 1

    #         # Specify the angular velocities of the axles.
    #         max_lin_speed = 0.0  # Maximum linear speed
    #         i_2 = 0
    #         for i in range(1, 6, 2):
    #             lin_vel = self._vel_cmd.angular.z * radii[i_2]
    #             lin_speed = abs(lin_vel)
    #             if lin_speed > max_lin_speed:
    #                 max_lin_speed = lin_speed
    #             ang_vel = (lin_vel / self._WHEEL_CIRC) * (2 * pi)
    #             self._joint_cmd.velocity[i] = vel_gains[i_2] * ang_vel
    #             i_2 += 1
    #         if max_lin_speed > self._MAX_LIN_VEL:
    #             gain = self._MAX_LIN_VEL / max_lin_speed
    #             for i in range(3, 6):
    #                 self._joint_cmd.velocity[i] *= gain

    #     self._joint_cmd.header.stamp = rospy.get_rostime()
    #     self._joint_cmd_pub.publish(self._joint_cmd)
    # # end _proc_vel_cmd()

    def _ctrl_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
        # Control the steering joints.

        # Compute theta, the virtual front wheel's steering angle.
        if steer_ang_vel_limit > 0.0:
            # Limit the steering velocity.
            ang_vel = (steer_ang - self._last_steer_ang) / delta_t
            ang_vel = max(-steer_ang_vel_limit,
                           min(ang_vel, steer_ang_vel_limit))
            theta = self._last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang

        # Compute the steering angles for the left and right front wheels.
        steer_ang_changed = theta != self._last_steer_ang
        x = self._wheelbase * math.tan((pi / 2) - theta)
        if steer_ang_changed:
            self._last_steer_ang = theta

            # Left wheel
            phi = math.atan(self._inv_wheelbase * (x - self._joint_dist_div_2))
            if phi >= 0.0:
                self._theta_left = (pi / 2) - phi
            else:
                self._theta_left = (-pi / 2) - phi
            # Right wheel
            phi = math.atan(self._inv_wheelbase * (x + self._joint_dist_div_2))
            if phi >= 0.0:
                self._theta_right = (pi / 2) - phi
            else:
                self._theta_right = (-pi / 2) - phi

        self._left_steer_cmd_pub.publish(self._theta_left)
        self._right_steer_cmd_pub.publish(self._theta_right)
        return steer_ang_changed, x

    def _ctrl_axles(self, speed, accel_limit, jerk_limit, delta_t,
                    steer_ang_changed, x):
        # Control the axle joints.

        # Compute veh_speed, the vehicle's speed.
        if accel_limit > 0.0:
            # Limit the vehicle's acceleration.

            if jerk_limit > 0.0:
                if self._last_accel_limit > 0.0:
                    jerk = (accel_limit - self._last_accel_limit) / delta_t
                    jerk = max(-jerk_limit, min(jerk, jerk_limit))
                    accel_limit_2 = self._last_accel_limit + jerk * delta_t
                else:
                    accel_limit_2 = accel_limit
            else:
                accel_limit_2 = accel_limit
            self._last_accel_limit = accel_limit_2

            accel = (speed - self._last_speed) / delta_t
            accel = max(-accel_limit_2, min(accel, accel_limit_2))
            veh_speed = self._last_speed + accel * delta_t
        else:
            self._last_accel_limit = accel_limit
            veh_speed = speed

        # Compute the angular velocities of the wheels.
        if veh_speed != self._last_speed or steer_ang_changed:
            self._last_speed = veh_speed

            # Left front wheel
            r = math.sqrt(pow(x - self._joint_dist_div_2, 2) +
                          self._wheelbase_sqr)
            wheel_speed = r * veh_speed / abs(x)
            self._left_front_ang_vel = \
                (2 * pi) * wheel_speed / self._left_front_circ
            # Right front wheel
            r = math.sqrt(pow(x + self._joint_dist_div_2, 2) +
                          self._wheelbase_sqr)
            wheel_speed = r * veh_speed / abs(x)
            self._right_front_ang_vel = \
                (2 * pi) * wheel_speed / self._right_front_circ
            # Left rear wheel
            wheel_speed = (x - self._joint_dist_div_2) * veh_speed / x
            self._left_rear_ang_vel = \
                (2 * pi) * wheel_speed / self._left_rear_circ
            # Right rear wheel
            wheel_speed = (x + self._joint_dist_div_2) * veh_speed / x
            self._right_rear_ang_vel = \
                (2 * pi) * wheel_speed / self._right_rear_circ

        self._left_front_axle_cmd_pub.publish(self._left_front_ang_vel)
        self._right_front_axle_cmd_pub.publish(self._right_front_ang_vel)
        self._left_rear_axle_cmd_pub.publish(self._left_rear_ang_vel)
        self._right_rear_axle_cmd_pub.publish(self._right_rear_ang_vel)

    def _ctrl_shocks(self):
        # Control the shock absorbers. TODO: It should not be necessary to
        # do this in the main loop.
        for i in range(len(self._shock_cmd_pubs)):
            self._shock_cmd_pubs[i].publish(self._eq_positions[i])

    _DEF_LEFT_STEER_CTRLR_NAME = "left_steering_controller"
    _DEF_LEFT_FRONT_STEER_JOINT_NAME = "left_steering_joint"
    _DEF_RIGHT_STEER_CTRLR_NAME = "right_steering_controller"
    _DEF_RIGHT_FRONT_STEER_JOINT_NAME = "right_steering_joint"

    _DEF_LEFT_FRONT_AXLE_CTRLR_NAME = "left_front_axle_controller"
    _DEF_RIGHT_FRONT_AXLE_CTRLR_NAME = "right_front_axle_controller"
    _DEF_LEFT_REAR_AXLE_CTRLR_NAME = "left_rear_axle_controller"
    _DEF_LEFT_REAR_AXLE_JOINT_NAME = "left_rear_axle_joint"
    _DEF_RIGHT_REAR_AXLE_CTRLR_NAME = "right_rear_axle_controller"
    _DEF_RIGHT_REAR_AXLE_JOINT_NAME = "right_rear_axle_joint"

    _DEF_CMD_TIMEOUT = 0.5  # Default command timeout. Unit: second.
    _DEF_PUB_FREQ = 30.0    # Default publishing frequency. Unit: hertz.
# end _AckermannCtrlr


def _get_link_pos(tf_listener, link):
    # Return the xy position of the specified link, relative to base_link.

    while True:
        try:
            trans, not_used = \
                tf_listener.lookupTransform("base_link", link, rospy.Time(0))
            return (trans[0], trans[1])
        except:
            pass


# main
if __name__ == "__main__":
    ctrlr = _AckermannCtrlr()
    ctrlr.spin()
