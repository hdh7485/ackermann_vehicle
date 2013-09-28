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
import numpy
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

        # Wheels
        (left_steer_link_name, left_steer_ctrlr_name,
         left_front_axle_ctrlr_name, self._left_front_circ) = \
         self._get_front_wheel_params("left")
        (right_steer_link_name, right_steer_ctrlr_name,
         right_front_axle_ctrlr_name, self._right_front_circ) = \
         self._get_front_wheel_params("right")
        (left_rear_wheel_link_name, left_rear_axle_ctrlr_name,
         self._left_rear_circ) = \
         self._get_rear_wheel_params("left")
        (self._right_rear_wheel_link_name, right_rear_axle_ctrlr_name,
         self._right_rear_circ) = \
         self._get_rear_wheel_params("right")

        # Shock absorbers
        shock_param_list = rospy.get_param("~shock_absorbers", [])
        self._shock_absorbers = []
        try:
            for shock_params in shock_param_list:
                try:
                    ctrlr_name = shock_params["controller_name"]
                    eq_pos = float(shock_params["equilibrium_position"])
                except:
                    rospy.logwarn("An invalid parameter was specified for a "
                                  "shock absorber. The shock absorber will "
                                  "not be used.")
                    continue
                shock = self._ShockAbsorber(ctrlr_name, eq_pos)
                self._shock_absorbers.append(shock)
        except:
            rospy.logwarn("The specified list of shock absorbers is invalid. "
                          "No shock absorbers will be used.")

        # Command timeout
        try:
            self._cmd_timeout = float(rospy.get_param("~cmd_timeout",
                                                      self._DEF_CMD_TIMEOUT))
        except:
            rospy.logwarn("The specified command timeout value is invalid. "
                          "The default timeout value will be used instead.")
            self._cmd_timeout = self._DEF_CMD_TIMEOUT

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

        # _last_cmd_time is the time at which the most recent Ackermann
        # driving command was received.
        self._last_cmd_time = rospy.get_time()

        # _ackermann_cmd_lock is used to control access to _steer_ang,
        # _steer_ang_vel, _speed, _accel, and _jerk.
        self._ackermann_cmd_lock = threading.Lock()
        self._steer_ang = 0.0      # Steering angle
        self._steer_ang_vel = 0.0  # Steering angle velocity
        self._speed = 0.0
        self._accel = 0.0          # Acceleration
        self._jerk = 0.0

        self._last_steer_ang = 0.0  # Last steering angle
        self._theta_left = 0.0      # Left steering joint angle
        self._theta_right = 0.0     # Right steering joint angle

        self._last_speed = 0.0
        self._last_accel_limit = 0.0  # Last acceleration limit
        # Axle angular velocities
        self._left_front_ang_vel = 0.0
        self._right_front_ang_vel = 0.0
        self._left_rear_ang_vel = 0.0
        self._right_rear_ang_vel = 0.0

        # _joint_dist_div_2 is the distance between the steering joints,
        # divided by two.
        tfl = tf.TransformListener()
        ls_pos = self._get_link_pos(tfl, left_steer_link_name)
        rs_pos = self._get_link_pos(tfl, right_steer_link_name)
        self._joint_dist_div_2 = numpy.linalg.norm(ls_pos - rs_pos) / 2
        lrw_pos = self._get_link_pos(tfl, left_rear_wheel_link_name)
        rrw_pos = numpy.array([0.0] * 3)
        front_cent_pos = (ls_pos + rs_pos) / 2     # Front center position
        rear_cent_pos = (lrw_pos + rrw_pos) / 2    # Rear center position
        self._wheelbase = numpy.linalg.norm(front_cent_pos - rear_cent_pos)
        self._inv_wheelbase = 1 / self._wheelbase  # Inverse of _wheelbase
        self._wheelbase_sqr = pow(self._wheelbase, 2)

        # Subscribers and publishers

        self._ackermann_cmd_sub = \
            rospy.Subscriber("ackermann_cmd", AckermannDriveStamped,
                             self.ackermann_cmd_cb, queue_size=1)

        self._left_steer_cmd_pub = _create_cmd_pub(left_steer_ctrlr_name)
        self._right_steer_cmd_pub = _create_cmd_pub(right_steer_ctrlr_name)

        self._left_front_axle_cmd_pub = \
            _create_axle_cmd_pub(left_front_axle_ctrlr_name)
        self._right_front_axle_cmd_pub = \
            _create_axle_cmd_pub(right_front_axle_ctrlr_name)
        self._left_rear_axle_cmd_pub = \
            _create_axle_cmd_pub(left_rear_axle_ctrlr_name)
        self._right_rear_axle_cmd_pub = \
            _create_axle_cmd_pub(right_rear_axle_ctrlr_name)

    def spin(self):
        """Control the vehicle."""

        last_time = rospy.get_time()
        last_shock_cmd_time = last_time

        while not rospy.is_shutdown():
            t = rospy.get_time()
            delta_t = t - last_time
            last_time = t

            if (self._cmd_timeout > 0.0 and
                t - self._last_cmd_time > self._cmd_timeout):
                # Too much time has elapsed since the last command. Stop the
                # vehicle.
                steer_ang_changed, center_y = \
                    self._ctrl_steering(self._last_steer_ang, 0.0, 0.001)
                self._ctrl_axles(0.0, 0.0, 0.0, 0.001, steer_ang_changed,
                                 center_y)
            elif delta_t > 0.0:
                with self._ackermann_cmd_lock:
                    steer_ang = self._steer_ang
                    steer_ang_vel = self._steer_ang_vel
                    speed = self._speed
                    accel = self._accel
                    jerk = self._jerk
                steer_ang_changed, center_y = \
                    self._ctrl_steering(steer_ang, steer_ang_vel, delta_t)
                self._ctrl_axles(speed, accel, jerk, delta_t,
                                 steer_ang_changed, center_y)

            # Publish the steering and axle joint commands.
            self._left_steer_cmd_pub.publish(self._theta_left)
            self._right_steer_cmd_pub.publish(self._theta_right)
            if self._left_front_axle_cmd_pub:
                self._left_front_axle_cmd_pub.publish(self._left_front_ang_vel)
            if self._right_front_axle_cmd_pub:
                self._right_front_axle_cmd_pub.\
                    publish(self._right_front_ang_vel)
            if self._left_rear_axle_cmd_pub:
                self._left_rear_axle_cmd_pub.publish(self._left_rear_ang_vel)
            if self._right_rear_axle_cmd_pub:
                self._right_rear_axle_cmd_pub.publish(self._right_rear_ang_vel)

            # Even though each shock absorber command publisher is latched,
            # commands can be lost if they are published too soon after
            # the publisher is created. Therefore, they are published here
            # periodically.
            if t - last_shock_cmd_time >= self._SHOCK_CMD_PERIOD:
                last_shock_cmd_time = t
                for shock in self._shock_absorbers:
                    shock.publish_cmd()
            self._sleep_timer.sleep()

    def ackermann_cmd_cb(self, ackermann_cmd):
        """Ackermann driving command callback

        :Parameters:
          ackermann_cmd: ackermann_msgs.msg.AckermannDriveStamped
            Ackermann driving command.
        """
        self._last_cmd_time = rospy.get_time()
        with self._ackermann_cmd_lock:
            self._steer_ang = ackermann_cmd.drive.steering_angle
            self._steer_ang_vel = ackermann_cmd.drive.steering_angle_velocity
            self._speed = ackermann_cmd.drive.speed
            self._accel = ackermann_cmd.drive.acceleration
            self._jerk = ackermann_cmd.drive.jerk

    class _ShockAbsorber(object):
        def __init__(self, ctrlr_name, eq_position):
            self._cmd_pub = rospy.Publisher(ctrlr_name + "/command", Float64,
                                            latch=True)
            self._eq_pos = eq_position  # Equilibrium position

        def publish_cmd(self):
            self._cmd_pub.publish(self._eq_pos)

    def _get_front_wheel_params(self, side):
        # Get front wheel parameters. Return a tuple containing the steering
        # link name, steering controller name, axle controller name (or None),
        # and wheel circumference.

        prefix = "~" + side + "_front_wheel/"
        steer_link_name = rospy.get_param(prefix + "steering_link_name",
                                          side + "_steering_link")
        steer_ctrlr_name = rospy.get_param(prefix + "steering_controller_name",
                                           side + "_steering_controller")
        axle_ctrlr_name, circ = self._get_common_wheel_params(prefix)
        return steer_link_name, steer_ctrlr_name, axle_ctrlr_name, circ

    def _get_rear_wheel_params(self, side):
        # Get rear wheel parameters. Return a tuple containing the wheel link
        # name, axle controller name, and wheel circumference.

        prefix = "~" + side + "_rear_wheel/"
        wheel_link_name = rospy.get_param(prefix + "wheel_link_name",
                                          side + "_wheel")
        axle_ctrlr_name, circ = self._get_common_wheel_params(prefix)
        return wheel_link_name, axle_ctrlr_name, circ

    def _get_common_wheel_params(self, prefix):
        # Get parameters used by the front and rear wheels. Return a tuple
        # containing the axle controller name (or None) and the wheel
        # circumference.

        axle_ctrlr_name = rospy.get_param(prefix + "axle_controller_name",
                                          None)

        try:
            dia = float(rospy.get_param(prefix + "diameter",
                                        self._DEF_WHEEL_DIA))
            if dia <= 0.0:
                raise ValueError()
        except:
            rospy.logwarn("The specified wheel diameter is invalid. "
                          "The default diameter will be used instead.")
            dia = self._DEF_WHEEL_DIA

        return axle_ctrlr_name, pi * dia

    def _get_link_pos(self, tfl, link):
        # Return the position of the specified link, relative to the right
        # rear wheel link.

        while True:
            try:
                trans, not_used = \
                    tfl.lookupTransform(self._right_rear_wheel_link_name,
                                        link, rospy.Time(0))
                return numpy.array(trans)
            except:
                pass

    def _ctrl_steering(self, steer_ang, steer_ang_vel_limit, delta_t):
        # Control the steering joints.

        # Compute theta, the virtual front wheel's desired steering angle.
        if steer_ang_vel_limit > 0.0:
            # Limit the steering velocity.
            ang_vel = (steer_ang - self._last_steer_ang) / delta_t
            ang_vel = max(-steer_ang_vel_limit,
                           min(ang_vel, steer_ang_vel_limit))
            theta = self._last_steer_ang + ang_vel * delta_t
        else:
            theta = steer_ang

        # Compute the desired steering angles for the left and right front
        # wheels.
        center_y = self._wheelbase * math.tan((pi / 2) - theta)
        steer_ang_changed = theta != self._last_steer_ang
        if steer_ang_changed:
            self._last_steer_ang = theta

            # Left wheel
            phi = math.atan(self._inv_wheelbase *
                            (center_y - self._joint_dist_div_2))
            if phi >= 0.0:
                self._theta_left = (pi / 2) - phi
            else:
                self._theta_left = (-pi / 2) - phi
            # Right wheel
            phi = math.atan(self._inv_wheelbase *
                            (center_y + self._joint_dist_div_2))
            if phi >= 0.0:
                self._theta_right = (pi / 2) - phi
            else:
                self._theta_right = (-pi / 2) - phi

        return steer_ang_changed, center_y

    def _ctrl_axles(self, speed, accel_limit, jerk_limit, delta_t,
                    steer_ang_changed, center_y):
        # Control the axle joints.

        # Compute veh_speed, the vehicle's desired speed.
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

        # Compute the desired angular velocities of the wheels.
        if veh_speed != self._last_speed or steer_ang_changed:
            self._last_speed = veh_speed

            # Left front
            r = math.sqrt(pow(center_y - self._joint_dist_div_2, 2) +
                          self._wheelbase_sqr)
            wheel_speed = r * veh_speed / abs(center_y)
            self._left_front_ang_vel = \
                (2 * pi) * wheel_speed / self._left_front_circ
            # Right front
            r = math.sqrt(pow(center_y + self._joint_dist_div_2, 2) +
                          self._wheelbase_sqr)
            wheel_speed = r * veh_speed / abs(center_y)
            self._right_front_ang_vel = \
                (2 * pi) * wheel_speed / self._right_front_circ
            # Left rear
            wheel_speed = \
                (center_y - self._joint_dist_div_2) * veh_speed / center_y
            self._left_rear_ang_vel = \
                (2 * pi) * wheel_speed / self._left_rear_circ
            # Right rear
            wheel_speed = \
                (center_y + self._joint_dist_div_2) * veh_speed / center_y
            self._right_rear_ang_vel = \
                (2 * pi) * wheel_speed / self._right_rear_circ

    _DEF_WHEEL_DIA = 1.0    # Default wheel diameter. Unit: meter.
    _DEF_CMD_TIMEOUT = 0.5  # Default command timeout. Unit: second.
    _DEF_PUB_FREQ = 30.0    # Default publishing frequency. Unit: hertz.

    _SHOCK_CMD_PERIOD = 1.0 # Shock absorber command period. Unit: second.
# end _AckermannCtrlr


def _create_axle_cmd_pub(axle_ctrlr_name):
    # Create an axle command publisher.
    if not axle_ctrlr_name:
        return None
    return _create_cmd_pub(axle_ctrlr_name)


def _create_cmd_pub(ctrlr_name):
    # Create a command publisher.
    return rospy.Publisher(ctrlr_name + "/command", Float64)


# main
if __name__ == "__main__":
    ctrlr = _AckermannCtrlr()
    ctrlr.spin()
