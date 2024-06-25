#!/usr/bin/env python
# car_states.py
"""Print out current states of the car.

We obtain: x, y, theta, speed, delta
"""
######################
# Imports & Globals
######################

from autopsy.core import Core, ROS_VERSION
from autopsy.node import Node

import math
import sys
import argparse

# ROS Messages
from nav_msgs.msg import Odometry
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped
if ROS_VERSION == 1:
    from teensy.msg import drive_values
else:
    from teensy_drive_msgs.msg import DriveValues as drive_values


# Arguments
PARSER = argparse.ArgumentParser(
    prog = "car_states.py",
    formatter_class = argparse.RawDescriptionHelpFormatter,
    description = """
Utility for displaying current car states.
""",
)

PARSER.add_argument(
    "-p",

    dest = "p_output",
    help = "Format states in a csv format",
    action = "store_true",
)

PARSER.add_argument(
    "-s",

    dest = "save",
    help = "Save the states into 'states.log' file",
    action = "store_true",
)


# Constants
STEER_LEFT_MIN = 8922.
STEER_LEFT_MAX = 5912.
STEER_RIGHT_MIN = 9122.
STEER_RIGHT_MAX = 12113.
STEER_LEFT_DEG = 29.106255993
STEER_RIGHT_DEG = 27.330086828


######################
# Utilities
######################

def quaternion_to_yaw(q):
    """Convert Quaternion to yaw angle.

    Notes
    -----
    Je to same jako: Ale pouzita verze je obecnejsi.
    np.arctan2(trajectory_file["orientation.z"],trajectory_file["orientation.w"])*2
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


######################
# Node
######################

class StateNode(Node):
    """ROS node that observes and prints states of the car."""

    def __init__(self, p_output = False, save = False):
        """Initialize node."""
        super(StateNode, self).__init__("state_watcher")


        self.x = 0
        self.y = 0
        self.d = 0
        self.theta = 0
        self.v = 0
        self.dv = 0
        self.delta = 0
        self.voltage = 0

        self.STATES = 8


        self.last_pwm_high = 0
        self.plan_v = 0
        self.plan_x = 0
        self.plan_y = 0


        self.sub_odom = self.Subscriber("/odom", Odometry, self.callback_odom,
                                        queue_size = 1, tcp_nodelay = True)
        self.sub_vesc = self.Subscriber("/sensors/core", VescStateStamped,
                                        self.callback_vesc,
                                        queue_size = 1, tcp_nodelay = True)
        self.sub_pwm = self.Subscriber("/drive_pwm", drive_values,
                                       self.callback_pwm,
                                       queue_size = 1, tcp_nodelay = True)
        self.sub_speed = self.Subscriber("/commands/motor/speed", Float64,
                                         self.callback_speed,
                                         queue_size = 1, tcp_nodelay = True)
        self.sub_curp = self.Subscriber("/trajectory/current_point",
                                        PointStamped, self.callback_curp,
                                        queue_size = 1, tcp_nodelay = True)

        if not p_output:
            sys.stdout.write("\n" * self.STATES)
            self.print_data = self.Timer(0.02, self.callback_timer)
        else:
            print("x_m,y_m,d_m,theta_rad,v_mps,dv_mps,delta_rad,voltage_v")
            self.print_data = self.Timer(0.02, self.callback_timer_p)

        self.timestamp = self.get_time()

        if save:
            try:
                open("states.log", "r")
                self.logwarn("Appending to an already exiting states file.")
            except IOError:
                self.loginfo("Creating file 'states.log'")
                with open("states.log", "w") as f:
                    f.write("t_s,x_m,y_m,v_mps\n")

            self.log_data = self.Timer(0.1, self.callback_log_data)


    def callback_odom(self, data):
        """Obtain data from Odometry."""
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.theta = quaternion_to_yaw(data.pose.pose.orientation)


    def callback_vesc(self, data):
        """Obtain data from VESC."""
        self.v = data.state.speed / 4105.324277107
        self.voltage = data.state.voltage_input


    def callback_speed(self, data):
        """Obtain data that is sent to VESC."""
        self.plan_v = data.data / 4105.324277107


    def callback_pwm(self, data):
        """Obtain data that are sent to servo."""
        self.obtain_delta(data.pwm_angle)


    def callback_pwm_high(self, data):
        """Obtain data that are sent to servo from the remote."""
        if self.last_pwm_high == data.period_str:
            return
        else:
            self.last_pwm_high = data.period_str
            self.obtain_delta(data.period_str)


    def callback_curp(self, data):
        """Obtain current point from plan."""
        self.plan_x = data.point.x
        self.plan_y = data.point.y

        self.d = math.sqrt((self.plan_x - self.x)**2 + (self.plan_y - self.y)**2)


    def obtain_delta(self, pwm):
        """Common function for obtaining steering angle.

        FIXME: Yes, it is hardcoded.
        """
        if STEER_LEFT_MIN >= pwm >= STEER_LEFT_MAX:
            self.delta = math.radians(((STEER_LEFT_MIN - pwm) / (STEER_LEFT_MIN - STEER_LEFT_MAX)) * STEER_LEFT_DEG)
        elif STEER_RIGHT_MAX >= pwm >= STEER_RIGHT_MIN:
            self.delta = -math.radians(((pwm - STEER_RIGHT_MIN) / (STEER_RIGHT_MAX - STEER_RIGHT_MIN)) * STEER_RIGHT_DEG)
        else:
            self.delta = 0.0


    def callback_timer(self, *args, **kwargs):
        """Print out current states."""
        sys.stdout.write("\033[%dA" % self.STATES)
        sys.stdout.write("\033[J")
        print("x:\t%s" % str(self.x))
        print("y:\t%s" % str(self.y))
        print("d:\t%+.6f m" % self.d)
        print("theta:\t%+.6f    \t(%+.2f deg)" % (self.theta, math.degrees(self.theta)))
        print("v:\t%+.6f m/s\t(%+.2f km/h)" % (self.v, self.v * 3.6))
        print("dv:\t%+.6f m/s\t(%+.2f km/h)" % (self.v - self.plan_v, (self.v - self.plan_v) * 3.6))
        print("delta:\t%+.6f    \t(%+.2f deg)" % (self.delta, math.degrees(self.delta)))
        print("voltage:\t%2.1f V" % (self.voltage))


    def callback_timer_p(self, *args, **kwargs):
        """Print out current states in -p format."""
        print(",".join([
            "%s" % str(self.x),
            "%s" % str(self.y),
            "%.6f" % self.d,
            "%.6f" % self.theta,
            "%.6f" % self.v,
            "%.6f" % (self.v - self.plan_v),
            "%.6f" % self.delta,
            "%2.1f" % self.voltage,
        ]))


    def callback_log_data(self, *args, **kwargs):
        """Log data."""
        with open("states.log", "a+") as f:
            f.write(
                "%f,%f,%f,%f\n"
                % (
                    (self.get_time() - self.timestamp), self.x, self.y, self.v
                )
            )


######################
# Main
######################

## TODO:
## TODO: Publikovat rozdil rychlosti (aktualni x plan)
## TODO: Publikovat rozdil polohy (aktualni x reference)

if __name__ == '__main__':
    args, others = PARSER.parse_known_args()

    Core.init(args = args)

    n = StateNode(
        p_output = args.p_output,
        save = args.save,
    )

    Core.spin(n)

    Core.shutdown()
