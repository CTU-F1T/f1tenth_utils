#!/usr/bin/env python
# simple_drive.py
"""Node for simply driving the car.

We bypass DAPI and directly control the servo (via Teensy)
and motor (via VESC).
"""
######################
# Imports & Globals
######################

from autopsy.node import Node
from autopsy.reconfigure import ParameterServer


# Message types
try:
    from teensy_drive_msgs.msg import DriveValues
except ImportError:
    from f1tenth_race.msg import drive_values as DriveValues

from std_msgs.msg import Float64


# Global variables
PARAMETERS = [
    ("steer_min", {
        "default": 6000, "min": 0, "max": 13000,
        "description": "[-] Minimum value of the PWM."
    }),
    ("steer", {
        "default": 9000, "min": 0, "max": 13000,
        "description": "[-] Requested steer PWM value."
    }),
    ("steer_max", {
        "default": 12000, "min": 0, "max": 13000,
        "description": "[-] Maximum value of the PWM."
    }),

    ("throttle_calm", {
        "default": 9000, "min": 0, "max": 13000,
        "description": "[-] Calm value of the PWM for throttle."
    }),

    ("vesc_speed", {
        "default": 0.0, "min": 0.0, "max": 10.0,
        "description": "[m.s^-1] Car speed."
    }),

    ("publish_rate", {
        "default": 10.0, "min": 0.1, "max": 100.0,
        "description": "[s^-1] Frequency of publishing the action values."
    }),
]


######################
# Node
######################

class SimpleDrive(Node):
    """SimpleDrive node."""

    def __init__(self):
        """Initialize the node."""
        super(SimpleDrive, self).__init__("simple_drive")

        self.loginfo("Starting the node...")

        # Parameters
        self.P = ParameterServer()
        self.P.update(PARAMETERS)

        self.P.link(self.P.steer_min, self.P.steer)
        self.P.link(self.P.steer, self.P.steer_max)

        self.P.reconfigure(node = self)

        # Publishers
        self.pub_teensy = self.create_publisher(
            DriveValues, "/drive_pwm", 1
        )
        self.pub_vesc = self.create_publisher(
            Float64, "/commands/motor/speed", 1
        )

        # Timers
        self.timer = self.create_timer(
            1.0 / self.P.publish_rate, self.publish_loop
        )
        self.loginfo(
            "Starting the Timer with frequency %g Hz" % self.P.publish_rate
        )


    def publish_loop(self, *args, **kwargs):
        """Periodically publish the action values."""
        self.pub_teensy.publish(
            DriveValues(
                pwm_drive = self.P.throttle_calm,
                pwm_angle = self.P.steer
            )
        )

        self.pub_vesc.publish(
            Float64(
                data = self.P.vesc_speed
            )
        )


######################
# Main
######################

if __name__ == "__main__":
    n = SimpleDrive()
    n.spin()
    n.timer.shutdown()
