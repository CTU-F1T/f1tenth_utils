#!/usr/bin/env python
# plot_traj_gg.py
"""Plot g-g diagram from Trajectory data.

Based on 'plot_error.py' and 'plot_gg.py'.

Source:
https://stackoverflow.com/questions/11874767/how-do-i-plot-in-real-time-in-a-while-loop
"""
######################
# Imports & Globals
######################

import math
import sys

from autopsy.node import Node
from autopsy.core import Core

from threading import Thread

from matplotlib import pyplot
from matplotlib.animation import FuncAnimation

# Message types
from autoware_auto_msgs.msg import Trajectory

# Global variables
G = 9.81
MU = 0.25
RO = 1.2
A = 0.3
CL = 1.0
L = 0.33
M = 3.68
VLIM = 4.5

figure = None
line = None
x_data, y_data = [], []


######################
# ROS related
######################

def update_data(data):
    """Obtain data from the topic. Process and store received data."""
    global x_data, y_data

    x_data, y_data = [], []

    for tpoint in data.points:
        _v = math.sqrt(
            tpoint.longitudinal_velocity_mps**2
            + tpoint.lateral_velocity_mps**2
        )

        _k = math.tan(tpoint.front_wheel_angle_rad) / L

        _Fz = M * G + 0.5 * RO * A * CL * _v**2
        _Fy = M * _k * _v**2

        _ax = math.sqrt(
            _Fz**2 * MU**2 - _Fy**2
        ) / M

        x_data.append(_Fy / M)
        y_data.append(_ax)


class Updater(Thread):
    """Node wrapper that runs the node in a separate thread.

    Source:
    https://stackoverflow.com/questions/48389470/python-update-matplotlib-from-threads
    """

    def __init__(self):
        """Initialize the node."""
        super(Updater, self).__init__()

        self.n = Node("plot_traj_gg")

        self.n.Subscriber(
            "/trajectory",
            Trajectory, update_data, queue_size = 1
        )


    def run(self):
        """Overloaded threading.Thread function."""
        Core.spin(self.n)


######################
# Plotting
######################

""" # Continuous can be done in this way:
x_data, y_data = [], []

figure = pyplot.figure()
line, = pyplot.plot([], [], 'x')

def update(frame):
    line.set_data(x_data, y_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    figure.gca().set_title("avg: %f" % numpy.mean(numpy.abs(y_data)))
    return line,
"""


def update(frame):
    """Perform a pyplot update using data from the node."""
    line.set_data(x_data, y_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    return line,


if __name__ == "__main__":
    # Initialize ROS
    Core.init(args = sys.argv)

    # Initialize matplotlib
    figure = pyplot.figure()
    line, = pyplot.plot(x_data, y_data, '-')
    limit = pyplot.Circle(
        xy = (0, 0),
        radius = (MU * G),  # x**2 + y**2 <= R**2
        ls = "--", facecolor = 'r', edgecolor = 'k', alpha = 0.25, lw = 2.0
    )
    limit2 = pyplot.Circle(
        xy = (0, 0),
        radius = MU * (G + 0.5 * RO * A * CL * VLIM**2 / M),
        ls = "--", facecolor = 'g', edgecolor = 'k', alpha = 0.25, lw = 2.0
    )
    figure.gca().add_patch(limit)
    figure.gca().add_patch(limit2)
    figure.gca().set_aspect("equal")
    figure.gca().set_title("g-g diagram")
    figure.gca().set_xlabel("lateral acceleration [m.s^{-2}]")
    figure.gca().set_ylabel("longitudinal acceleration [m.s^{-2}]")

    animation = FuncAnimation(figure, update, interval=200)

    # Run node
    U = Updater()
    U.start()

    pyplot.show()  # This blocks the execution.

    Core.shutdown()
