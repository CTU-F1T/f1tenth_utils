#!/usr/bin/env python
# plot_gg.py
"""Plot g-g diagram from Imu data.

Based on 'plot_error.py'.

Source:
https://stackoverflow.com/questions/11874767/how-do-i-plot-in-real-time-in-a-while-loop
"""
######################
# Imports & Globals
######################

import sys

from autopsy.node import Node
from autopsy.core import Core

from threading import Thread

import numpy

from matplotlib import pyplot
from matplotlib.animation import FuncAnimation

# Message types
from sensor_msgs.msg import Imu

# Global variables
last_int_x = None
path_length = 0

figure = None
line = None
x_data, y_data = [], []


######################
# ROS related
######################

def update_data(data):
    """Obtain data from the topic. Process and store received data."""
    global last_int_x, path_length

    x, y = data.linear_acceleration.x, data.linear_acceleration.y

    # To plot, just add the values to the arrays.
    x_data.append(x / 9.81)
    y_data.append(y / 9.81)


class Updater(Thread):
    """Node wrapper that runs the node in a separate thread.

    Source:
    https://stackoverflow.com/questions/48389470/python-update-matplotlib-from-threads
    """

    def __init__(self):
        """Initialize the node."""
        super(Updater, self).__init__()

        self.n = Node("plot_error")

        self.n.Subscriber(
            "/imu",
            Imu, update_data, queue_size = 1
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
    figure.gca().set_title("g-g diagram")
    figure.gca().set_xlabel("g_x")
    figure.gca().set_ylabel("g_y")

    animation = FuncAnimation(figure, update, interval=200)

    # Run node
    U = Updater()
    U.start()

    pyplot.show()  # This blocks the execution.

    Core.shutdown()
