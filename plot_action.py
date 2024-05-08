#!/usr/bin/env python
# plot_action.py
"""Plot action values from mt from 'follow_trajectory'.

It obtains data as a string '%f,%f,...', each one corresponds to its own
data line.

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
from std_msgs.msg import String

# Global variables
figure = None
line1 = None
line2 = None
line3 = None
line4 = None
x1_data, y1_data = [], []
x2_data, y2_data = [], []
x3_data, y3_data = [], []
x4_data, y4_data = [], []


######################
# ROS related
######################

def update_data(data):
    """Obtain data from the topic. Process and store received data."""

    d1, d2, d3 = [float(_d) for _d in data.data.split(",")]

    # Append plot
    length = len(x1_data)

    x1_data.append(length)
    x2_data.append(length)
    x3_data.append(length)
    x4_data.append(length)

    y1_data.append(d1)
    y2_data.append(d2)
    y3_data.append(d3)
    y4_data.append(d1+d2+d3)


class Updater(Thread):
    """Node wrapper that runs the node in a separate thread.

    Source:
    https://stackoverflow.com/questions/48389470/python-update-matplotlib-from-threads
    """

    def __init__(self):
        """Initialize the node."""
        super(Updater, self).__init__()

        self.n = Node("plot_action")

        self.n.Subscriber(
            "/trajectory/action/mt",
            String, update_data, queue_size = 1
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
    line1.set_data(x1_data, y1_data)
    line2.set_data(x2_data, y2_data)
    line3.set_data(x3_data, y3_data)
    line4.set_data(x4_data, y4_data)
    figure.gca().relim()
    figure.gca().autoscale_view()
    return line1,


if __name__ == "__main__":
    # Initialize ROS
    Core.init(args = sys.argv)

    # Initialize matplotlib
    figure = pyplot.figure()
    line1, = pyplot.plot(x1_data, y1_data, '-')
    line2, = pyplot.plot(x2_data, y2_data, '-')
    line3, = pyplot.plot(x3_data, y3_data, '-')
    line4, = pyplot.plot(x4_data, y4_data, '-')

    animation = FuncAnimation(figure, update, interval=200)

    # Run node
    U = Updater()
    U.start()

    pyplot.show()  # This blocks the execution.

    Core.shutdown()
