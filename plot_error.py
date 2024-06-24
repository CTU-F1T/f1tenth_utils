#!/usr/bin/env python
# plot_error.py
"""Plot lateral error from 'follow_trajectory'.

It obtains data as a string '%f,%f', where first number is an index to
the trajectory and second number is lateral distance from the trajectory.

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
from std_srvs.srv import Trigger
TriggerRequest = Trigger._request_class
TriggerResponse = Trigger._response_class

# Global variables
last_int_x = None
path_length = 0

figure = None
line = None
x_data, y_data = [], []
y_data_history = {}

INTERPOLATE = True
print ("Running with INTERPOLATE = %s" % str(INTERPOLATE))


######################
# ROS related
######################

def update_data(data):
    """Obtain data from the topic. Process and store received data."""
    global last_int_x, path_length

    x, y = [float(_d) for _d in data.data.split(",")]
    # For scatter
    # x_data.append(x)
    # y_data.append(y)

    int_x = int(x)
    path_length = max(path_length, int_x)


    # For plot
    # a) Add data to the xy arrays
    # a.1) The x-value is already there
    if int_x in x_data:
        y_data[x_data.index(int_x)] = y

    # a.2) It is not there yet. Therefore, we need to find the place where
    #      to put it.
    else:
        i = 0
        for i in range(len(x_data)):
            if int_x < x_data[i]:
                break
        else:
            # If there is no lower value, we need to append it.
            i = i + 1

        # We found first larger value, therefore we add it at that index.
        x_data.insert(i, int_x)
        y_data.insert(i, y)


    # b) We have to clear all old values. Idea is to remove all values that
    #    are in-between of the current and last changed x value.
    if last_int_x is not None:
        if last_int_x < int_x:
            RANGE = range(last_int_x + 1, int_x)
        elif (path_length - last_int_x) + int_x < 50:
            # We have crossed over the starting point.
            RANGE = range(last_int_x + 1, max(x_data) + 1) + range(int_x)
        else:
            # Something nasty happened (e.g., we stutter around the index)
            RANGE = []

        if not INTERPOLATE:
            for value in RANGE:
                if value in x_data:
                    _index = x_data.index(value)
                    x_data.pop(_index)
                    y_data.pop(_index)
        else:  # Do not clear the values, but interpolate the error
            i = x_data.index(last_int_x)
            last_y = y_data[x_data.index(last_int_x)]
            dy = (y - last_y) / (len(RANGE) + 1)

            for j, value in enumerate(RANGE):
                if value == 0:
                    i = 0

                if value in x_data:
                    i = x_data.index(value)
                    y_data[i] = last_y + dy * (1 + j)
                else:
                    x_data.insert(i + 1, value)
                    y_data.insert(i + 1, last_y + dy * (1 + j))
                    i = i + 1

    # Save the value for the next loop.
    last_int_x = int_x

    # Append the value to the history
    if int_x not in y_data_history:
        y_data_history[int_x] = [y]
    else:
        y_data_history[int_x].append(y)


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
            "/trajectory/difference/index_lateral",
            String, update_data, queue_size = 1
        )

        self.n.Service(
            "get_history_mean", Trigger, self.service_history_mean
        )

        self.n.Service(
            "erase_history", Trigger, self.service_erase_history
        )


    def run(self):
        """Overloaded threading.Thread function."""
        Core.spin(self.n)


    def service_history_mean(self, msg, response = None):
        """Callback service request '/get_history_mean'."""  # noqa: D401
        global y_data_history

        if response is not None:
            self.n.logerr(
                "ROS2 is not currently supported here."
            )
            return

        response = TriggerResponse()
        response.success = True
        response.message = ";".join([
            "%d,%f" % (_x, numpy.mean(y_data_history[_x]))
            for _x in sorted(y_data_history)
        ])

        return response


    def service_erase_history(self, msg, response = None):
        """Callback service request '/erase_history'."""  # noqa: D401
        global y_data_history

        if response is not None:
            self.n.logerr(
                "ROS2 is not currently supported here."
            )
            return

        response = TriggerResponse()
        response.success = True
        y_data_history = {}

        self.n.loginfo("History erased.")

        return response


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
    if len(y_data) > 0:
        figure.gca().set_title("MSE: %f" % numpy.mean(numpy.power(y_data, 2)))
    return line,


if __name__ == "__main__":
    # Initialize ROS
    Core.init(args = sys.argv)

    # Initialize matplotlib
    figure = pyplot.figure()
    line, = pyplot.plot(x_data, y_data, '-')

    animation = FuncAnimation(figure, update, interval=200)

    # Run node
    U = Updater()
    U.start()

    pyplot.show()  # This blocks the execution.

    Core.shutdown()
