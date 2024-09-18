#!/usr/bin/env python
# alt_trajectory.py
"""A ROS node for moving the path according to the tracking error.

Originated from 'move_trajectory_by_error.py'. Probably separated into its own
package in the future.
"""
######################
# Imports & Globals
######################

import math

import numpy

from autopsy.node import Node
# Requires autopsy>0.10.7; but may be incompatible with 0.11
from autopsy.helpers import Publisher, Subscriber, Execute, Timer

from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler
)

from functools import partial

from scipy.ndimage import uniform_filter1d


# ROS messages
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from nav_msgs.msg import Path
from autoware_auto_msgs.msg import Trajectory


# Global variables
# Set default values of the decorators
Publisher = partial(Publisher, queue_size = 1)
Subscriber = partial(Subscriber, queue_size = 1)
MAX_ERROR = 2.0
MAX_STEERING = 30  # deg
L = 0.32
K_LIM = numpy.tan(numpy.radians(MAX_STEERING)) / (2 * L)


######################
# PathHandler
######################

class PathHandler(object):
    """An object to handle and edit paths."""

    class Point(object):
        """Object to handle point of a path."""

        def __init__(self, x, y, z, ox, oy, oz, ow):
            """Initialize the path point."""
            self.x = x
            self.y = y
            self.z = z
            self.orig_x = x
            self.orig_y = y
            self.orig_z = z
            self.ox = ox
            self.oy = oy
            self.oz = oz
            self.ow = ow
            self.ex = x
            self.ey = y
            self.ez = z
            self.error = 0.0
            self.last_error = 0.0
            self.last_valid_error = 0.0


        @classmethod
        def from_pq(cls, p, q):
            """Initialize the path point from a point and a quaternion."""
            return cls(p.x, p.y, p.z, q.x, q.y, q.z, q.w)


        @classmethod
        def from_pose(cls, pose):
            """Initialize the path point from a pose."""
            return cls.from_pq(pose.position, pose.orientation)


        @classmethod
        def from_pyaw(cls, p, yaw):
            """Initialize the path point from a point and a yaw angle."""
            return cls(p.x, p.y, p.z, *quaternion_from_euler(0.0, 0.0, yaw))


        @property
        def yaw(self):
            """Get the yaw angle / orientation of the point."""
            return euler_from_quaternion(
                [self.ox, self.oy, self.oz, self.ow]
            )[-1]


        @property
        def yaw_normal(self):
            """Get the normal to the yaw angle / orientation of the point."""
            return self.yaw + math.radians(90)


        # @classmethod
        # def apply_error(cls, self, error):
        #     """Return a point moved by a lateral error."""
        #     return cls.from_pyaw(
        #         self.x + math.cos(self.yaw) * error,
        #         self.y + math.sin(self.yaw) * error,
        #         self.z,
        #         self.yaw
        #     )


        def apply_error(self, error):
            """Apply error to the point."""
            self.ex = self.x + math.cos(self.yaw + math.radians(90)) * error
            self.ey = self.y + math.sin(self.yaw + math.radians(90)) * error
            self.last_error = error
            self.last_valid_error = error


        def distanceTo(self, other):
            """Distance to different point.

            Returns:
            distance to other point, [m], float
            """
            return (
                (other.orig_x - self.orig_x)**2
                + (other.orig_y - self.orig_y)**2
                + (other.orig_z - self.orig_z)**2
            )**0.5


        def save_error(self, left = MAX_ERROR, right = -MAX_ERROR):
            """Save the error by changing the point location."""
            self.error = min(
                max(right, self.error + self.last_error), left
            )
            self.last_error = 0.0

            self.x = self.orig_x + math.cos(self.yaw_normal) * self.error
            self.y = self.orig_y + math.sin(self.yaw_normal) * self.error


        def set_error(self, error):
            """Set the error value and recompute the point location."""
            self.error = error

            self.x = self.orig_x + math.cos(self.yaw_normal) * self.error
            self.y = self.orig_y + math.sin(self.yaw_normal) * self.error


        def to_msg(self):
            """Convert the Point into PoseStamped message."""
            p = PoseStamped()
            p.pose = self.to_pose_msg()
            return p


        def to_pose_msg(self):
            """Convert the Point into Pose message."""
            p = Pose()
            p.position.x = self.ex
            p.position.y = self.ey
            p.orientation.x = self.ox
            p.orientation.y = self.oy
            p.orientation.z = self.oz
            p.orientation.w = self.ow
            return p


    def __init__(self, points = [], frame_id = "map", trajectory = None):
        """Initialize the path handler."""
        super(PathHandler, self).__init__()
        self.points = points
        self.frame_id = frame_id
        self.update(trajectory)

        self._x = None
        self._y = None
        self._yaw = None
        self._dirty = False
        self.last_error = None
        self.last_index = None
        self.error_limits = [0.0 for _ in range(len(points))]
        self.left_error = numpy.ones(len(points)) * MAX_ERROR
        self.right_error = numpy.ones(len(points)) * -MAX_ERROR


    @classmethod
    def from_msg(cls, msg):
        """Form a path from a Path message."""
        return cls(
            [PathHandler.Point.from_pose(pose.pose) for pose in msg.poses],
            frame_id = msg.header.frame_id
        )


    @classmethod
    def from_trajectory_msg(cls, msg):
        """Form a path from a Trajectory message."""
        return cls(
            [PathHandler.Point.from_pose(point.pose) for point in msg.points],
            frame_id = msg.header.frame_id,
            trajectory = msg
        )


    def update(self, msg):
        """Update speed profile from the message."""
        self.trajectory = msg
        self.update_error_limit()


    @property
    def x(self):
        """Get the vector of all x-values."""
        if self._dirty or self._x is None:
            self._x = [p.x for p in self.points]

        return self._x


    @property
    def y(self):
        """Get the vector of all y-values."""
        if self._dirty or self._y is None:
            self._y = [p.y for p in self.points]

        return self._y


    @property
    def yaw(self):
        """Get the vector of all yaw values."""
        if self._dirty or self._yaw is None:
            self._yaw = [p.yaw for p in self.points]

        return self._yaw


    @property
    def error(self):
        """Get the vector of all error values."""
        return [p.error for p in self.points]


    @property
    def last_valid_error(self):
        """Get the vector of all error values."""
        return [p.last_valid_error for p in self.points]


    def arange(self, start_index, end_index):
        """Create a range of indices.

        Note: Considers wrapping over path end-start.
        Note: Start and end index are not considered.
        """
        if end_index < start_index:
            return (
                list(range(start_index + 1, len(self.points)))
                + list(range(end_index))
            )
        else:
            return list(range(start_index + 1, end_index))


    # @classmethod
    # def apply_error(cls, self, error):
    #     """Apply an error vector the the path."""
    #     return cls([
    #         PathHandler.Point.apply_error(p, e)
    #         for p, e in zip(self.points, error)
    #     ])


    def apply_error(self, index, error):
        """Apply error to the point of the path."""
        self.points[index].apply_error(error)


    def apply_error_with_interpolation(self, index, error):
        """Apply error to the point of the path with interpolation.

        Note: This interpolates the error between last and current
              received point.
        """
        if self.last_error is not None:
            rg = self.arange(self.last_index, index)
            for _i, i in enumerate(rg):
                self.apply_error(
                    i,
                    (
                        self.last_error
                        + _i * (error - self.last_error) / (1 + len(rg))
                    )
                )

        self.points[index].apply_error(error)

        self.last_index = index
        self.last_error = error


    def save_error(self):
        """Save the error in every point of the path."""
        for i, p in enumerate(self.points):
            p.save_error(
                left = self.left_error[i], right = self.right_error[i]
            )


    def set_error(self, errors):
        """Set the error values and recompute the points location."""
        for p, error in zip(self.points, errors):
            p.set_error(error)


    def to_msg(self):
        """Convert the path into Path message."""
        pth = Path()
        pth.header.frame_id = self.frame_id
        pth.poses = [
            p.to_msg() for p in self.points
        ]
        return pth


    def to_trajectory_msg(self):
        """Convert the path into Trajectory message, reusing states."""
        if self.trajectory is None:
            raise ValueError(
                "Unable to construct Trajectory message as it was not "
                "received yet."
            )

        traj = self.trajectory

        for i in range(len(self.points)):
            traj.points[i].pose = self.points[i].to_pose_msg()

        return traj


    def update_error_limit(self):
        """Update the maximum allowed error based on the velocity and TPP."""
        if self.trajectory is None:
            raise ValueError(
                "Unable to update error limits as Trajectory message was not"
                "received yet."
            )

        error_limits = [0.0 for _ in range(len(self.points))]
        left_error = numpy.ones(len(self.points)) * MAX_ERROR
        right_error = numpy.ones(len(self.points)) * -MAX_ERROR

        # Compute array that contains indices of points shifted by P (from TPP)
        time_array = numpy.asarray([
            tp.time_from_start.to_sec() for tp in self.trajectory.points
        ])

        dt = numpy.diff(
            time_array,
            append = 2 * time_array[-1] - time_array[-2]
        )

        helper_array = numpy.empty(dt.shape, dtype = numpy.int)
        _dt = 0.0
        j = len(dt) - 1

        for i in reversed(range(len(dt))):
            while _dt < 0.7:
                j = (j - 1) % len(dt)
                _dt += dt[j]

            helper_array[i] = j
            _dt -= dt[(i - 1) % len(dt)]


        # At this point, helper_array contains directly index j of the point
        # that has point i as a lookahead point.
        for i2, i1 in enumerate(helper_array):
            p1 = self.points[i1]
            p2 = self.points[i2]

            distance = p1.distanceTo(p2)
            yaw1, yaw2 = p1.yaw, p2.yaw
            w1 = [numpy.cos(yaw1), numpy.sin(yaw1)]
            w2 = [p2.x - p1.x, p2.y - p1.y]

            alpha = numpy.arccos(
                numpy.dot(w1, w2)
                / (numpy.linalg.norm(w1) * numpy.linalg.norm(w2))
            )

            # Change the orientation of alpha
            if (
                numpy.arctan2(
                    w1[0] * w2[1] - w1[1] * w2[0],
                    w1[0] * w2[0] + w1[1] * w2[1]
                ) < 0.0
            ):
                alpha *= -1

            # Compute max error based on angle and constrain it
            if yaw2 - yaw1 != 0.0:
                max_error = (
                    distance * numpy.cos(alpha) / numpy.sin(yaw2 - yaw1)
                )
            error_limits[i2] = min(max(max_error, -MAX_ERROR), MAX_ERROR)


            # Compute the max error with respect to the turning circle
            gamma = numpy.radians(90) - abs(yaw1 + alpha - yaw2)
            inner = (
                -2 * K_LIM * numpy.sin(gamma) * distance
                + numpy.cos(abs(alpha) - gamma)
            )

            # If this is not met, the turning circle does not intersect
            # with the error line; i.e. it does not apply.
            if abs(inner) <= 1.0:
                beta = (numpy.arccos(inner) - abs(alpha) - gamma) / 2.0

                if alpha < 0:
                    beta *= -1.0

                Bdistance = numpy.sin(alpha + beta) / K_LIM
                max_error = Bdistance * numpy.sin(beta) / numpy.sin(gamma)

                if Bdistance < 0.0:
                    max_error *= -1

                error_limits[i2] = min(max(max_error, -MAX_ERROR), MAX_ERROR)


            if error_limits[i2] >= 0.0:
                left_error[i2] = max_error

            if error_limits[i2] <= 0.0:
                right_error[i2] = max_error


        self.left_error = left_error
        self.right_error = right_error


######################
# Node
######################

class RunNode(Node):
    """ROS node that moves the path according to the tracking error."""

    def __init__(self, name = "alt_trajectory", *args, **kwargs):
        """Initialize the node."""
        super(RunNode, self).__init__(name, *args, **kwargs)

        self.original_path = None
        self.saving = False


    @Subscriber("/path/original", Path)
    # TODO: Path actually does not contain the orientation every time.
    def callback_original_path(self, msg):
        """Obtain the original path to be moved."""
        self.loginfo("Received original path with size %d." % len(msg.poses))
        self.original_path = PathHandler.from_msg(msg)


    @Subscriber("/trajectory/autoware", Trajectory)
    def callback_original_trajectory(self, msg):
        """Obtain the original trajectory to be moved."""
        if self.original_path is not None:
            self.original_path.update(msg)

            self.loginfo(
                "Updated trajectory speed profile."
            )
            return

        self.loginfo(
            "Received original trajectory with size %d." % len(msg.points)
        )
        self.original_path = PathHandler.from_trajectory_msg(msg)


    @Subscriber("/trajectory/difference/index_lateral_first", String)
    def callback_error(self, msg):
        """Obtain the tracking error."""
        if self.saving:
            return

        index, error = msg.data.split(",")
        self.original_path.apply_error_with_interpolation(
            int(float(index)), float(error)
        )


    @Timer(1)
    @Publisher("/path/visualization", Path)
    def pub_path_viz(self, *args, **kwargs):
        """Publish the edited path for visualization.

        Note: *args, **kwargs are required because of @Timer.
        """
        if self.original_path is not None:
            return self.original_path.to_msg()


    @Publisher("/path/moved", Path, latch = True)
    def pub_path(self, *args, **kwargs):
        """Publish the edited path.

        Note: *args, **kwargs are required because of @Timer.
        """
        self.saving = True
        self.original_path.save_error()
        m = self.original_path.to_msg()
        self.saving = False
        return m


    # @Timer(2)  # 20
    @Subscriber("/lap_time", Time)
    @Publisher("/trajectory", Trajectory, latch = True)
    def pub_trajectory(self, *args, **kwargs):
        """Publish the edited path.

        Note: *args, **kwargs are required because of @Timer.
        """
        if self.original_path is not None:
            self.saving = True
            self.original_path.save_error()
            m = self.original_path.to_trajectory_msg()
            self.saving = False
            return m


    # @Timer(10)  # 70
    @Publisher("/friction_vector/change_diff", String)
    @Publisher("/trajectory/tracking_error", String)
    @Publisher("/trajectory/tracking_error_total", String)
    def pub_fvector(self, *args, **kwargs):
        """Publish the friction vector to update the trajectory speed.

        Note: *args, **kwargs are required because of @Timer.
        """
        if self.original_path is None:
            return None

        errs = []
        error = self.original_path.error
        lverror = self.original_path.last_valid_error

        errs_from_total = [0.0 for i in range(len(error))]

        for index, (total_error, last_error) in enumerate(zip(error, lverror)):
            if not (
                self.original_path.left_error[index] > total_error > self.original_path.right_error[index]
            ):
                errs.append(-0.02)

                for i in self.original_path.arange(index - 20, index):
                    errs_from_total[i] += 1.0
            else:
                err = abs(last_error)

                if err < 0.07:
                    errs.append(0.02)
                elif err < 0.12:
                    errs.append(0.0)
                else:
                    errs.append(-0.02)

        for i in range(len(errs)):
            if errs_from_total[i] > 0.0:
                errs[i] += -0.01

        return [
            String(",".join(["%f" % value for value in errs])),
            String(",".join(["%f" % value for value in lverror])),
            String(",".join(["%f" % value for value in error]))
        ]


    # @Timer(15)
    def smoothen_error(self, *args, **kwargs):
        """Smoothen the total error using average filter."""
        if self.original_path is None:
            return

        self.original_path.set_error(
            numpy.clip(
                uniform_filter1d(self.original_path.error, 10, mode = "wrap"),
                self.original_path.right_error,
                self.original_path.left_error
            )
        )


######################
# Main
######################

if __name__ == "__main__":
    Execute(RunNode)
