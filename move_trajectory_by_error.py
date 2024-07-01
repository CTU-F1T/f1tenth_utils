#!/usr/bin/env python
# move_trajectory_by_error.py
"""A testing file for moving a trajectory by tracking error.

Will be incorporated into profile_trajectory2 probably.
"""
######################
# Imports & Globals
######################

import csv
import matplotlib.pyplot
import numpy
import argparse

PARSER = argparse.ArgumentParser(
    description = """
Move a trajectory according to the tracking error.
"""
)

PARSER.add_argument(
    "trajectory_file",

    help = "a .csv file with trajectory data",
    type = argparse.FileType("r")
)

PARSER.add_argument(
    "error_file",

    help = "a .csv file with error data",
    type = argparse.FileType("r")
)

PARSER.add_argument(
    "-e",

    help = "a factor to multiply error before creating new trajectory",
    type = float,
    default = 1.0,
)

PARSER.add_argument(
    "-s",

    help = "name of the file to save the edited trajectory",
    type = argparse.FileType("w")
)


######################
# Trajectory
######################

class Trajectory(object):
    """Basic class for representing a trajectory."""

    def __init__(self):
        """Initialize the trajectory."""
        super(Trajectory, self).__init__()
        self.trajectory = []


    def addPoint(self, p):
        """Add a point to the trajectory.

        Arguments
        ---------
        p: TrajectoryPoint
        """
        self.trajectory.append(p)


    def __getitem__(self, item):
        """Get parameter from all points of the trajectory.

        Arguments
        ---------
        item: str
            name of the item
        """
        if isinstance(item, int):
            return self.trajectory[item]
        else:
            return [p[item] for p in self.trajectory]



    def __len__(self):
        """Get length of the trajectory.

        Returns
        -------
        length: int
            number of points in the trajectory
        """
        return len(self.trajectory)


class TrajectoryPoint(object):
    """Basic class for representing a trajectory point."""

    def __init__(self, **args):
        """Initialize the trajectory point."""
        super(TrajectoryPoint, self).__init__()

        self.values = args
        self.errors = []

        self.values["error_m"] = 0.0


    def __getitem__(self, item):
        """Get parameter of the trajectory point.

        Arguments
        ---------
        item: str
            name of the item
        """
        return float(self.values[item])


    def setError(self, error):
        """Set tracking error of the trajectory point."""
        self.errors.append(error)

        self.values["error_m"] = sum(self.errors) / len(self.errors)


######################
# Utilities
######################

def plotTrajectory(t):
    """Plot a trajectory.

    Arguments
    ---------
    t: Trajectory
    """
    matplotlib.pyplot.plot(
        t["x_m"], t["y_m"]
    )

    matplotlib.pyplot.axis("equal")

    matplotlib.pyplot.show()


def scatterTrajectoryError(t, A):
    """Scatter a trajectory with error.

    Arguments
    ---------
    t: Trajectory
    """
    matplotlib.pyplot.plot(
        t["x_m"], t["y_m"]
    )

    x_m = numpy.asarray(t["x_m"])
    y_m = numpy.asarray(t["y_m"])

    psi_rad = numpy.asarray(t["psi_rad"]) + numpy.radians(90)
    error_m = numpy.asarray(t["error_m"])

    x2_m = x_m + (numpy.cos(psi_rad) * error_m * A)
    y2_m = y_m + (numpy.sin(psi_rad) * error_m * A)

    matplotlib.pyplot.plot(x2_m, y2_m)

    for i in range(len(x_m)):
        matplotlib.pyplot.plot(
            [x_m[i], x2_m[i]], [y_m[i], y2_m[i]], 'black'
        )

    matplotlib.pyplot.axis("equal")

    matplotlib.pyplot.show()

    return (x2_m, y2_m)


######################
# Main
######################

def main():
    """Execute the application."""
    args = PARSER.parse_args()

    print ("Arguments:")
    print ("\n".join([
        "\t%s: %s" % (key, value) for key, value in vars(args).items()
    ]))

    t = Trajectory()

    reader = csv.DictReader(args.trajectory_file)

    for row in reader:
        t.addPoint(
            TrajectoryPoint(**row)
        )

    print ("Loaded %d points." % len(t))


    try:
        reader = csv.DictReader(
            args.error_file, fieldnames = ["time", "index", "error"]
        )
        reader.next()

        for row in reader:
            t[int(float(row["index"]))].setError(float(row["error"]))

    except Exception:
        reader = csv.DictReader(
            args.error_file, fieldnames = ["index", "error"]
        )
        reader.next()

        for row in reader:
            t[int(float(row["index"]))].setError(float(row["error"]))


    x2, y2 = scatterTrajectoryError(t, args.e)


    if args.s is not None:
        writer = csv.DictWriter(args.s, fieldnames = ["x_m", "y_m"])
        writer.writeheader()

        for _x, _y in zip(x2, y2):
            writer.writerow({"x_m": _x, "y_m": _y})

        print ("Wrote %d points into %s." % (len(x2), args.s.name))


if __name__ == "__main__":
    main()
