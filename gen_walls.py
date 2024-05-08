#!/usr/bin/env python3
from autopsy.node import Node
from autopsy.core import Core
from autopsy.reconfigure import ParameterServer

from nav_msgs.msg import OccupancyGrid

import numpy
from PIL import Image as P
from PIL.Image import Image
from PIL import ImageDraw as PDraw

from typing import List

import sys

args = ParameterServer()
args.verbose = True
args.inflate = 0 # does not work now; it inflates but then the borders are not found


HOOD4 = numpy.asarray([[-1, 0], [0, -1], [1, 0], [0, 1]])


def getWalls(im: Image) -> List[numpy.ndarray]:
    """Get wall sequences from the image.

    Arguments:
    im -- source image, Image

    Returns:
    walls -- sets of points belonging to walls, m-list of x2 numpy.ndarray

    Sources:
    https://stackoverflow.com/questions/46083880/fill-in-a-hollow-shape-using-python-and-pillow-pil
    """
    # Duplicate image
    _im = im.copy()

    # Obtain wall color
    # Note: In grayscale image, 0 is black, 1 is white.
    hist = numpy.asarray(_im.histogram())
    wall_color = numpy.argwhere(hist == hist[hist > 0][0])[0][0]

    walls = []

    # Progress
    starting_value = _im.histogram()[wall_color]

    while _im.histogram()[wall_color] > 0:

        # Convert image to numpy array
        _nd = numpy.asarray(_im)

        # Find specified color somewhere
        _wall = numpy.argwhere(_nd == wall_color)[0]

        # Use PIL fill tool to color the wall
        _color = numpy.argwhere(numpy.asarray(_im.histogram()) == 0)[0][0]
        PDraw.floodfill(
            _im,
            xy = (int(_wall[1]), int(_wall[0])),
            value = int(_color),
            thresh = 0,
        )

        # Recast image to numpy
        _nd = numpy.asarray(_im)

        # Get coordinates of newly colored walls
        walls.append(
            numpy.argwhere(
                _nd == _color
            )
        )

        sys.stdout.write(
            "\rObtaining walls... %03.2f%%"
            % (100.0 - (100.0 * _im.histogram()[wall_color] / starting_value))
        )

    print (" - DONE")

    # We sort the walls in descending size
    # i.e., indices here are not the same as latter ones,
    # but that is ok.
    walls = sorted(walls, key = lambda x: -len(x))

    if args.verbose:
        print ("Found %d walls:" % len(walls))
        print (
            "\n".join([
                "\t Wall %d: length %d" % (_i + 1, len(wall))
                for _i, wall in enumerate(walls)
            ])
        )

    return walls


def intersectArrays(arr1: numpy.ndarray, arr2: numpy.ndarray) -> numpy.ndarray:
    """Receive rows that are present in both arrays.

    Arguments:
    arr1, arr2 -- input array, numpy.ndarray

    Returns:
    iarr -- intersection of both arrays, numpy.ndarray

    Source:
    # https://stackoverflow.com/questions/9269681/intersection-of-2d-numpy-ndarrays
    """
    arr1_view = arr1.view([('', arr1.dtype)] * arr1.shape[1])
    arr2_view = arr2.view([('', arr2.dtype)] * arr2.shape[1])
    intersected = numpy.intersect1d(arr1_view, arr2_view)
    return intersected.view(arr1.dtype).reshape(-1, arr1.shape[1])


def inflateWalls(im: Image) -> Image:
    """Inflate the walls by a certain amount.

    Arguments:
    im -- loaded image

    Returns:
    _im -- inflated image
    """
    # Create numpy array (also duplicates the image)
    # TODO: Find out, when this was implemented.
    #  it is not required with Python 3.6, Pillow 8.4.0 and numpy 1.19.5
    #  but it is required with Python 3.11, Pillow 9.4.0 and numpy 1.23.5
    nd = numpy.array(im, dtype = numpy.uint8)

    # Name colors (just in case)
    try:
        WALL, FREE, UNKN = numpy.unique(nd) # Swapped!
    except ValueError:
        WALL, FREE = numpy.unique(nd)

    print (
        "Using colors:"
        "\n\tWALL = %d"
        "\n\tUNKN = %d"
        "\n\tFREE = %d"
        % (WALL, UNKN, FREE)
    )


    sys.stdout.write("\rInflating walls...")

    print("\n\tOriginal: %d of wall cells" % len(numpy.argwhere(nd == WALL)))

    for _i in range(args.inflate):

        walls = numpy.argwhere(nd == WALL)

        # Obtain all walls of one color and replicate it
        # https://stackoverflow.com/questions/53239242/how-to-duplicate-each-row-of-a-matrix-n-times-numpy
        _walls = numpy.repeat(walls, repeats = HOOD4.shape[0], axis = 0)

        # Find the neighbourhood
        _hood = numpy.add(
            _walls,
            numpy.tile(HOOD4, reps = (walls.shape[0], 1))
        )

        # Throw away points that are outside the image and remove duplicits
        _hood = numpy.unique(
            _hood[~numpy.any((_hood < 0) | (_hood >= nd.shape), axis = 1), :],
            axis = 0
        )

        # Find all other points
        # Copy is required as argwhere returns 2xn array with nx2 view,
        # which is unfortunately incompatible with current intersect function.
        _not_wall = numpy.argwhere(nd != WALL).copy()

        # Find all free points that are also in the neighbourhood
        # And color them
        z = intersectArrays(_hood, _not_wall)
        nd[z[:, 0], z[:, 1]] = WALL

        print(
            "\tLoop %d: %d of wall cells"
            % (_i + 1, len(numpy.argwhere(nd == WALL)))
        )

    print (" - DONE")

    return P.fromarray(nd)



n = Node("gen_walls")


print ("waiting for map")

m = n.wait_for_message("/map", OccupancyGrid)


print ("received", m.info)


print ("detected colors:", set(m.data))


mn = numpy.asarray(m.data, dtype=numpy.uint8).reshape(m.info.height, m.info.width).T

print (numpy.unique(mn))

print ("swapping colors")

mn[mn <=100] = 100-mn[mn<=100]

print(mn, mn.shape)

im = P.fromarray(mn)

if args.inflate > 0:
    im = inflateWalls(im)
#im.show()
walls = getWalls(im)

print ("found %d walls" % len(walls))
print(im)
print(im.histogram())

#im.show()

nd = mn

# Name colors (just in case)
try:
    WALL, FREE, UNKN = numpy.unique(nd)
    print (
        "Using colors:"
        "\n\tWALL = %d"
        "\n\tUNKN = %d"
        "\n\tFREE = %d"
        % (WALL, UNKN, FREE)
    )
except ValueError:
    WALL, FREE = numpy.unique(nd)
    print (
        "Using colors:"
        "\n\tWALL = %d"
        "\n\tFREE = %d"
        % (WALL, FREE)
    )


# Helper array
nd_walls = numpy.zeros_like(im, dtype = int)


# Color initial walls
for i in range(len(walls)):
    nd_walls[walls[i][:, 0], walls[i][:, 1]] = i + 1

where_free = numpy.argwhere(nd == FREE)
nd_walls[where_free[:, 0], where_free[:, 1]] = len(walls) + 1
#im_walls = P.fromarray(nd_walls.astype(numpy.uint8))
#im_walls.show()

# # Obtain centerline # #
sys.stdout.write("\rGenerating borders...")

border = []
wall_len_1 = 0
nd2 = numpy.asarray(im, dtype = int)

# Get neighbourhood of a wall where it meets different walls.
for wall_index_1 in range(1, len(walls) + 1):

    wall_1 = numpy.argwhere(nd_walls == wall_index_1)

    # Obtain all walls of one color and replicate it
    # https://stackoverflow.com/questions/53239242/how-to-duplicate-each-row-of-a-matrix-n-times-numpy
    _wall = numpy.repeat(wall_1, repeats = HOOD4.shape[0], axis = 0)

    # Find the neighbourhood
    _hood = numpy.add(
        _wall,
        numpy.tile(HOOD4, reps = (wall_1.shape[0], 1))
    )

    # Throw away points that are outside the image and remove duplicits
    _hood = numpy.unique(
        _hood[~numpy.any((_hood < 0) | (_hood >= nd.shape), axis = 1), :],
        axis = 0
    )

    # Find all points of other walls
    # Copy is required as argwhere returns 2xn array with nx2 view,
    # which is unfortunately incompatible with current intersect function.
    _other_walls = numpy.argwhere(nd_walls > wall_index_1).copy()

    # Find all free points that are also in the neighbourhood
    # And append them to the current skeleton
    border.append(numpy.asarray(intersectArrays(
        intersectArrays(_hood, _other_walls),
        numpy.argwhere(nd2 == FREE).copy()
    ).tolist()))

    if wall_index_1 == 1:
        wall_len_1 = len(border)

    sys.stdout.write(
        "\rGenerating borders... %03.2f%%"
        % (100.0 * wall_index_1 / (len(walls)))
    )


#border = numpy.asarray(border)
print (" - DONE")
#print ("Centerline has length %d." % len(border))

for i in range(len(border)):
    print ("Wall-border %d has length %d" % (i+1, len(border[i])))


#border = numpy.unique(border, axis = 0)
#print ("and has %d unique points." % len(border))



nnd = numpy.array(im.convert("RGB"))

# Show the centerline
#  - RED = Final centerline
#  - GREEN = Filtered points
#try:
#    nnd[border[:, 0], border[:, 1]] = [0, 255, 0]
#except IndexError:
#    pass

for i, color in zip(
    range(len(border)),
    [[0, 255, 0], [255, 0, 0]]
):
    nnd[border[i][:, 0], border[i][:, 1]] = color

nndi = P.fromarray(nnd)
#nndi.save("generated_data.png")
#nndi.show()

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

pub_walls = n.Publisher("/borders", MarkerArray, queue_size = 1, latch=True)

msg = MarkerArray()

for i in range(len(border)):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.ns = "wall%d" % i
    marker.scale.x = m.info.resolution
    marker.scale.y = m.info.resolution
    marker.color = ColorRGBA(
        r = 0.0, g = 1.0, b = 0.0, a = 1.0
    )
    marker.type = 8
    marker.points = []

    for x, y in zip(border[i][:, 0], border[i][:, 1]):
        marker.points.append(
            Point(
                x = float(x * m.info.resolution + m.info.origin.position.x + m.info.resolution/2),
                y = float(y * m.info.resolution + m.info.origin.position.y + m.info.resolution/2),
                z = 0.0
            )
        )

    msg.markers.append(marker)

print ("publishing walls")

pub_walls.publish(msg)

Core.spin(n)
