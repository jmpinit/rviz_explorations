#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker

class UID32(object):
    def __init__(self):
        self.id = 0

    def next(self):
        id = self.id
        self.id += 1
        return id

uid32 = UID32()

def wait_for_time():
    """Wait for simulated time to begin."""

    # An explanation of how this system works can be found here:
    # http://wiki.ros.org/Clock
    # rospy is subscribing to /clock to get the simulation time
    # NOTE: the docs say this API is not for real-time applications
    # I do not know the implications on implementation of real-time systems
    while rospy.Time().now().to_sec() == 0:
        pass

def line_strip_from_xyz(points, color=(1.0, 1.0, 1.0)):
    """Create a line strip marker message from a list of (x, y, z) tuples"""

    path = [Point(x, y, z) for (x, y, z) in points]

    # ns (namespace) is combined with id to make a unique identifier
    # so id must be unique. It's a 32 bit integer.

    scale = 0.05 # How thick this line will be drawn
    pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    r, g, b = color

    lineStripMsg = Marker(
        type=Marker.LINE_STRIP,
        ns='path',
        id=uid32.next(),
        points=path,
        scale=Vector3(scale, 0.0, 0.0),
        pose=pose,
        header=Header(frame_id='odom'),
        color=ColorRGBA(r, g, b, 1.0))

    return lineStripMsg

class Turtle(object):
    def __init__(self):
        self.angle = 0
        self.x = 0
        self.y = 0
        self.points = [(self.x, self.y)]

    def forward(self, distance):
        self.x += distance * math.cos(self.angle)
        self.y += distance * math.sin(self.angle)
        self.points.append((self.x, self.y))

    def turn(self, angle):
        self.angle += angle

def dragon_curve_construct(turtle, order, length, sign):
  if order == 0:
    turtle.forward(length)
  else:
    rootHalf = (1.0 / 2.0) ** (1.0 / 2.0)
    dragon_curve_construct(turtle, order - 1, length * rootHalf, 1)
    turtle.turn(sign * -math.pi / 2.0)
    dragon_curve_construct(turtle, order - 1, length * rootHalf, -1)

def dragon_curve(order, length):
    turtle = Turtle()
    turtle.turn(order * math.pi / 4.0)
    dragon_curve_construct(turtle, order, length, 1)
    return turtle.points

def main():
    rospy.init_node('path_viz')
    wait_for_time()

    # Queue size must be specified
    # It is the number of messages to queue before starting to drop messages
    # Bigger queue size means more latency but fewer missed messages
    # Smaller queue size means low latency but less reliability
    markerPublisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(0.5) # Have to wait for others to properly subscribe to us

    points = [(x, y, 0) for (x, y) in dragon_curve(8, 4)]

    lineStripMsg = line_strip_from_xyz(points)
    markerPublisher.publish(lineStripMsg)

    rospy.spin()

if __name__ == '__main__':
    main()
