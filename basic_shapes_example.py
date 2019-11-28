#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
from uid import UID32

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

def make_shape(typeName, x, y, z):
    ns = 'shape'
    scale = 1
    pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))

    r = 1
    g = 1
    b = 1

    types = {
        'cube': Marker.CUBE,
        'sphere': Marker.SPHERE,
        'arrow': Marker.ARROW,
        'cylinder': Marker.CYLINDER,
    }

    return Marker(
        type=types[typeName],
        ns=ns,
        id=uid32.next(ns),
        scale=Vector3(scale, scale, scale),
        pose=pose,
        header=Header(frame_id='map'),
        color=ColorRGBA(r, g, b, 1.0))

def main():
    rospy.init_node('path_viz')
    wait_for_time()

    # Queue size must be specified
    # It is the number of messages to queue before starting to drop messages
    # Bigger queue size means more latency but fewer missed messages
    # Smaller queue size means low latency but less reliability
    markerPublisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(0.5) # Have to wait for others to properly subscribe to us

    cubeMsg = make_shape('cube', 0, 0, 0)
    sphereMsg = make_shape('sphere', 1, 0, 0)
    arrowMsg = make_shape('arrow', 2, 0, 0)
    cylinderMsg = make_shape('cylinder', 3, 0, 0)

    markerPublisher.publish(cubeMsg)
    markerPublisher.publish(sphereMsg)
    markerPublisher.publish(arrowMsg)
    markerPublisher.publish(cylinderMsg)

    rospy.spin()

if __name__ == '__main__':
    main()
