#!/usr/bin/env python2.7
# -*- encoding: utf-8 -*-
"""
"""
from __future__ import print_function

import re
import sys
import numpy
import rospy
import threading
from geometry_msgs.msg import Pose, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from common import find_links

def marker_delete_all():
    marker = Marker()


def mark_points(points, scale=0.25):
    marker = Marker()
    marker.ns = 'pose'
    marker.id = 0
    marker.type = Marker.POINTS
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.action = Marker.ADD

    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = 0.75
    marker.color.g = 0.75
    marker.color.b = 0.0
    marker.color.a = 0.5

    marker.points = [p.pose.position for p in points]

    rospy.loginfo("Created pose marker consisting of %d points", len(points))

    return marker

def vector3_to_numpy(vectors):
    arr = numpy.array(
        [[vec.pose.position.x, vec.pose.position.y, vec.pose.position.z]
         for vec in vectors]
    )
    return arr

def simplify_line(np_vectors, distance=0.025):
    '''Simplify points on the line using Ramer-Douglas-Peucker line simplification.

    https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm

    Args:
        np_vectors: A list of numpy points
        distance: maximum perpendicular distance between points
    Returns:
        List of simplified points
    '''
    # This sort of works?
    raise NotImplementedError
    if len(np_vectors) < 10:
        return np_vectors

    begin = np_vectors[0]
    end = np_vectors[-1] if (np_vectors[-1] != begin).all() else np_vectors[-2]

    # perpendicular distance
    distSq = [
        abs((numpy.cross(vec - begin, end - begin) / numpy.linalg.norm(vec - begin))[2])
        for vec in np_vectors[1:-1]
    ]

    maxdist = max(distSq)
    if maxdist < distance ** 2:
        return [begin, end]

    index = distSq.index(maxdist)
    return (simplify_line(np_vectors[:index + 2], distance) +
            simplify_line(np_vectors[index + 1:], distance)[1:])

def NavfnROS_callback(msg, cb_args):
    points = cb_args[0]
    points_lock = cb_args[1]
    with points_lock:
        points.extend(msg.poses)

def main():
    # init ros node
    rospy.init_node('cme_path_marker', anonymous=False)

    points = []
    points_lock = threading.RLock()
    #   TODO: Configure plan type from
    navfn_subscriber = rospy.Subscriber(
        '/move_base/NavfnROS/plan', Path, NavfnROS_callback, [points, points_lock], queue_size=1,
    )

    marker_publisher = rospy.Publisher(
        '/visualization_marker',
        Marker,
        queue_size=1,
    )

    rospy.sleep(1)
    while marker_publisher.get_num_connections() == 0:
        rospy.loginfo_throttle(60, "Waiting for subscriber on " + marker_publisher.name)
        rospy.sleep(1)

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        if not points:
            continue
        with points_lock:
            try:
                simple_points = simplify_line(vector3_to_numpy(points))
                # TODO: numpy arr back to vector3 - use indexes?
            except (ValueError, NotImplementedError):
                size = int(round(
                    len(points) / numpy.interp(len(points), [1, 400], [1, 10])
                ))
                # fallback to just slicing the array
                simple_points = points[:-size+1:size]
                simple_points.append(points[-1])
            marker_publisher.publish(mark_points(simple_points))
            # flush points array without reinitializing
            del points[:]

    return 0

if __name__ == "__main__":
    sys.exit(main())
