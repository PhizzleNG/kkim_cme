#!/usr/bin/env python2.7
# -*- encoding: utf-8 -*-
"""
"""
from __future__ import print_function

import re
import sys
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import (
    Marker, MarkerArray,
)
from common import find_links

def mark_door(link, pose=None, scale=0.2):
    if not pose:
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 3.1

    marker = Marker()
    marker.ns = 'door'
    marker.id = int(link.split('_')[1])
    marker.text = '{} {}'.format(marker.ns.title(), marker.id)
    marker.header.frame_id = link
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = 0.75
    marker.color.g = 0.75
    marker.color.b = 0.75
    marker.color.a = 0.75

    rospy.loginfo("Created marker \"%s\" for link \"%s\"", marker.text, link)

    return marker

def main():
    # init ros node
    rospy.init_node('cme_door_marker', anonymous=False)

    door_link_re = re.compile('door_([0-9]+)_frame')

    rospy.loginfo('Searching for doors...')
    rospy.sleep(3)

    # TODO: find world namespace?
    door_links = find_links(door_link_re, namespace='/world')

    marker_publisher = rospy.Publisher(
        '/visualization_marker_array',
        MarkerArray,
        queue_size=1,
        latch=True,
    )

    while marker_publisher.get_num_connections() == 0:
        rospy.loginfo_throttle(60, "Waiting for subscriber on " + marker_publisher.name)
        rospy.sleep(1)

    door_markers = MarkerArray()
    door_markers.markers = []
    for link in door_links:
        # create a text marker for each door
        door_markers.markers.append(mark_door(link.name))

    if not door_markers.markers:
        rospy.logfatal("No links found for %s", door_link_re.pattern)
        return 1

    marker_publisher.publish(door_markers)
    rospy.loginfo("Published marker array")

    rospy.loginfo("Latching publisher")
    rospy.spin()

    return 0

if __name__ == "__main__":
    sys.exit(main())
