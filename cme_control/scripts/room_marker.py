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

def mark_room(link, pose=None, scale=0.25):
    if not pose:
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 3.1

    marker = Marker()
    marker.ns = 'room'
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
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 0.75

    rospy.loginfo("Created marker \"%s\" for link \"%s\"", marker.text, link)

    return marker

def main():
    # init ros node
    rospy.init_node('cme_room_marker', anonymous=False)

    room_link_re = re.compile('room_([0-9]+)')

    rospy.loginfo('Searching for rooms...')
    rospy.sleep(3)

    # TODO: find world namespace?
    room_links = find_links(room_link_re, namespace='/world')

    marker_publisher = rospy.Publisher(
        '/visualization_marker_array',
        MarkerArray,
        queue_size=1,
        latch=True,
    )

    while marker_publisher.get_num_connections() == 0:
        rospy.loginfo_throttle(60, "Waiting for subscriber on " + marker_publisher.name)
        rospy.sleep(1)

    room_markers = MarkerArray()
    room_markers.markers = []
    for link in room_links:
        # create a text marker for each room
        room_markers.markers.append(mark_room(link.name))

    if not room_markers.markers:
        rospy.logfatal("No links found for %s", room_link_re.pattern)
        return 1

    marker_publisher.publish(room_markers)
    rospy.loginfo("Published marker array")

    rospy.loginfo("Latching publisher")
    rospy.spin()

    return 0

if __name__ == "__main__":
    sys.exit(main())
