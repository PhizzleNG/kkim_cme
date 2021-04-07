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

def mark_light(link, pose=None, scale=0.15):
    if not pose:
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0.1
        pose.position.z = 1

    marker = Marker()
    marker.ns = 'light'
    marker.id = int(link.split('_')[2])
    marker.text = '{} {}'.format(marker.ns.title(), marker.id)
    marker.header.frame_id = link
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose = pose
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r = 0.0
    marker.color.g = 0.45
    marker.color.b = 0.95
    marker.color.a = 0.75

    rospy.loginfo("Created marker \"%s\" for link \"%s\"", marker.text, link)

    return marker

def main():
    # init ros node
    rospy.init_node('cme_light_marker', anonymous=False)

    light_link_re = re.compile('light_switch_([0-9]+)')

    rospy.loginfo('Searching for lights...')
    rospy.sleep(3)

    # TODO: find world namespace?
    light_links = find_links(light_link_re, namespace='/world')

    marker_publisher = rospy.Publisher(
        '/visualization_marker_array',
        MarkerArray,
        queue_size=1,
        latch=True,
    )

    while marker_publisher.get_num_connections() == 0:
        rospy.loginfo_throttle(60, "Waiting for subscriber on " + marker_publisher.name)
        rospy.sleep(1)

    light_markers = MarkerArray()
    light_markers.markers = []
    for link in light_links:
        # create a text marker for each light
        light_markers.markers.append(mark_light(link.name))

    if not light_markers.markers:
        rospy.logfatal("No links found for %s", light_link_re.pattern)
        return 1

    marker_publisher.publish(light_markers)
    rospy.loginfo("Published marker array")

    rospy.loginfo("Latching publisher")
    rospy.spin()

    return 0

if __name__ == "__main__":
    sys.exit(main())
