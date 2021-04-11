#!/usr/bin/env python2.7
# -*- encoding: utf-8 -*-
"""
"""
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import (
    Marker, MarkerArray,
)
from cme_control.srv import (
    LightOn, LightOnResponse,
    LightOff, LightOffResponse,
)
from common import find_links, build_marker

MARKER_SIZE = 0.5
MARKER_ON_COLOR = (1, 1, 1, 0.5)
MARKER_OFF_COLOR = (0.2, 0.2, 0.2, 0.5)

def build_marker(_id):
    marker = Marker()
    marker.ns = rospy.get_name()
    marker.id = _id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.header.frame_id = link
    marker.header.stamp = rospy.Time.now()
    marker.pose = pose
    marker.scale.x = MARKER_SIZE
    marker.scale.y = MARKER_SIZE
    marker.scale.z = MARKER_SIZE
    marker.color.r = MARKER_COLOR[0]
    marker.color.g = MARKER_COLOR[1]
    marker.color.b = MARKER_COLOR[2]
    marker.color.a = MARKER_COLOR[3] if len(MARKER_COLOR) > 3 else 0.5

    rospy.loginfo("Created light %d marker", _id)

    return marker

def light_on_service(name, marker):
    # TODO: Light switch moves
    def callback(req):
        return LightOnResponse(True)

    service_name = '{}/on'.format(name.rpartition('_')[0])
    return rospy.Service(service_name, LightOn, callback)

def light_off_service(name, marker):
    # TODO: Light switch moves
    def callback(req):
        return LightOffResponse(True)

    service_name = '{}/off'.format(name.rpartition('_')[0])
    return rospy.Service(service_name, LightOff, callback)

def main():
    # init ros node
    rospy.init_node('cme_light_service', anonymous=False)

    marker_publisher = rospy.Publisher(
        '/visualization_marker_array',
        MarkerArray,
        queue_size=1,
        latch=True,
    )

    while marker_publisher.get_num_connections() == 0:
        rospy.loginfo_throttle(60, "Waiting for subscriber on " + marker_publisher.name)
        rospy.sleep(1)

    rospy.loginfo('Searching for lights...')
    # TODO: Find world namespace?
    light_links = find_links('light_switch_([0-9]+)', namespace='/world')

    light_markers = MarkerArray()
    light_markers.markers = []
    light_services = {}

    for light in light_links:
        print(light)
        marker = build_marker(light)
        light_markers.append(marker)
        light_services[light] = {
            'on': light_on_service(light, marker, marker_publisher),
            'off': light_off_service(light, marker, marker_publisher),
        }

    if not light_markers.markers:
        rospy.logfatal("No links found for %s", light_link_re.pattern)
        return 1

    marker_publisher.publish(light_markers)
    rospy.loginfo("Published marker array")

    rospy.loginfo("Latching publisher")
    rospy.spin()

    return 0
