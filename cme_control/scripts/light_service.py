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
from common import find_links

MARKER_SIZE = [1, 1, 0.2]
MARKER_ON_COLOR = (1, 1, 1, 0.75)
MARKER_OFF_COLOR = (0.2, 0.2, 0.2, 0.5)

def build_marker(_id, link, pose=None):
    if not pose:
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 2.8

    marker = Marker()
    marker.ns = rospy.get_name()
    marker.id = _id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.header.frame_id = link
    marker.header.stamp = rospy.Time.now()
    marker.pose = pose
    marker.scale.x = MARKER_SIZE[0]
    marker.scale.y = MARKER_SIZE[1]
    marker.scale.z = MARKER_SIZE[2]
    marker.color.r = MARKER_OFF_COLOR[0]
    marker.color.g = MARKER_OFF_COLOR[1]
    marker.color.b = MARKER_OFF_COLOR[2]
    marker.color.a = MARKER_OFF_COLOR[3] if len(MARKER_OFF_COLOR) > 3 else 0.5

    rospy.loginfo("Created light %d marker", _id)

    return marker

def light_on_service(_id, marker, marker_publish):
    # TODO: Light switch moves
    def callback(req):
        marker.color.r = MARKER_ON_COLOR[0]
        marker.color.g = MARKER_ON_COLOR[1]
        marker.color.b = MARKER_ON_COLOR[2]
        marker.color.a = MARKER_ON_COLOR[3] if len(MARKER_ON_COLOR) > 3 else 0.5
        marker_publish()
        return LightOnResponse()

    service_name = 'light_{}/on'.format(_id)
    return rospy.Service(service_name, LightOn, callback)

def light_off_service(_id, marker, marker_publish):
    # TODO: Light switch moves
    def callback(req):
        marker.color.r = MARKER_OFF_COLOR[0]
        marker.color.g = MARKER_OFF_COLOR[1]
        marker.color.b = MARKER_OFF_COLOR[2]
        marker.color.a = MARKER_OFF_COLOR[3] if len(MARKER_OFF_COLOR) > 3 else 0.5
        marker_publish()
        return LightOffResponse()

    service_name = 'light_{}/off'.format(_id)
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
    light_links = find_links('light_switch_([0-9]+)$', namespace='/world')

    room_links = find_links('room_([0-9]+)', namespace='/world')

    light_markers = MarkerArray()
    light_markers.markers = []
    light_services = {}

    # TODO: yeah...
    def marker_publish():
        marker_publisher.publish(light_markers)

    for light in light_links:
        light_id = int(light.name.rpartition('_')[-1])
        marker = build_marker(light_id, *[l.name for l in room_links if l.name == 'room_{}'.format(light_id)])
        light_markers.markers.append(marker)
        light_services[light_id] = {
            'on': light_on_service(light_id, marker, marker_publish),
            'off': light_off_service(light_id, marker, marker_publish),
        }

    if not light_markers.markers:
        rospy.logfatal("No links found for %s", light_link_re.pattern)
        return 1

    marker_publish()
    rospy.loginfo("Published marker array")

    rospy.loginfo("Latching publisher")
    rospy.spin()

    return 0

if __name__ == "__main__":
    sys.exit(main())
