#!/usr/bin/env python2.7
# -*- encoding: utf-8 -*-
"""
TODO: Document what topics/services this affects
"""
from __future__ import print_function

import re
import sys
import rospy
from math import pi, sqrt
from cme_control.srv import (
    DoorClose, DoorCloseResponse,
    DoorOpen, DoorOpenResponse,
)
from controller_manager_msgs.srv import (
    ListControllers, ListControllersRequest,
)
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from std_msgs.msg import Float64
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
)

# TODO(Cosmo): Get joint limits from joint_limits_interface:
#              https://github.com/ros-controls/ros_control/wiki/joint_limits_interface
DOOR_OPEN_ANGLE = pi/2
DOOR_CLOSE_ANGLE = 0


def get_controller_list():
    # abstracted from controller_manager.controller_manager_interface
    rospy.wait_for_service('controller_manager/list_controllers')

    s = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())

    controllers = {}
    if len(resp.controller) > 0:
        for c in resp.controller:
            hwi = list(set(r.hardware_interface for r in c.claimed_resources))
            rospy.loginfo(
                'Found controller: {name} - {hwi} ( {state} )'.format(
                    name=c.name,
                    hwi='+'.join(hwi),
                    state=c.state,
                )
            )

    return resp.controller

def get_controller_joints():
    # return [(controller_name, joint_name)]
    rospy.get_param("{}/joint".format(controller_name))

def marker_callback(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        x=feedback.pose.position.x,
        y=feedback.pose.position.y,
        z=feedback.pose.position.z,
        rospy.loginfo_throttle(.1,
            "{name} is now at {x}, {y}, {z}".format(
                name=feedback.marker_name,
                x=x, y=y, z=z,
            )
        )

def mark_door(controller, server, pose=None):
    # https://github.com/ros-visualization/visualization_tutorials/blob/kinetic-devel/interactive_marker_tutorials/scripts/simple_marker.py
    # TODO(Cosmo): Multiple joints?
    # TODO(Cosmo): get name of link joint is attached to
    # TODO(Cosmo): put at center of door?
    # TODO(Cosmo): Marker should follow door link position

    return controller.name

    #controller_joint = controller.claimed_resources[0].resources[0]
    controller_link = controller.name.rpartition('_')[0] + '_frame'

    if not pose:
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 1.75

    marker = InteractiveMarker()
    marker.header.frame_id = controller_link
    marker.scale = 0.5
    marker.pose = pose

    marker.name = controller.name.rpartition('_')[0]
    marker.description = controller.name.replace('_', ' ')

    box = Marker()
    box.type = Marker.ARROW
    box.scale.x = marker.scale * 0.45
    box.scale.y = marker.scale * 0.45
    box.scale.z = marker.scale * 0.45
    box.color.r = 0.0
    box.color.g = 0.5
    box.color.b = 0.5
    box.color.a = 1.0

    control = InteractiveMarkerControl()
    control.always_visible = True
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    #control.markers.append(box)

    marker.controls.append(control)

    server.insert(marker)
    server.setCallback(marker.name, marker_callback)

    return marker

def door_open_service(name):
    door_open_publisher = rospy.Publisher(
            '/{}/command'.format(name),
            Float64,
            queue_size=1,
    )
    def callback(req):
        door_open_publisher.publish(
            Float64(DOOR_OPEN_ANGLE)
        )
        return DoorOpenResponse(DOOR_OPEN_ANGLE)

    service_name = '/{}/open'.format(name.rpartition('_')[0])
    return rospy.Service(service_name, DoorOpen, callback)

def door_close_service(name):
    door_close_publisher = rospy.Publisher(
            '/{}/command'.format(name),
            Float64,
            queue_size=1,
    )
    def callback(req):
        door_close_publisher.publish(
            Float64(DOOR_CLOSE_ANGLE)
        )
        return DoorCloseResponse(DOOR_CLOSE_ANGLE)

    service_name = '/{}/close'.format(name.rpartition('_')[0])
    return rospy.Service(service_name, DoorClose, callback)

def find_controllers(search_term):
    controllers = []
    if isinstance(search_term, re._pattern_type):
        controller_re = search_term
    else:
        controller_re = re.compile(search_term)

    while not controllers and not rospy.is_shutdown():
        controller_list = get_controller_list()
        if not controller_list:
            rospy.loginfo_throttled(120, 'Waiting for more controllers...')
            continue

        controllers = [
            controller for controller in controller_list
            if controller_re.match(controller.name)
        ]
        rospy.sleep(1)

    return controllers

def main():
    # init ros node
    rospy.init_node('cme_door_service', anonymous=False)

    door_controller_re = re.compile('door_([0-9]+)_([A-Za-z0-9-]+)')

    rospy.loginfo('Searching for controllers...')

    door_controllers = find_controllers(door_controller_re)

    marker_server = InteractiveMarkerServer('cme_door_service')

    door_markers = {}
    door_services = {}

    for controller in door_controllers:
        if controller.name.endswith('position'):
            # create an interactive marker for each door
            door_markers[controller.name] = mark_door(controller, marker_server)
            # create an open/close service for each door
            door_services[controller.name] = {
                'open': door_open_service(controller.name),
                'close': door_close_service(controller.name),
            }
        else:
            rospy.logdebug('Not handling controller "%s"', controller.name)

    marker_server.applyChanges()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

    return 0

if __name__ == "__main__":
    sys.exit(main())
