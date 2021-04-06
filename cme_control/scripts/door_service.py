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
from std_msgs.msg import Float64
from common import get_controller_list, find_controllers

# TODO(Cosmo): Get joint limits from joint_limits_interface:
#              https://github.com/ros-controls/ros_control/wiki/joint_limits_interface
DOOR_OPEN_ANGLE = pi/2
DOOR_CLOSE_ANGLE = 0


def door_open_service(name):
    door_open_publisher = rospy.Publisher(
            '{}/command'.format(name),
            Float64,
            queue_size=1,
    )
    def callback(req):
        door_open_publisher.publish(
            Float64(DOOR_OPEN_ANGLE)
        )
        return DoorOpenResponse(DOOR_OPEN_ANGLE)

    service_name = '{}/open'.format(name.rpartition('_')[0])
    return rospy.Service(service_name, DoorOpen, callback)

def door_close_service(name):
    door_close_publisher = rospy.Publisher(
            '{}/command'.format(name),
            Float64,
            queue_size=1,
    )
    def callback(req):
        door_close_publisher.publish(
            Float64(DOOR_CLOSE_ANGLE)
        )
        return DoorCloseResponse(DOOR_CLOSE_ANGLE)

    service_name = '{}/close'.format(name.rpartition('_')[0])
    return rospy.Service(service_name, DoorClose, callback)

def main():
    # init ros node
    rospy.init_node('cme_door_service', anonymous=False)

    door_controller_re = re.compile('door_([0-9]+)_([A-Za-z0-9-]+)')

    rospy.loginfo('Searching for controllers...')
    rospy.sleep(3)

    door_controllers = find_controllers(door_controller_re)

    door_markers = {}
    door_services = {}

    for controller in door_controllers:
        if controller.name.endswith('position'):
            # create an open/close service for each door
            door_services[controller.name] = {
                'open': door_open_service(controller.name),
                'close': door_close_service(controller.name),
            }
        else:
            rospy.logdebug('Not handling controller "%s"', controller.name)

    rospy.spin()

    return 0

if __name__ == "__main__":
    sys.exit(main())
