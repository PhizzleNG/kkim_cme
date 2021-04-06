#!/usr/bin/env python2
# -*- encoding: utf-8 -*-
import contextlib
import re
import sys
import rospy
from controller_manager_msgs.srv import (
    ListControllers, ListControllersRequest,
)
from urdf_parser_py.urdf import URDF

# https://stackoverflow.com/a/2829036
class DummyFile(object):
    def write(self, x): pass

# https://stackoverflow.com/a/2829036
@contextlib.contextmanager
def redirect_std(stdout=True, stderr=True):
    if stdout:
        _stdout = sys.stdout
        sys.stdout = DummyFile()
    if stderr:
        _stderr = sys.stderr
        sys.stderr = DummyFile()
    yield
    if stdout:
        sys.stdout = _stdout
    if stderr:
        sys.stderr = _stderr

def find_joints(search_term, namespace='', robot=None):
    if isinstance(search_term, re._pattern_type):
        joint_re = search_term
    else:
        joint_re = re.compile(search_term)
    if not robot:
        with redirect_std():
            robot = URDF.from_parameter_server(
                '{0}{1}robot_description'.format(
                    namespace, '/' if not namespace.endswith('/') else '',
                )
            )
    joints = [
        joint for joint in robot.joints
        if joint_re.match(joint.name)
    ]
    return joints

def find_links(search_term, namespace='', robot=None):
    if isinstance(search_term, re._pattern_type):
        link_re = search_term
    else:
        link_re = re.compile(search_term)
    if not robot:
        with redirect_std():
            robot = URDF.from_parameter_server(
                '{0}{1}robot_description'.format(
                    namespace, '/' if not namespace.endswith('/') else '',
                )
            )
    links = [
        link for link in robot.links
        if link_re.match(link.name)
    ]
    return links

def get_controller_list():
    # abstracted from controller_manager.controller_manager_interface
    rospy.wait_for_service('controller_manager/list_controllers')

    s = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())

    if len(resp.controller) > 0:
        for c in resp.controller:
            hwi = list(set(r.hardware_interface for r in c.claimed_resources))
            rospy.logdebug(
                'Found controller: {name} - {hwi} ( {state} )'.format(
                    name=c.name,
                    hwi='+'.join(hwi),
                    state=c.state,
                )
            )

    return resp.controller

def find_controllers(search_term):
    controllers = []
    if isinstance(search_term, re._pattern_type):
        controller_re = search_term
    else:
        controller_re = re.compile(search_term)

    while not controllers and not rospy.is_shutdown():
        controller_list = get_controller_list()
        if not controller_list:
            rospy.loginfo_throttled(60, 'Waiting for more controllers...')
            continue

        controllers = [
            controller for controller in controller_list
            if controller_re.match(controller.name)
        ]
        rospy.sleep(5)

    return controllers

