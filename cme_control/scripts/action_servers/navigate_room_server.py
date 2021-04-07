#!/usr/bin/env python2.7
# -*- encoding: utf-8 -*-
'''
/opt/ros/kinetic/share/actionlib_msgs/msg/GoalStatus.msg
'''
from __future__ import print_function

import actionlib
import math
import os
import re
import rospy
import sys
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from cme_control.msg import NavigateRoomAction, NavigateRoomFeedback, NavigateRoomResult

sys.path.append(os.path.join(os.path.dirname(sys.path[0])))
from common import find_links, find_joints

def get_room_links(namespace='/world'):
    room_link_re = re.compile('room_([0-9]+)')
    return find_links(room_link_re, namespace=namespace)

def get_room_joints(namespace='/world'):
    room_joint_re = re.compile('room_([0-9]+)_joint')
    return find_joints(room_joint_re, namespace=namespace)

def urdf_pose_to_pose(pose):
    geo_pose = Pose()
    geo_pose.position = Point(*pose.xyz)
    geo_pose.orientation = Quaternion(*quaternion_from_euler(*pose.rpy))

    return geo_pose

class MoveBaseClient(object):
    def __init__(self, name):
        self._action_name = name
        self._client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    def wait_for_server(self):
        rospy.loginfo('Waiting for move_base action server to start')
        self._client.wait_for_server()

    @property
    def client(self):
        return self._client

    def send_goal(self, target_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = target_pose

        rospy.loginfo('Sending goal')
        self._client.send_goal(goal)

    def get_state(self):
        return self._client.get_state()

    def wait_for_result(self):
        # TODO: what action state is "idle"?
        self._client.wait_for_result()
        # Check state against actionlib.SimpleGoalState
        return self._client.get_state()

class NavigateRoomServer(object):
    _feedback = NavigateRoomFeedback()
    _result = NavigateRoomResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            NavigateRoomAction,
            execute_cb=self.execute_cb,
            auto_start = False,
        )

        self._move_base = MoveBaseClient('move_base')

        # get path from amcl
        # if path from planner intersects a door, split path in 2 at door point
        #   need some distance away from door...
        # navigate to door, check if door blocks
        # open door if blocking
        # navigate to end point

        self._sub_status = rospy.Subscriber(
            '/move_base/status', GoalStatusArray, self._status_cb, queue_size=10,
        )

        # allow time to initialize
        self._move_base.wait_for_server()

        self._as.start()

        self.room_joints = {joint.name.strip('_joint'): joint for joint in get_room_joints()}

        if not self.room_joints:
            rospy.logfatal('%s: Could not find any room joints!', self._action_name)
            rospy.signal_shutdown('fatal')
            return

        rospy.loginfo('%s: Found %d room%s!', self._action_name, len(self.room_joints),
            's' if self.room_joints != 1 else '')

        rospy.loginfo('%s: Started!', self._action_name)

    def _status_cb(self, msg):
        pass

    def execute_cb(self, goal):
        # TODO: Errors should be a terminal status
        r = rospy.Rate(5)
        success = True

        room = 'room_' + str(goal.room)
        room_joint = self.room_joints.get(room, None)
        if not room_joint:
            rospy.logerr('%s: %s does not exist!', self._action_name, room)
            return

        if not room_joint.origin:
            ropsy.logerr('%s: %s does not have an origin!', self._action_name, room)
            return

        rospy.loginfo(
            '%s: Attempting to go to room %d',
            self._action_name, goal.room,
        )

        # Start time
        start_time = rospy.Time.now()

        self._move_base.send_goal(urdf_pose_to_pose(room_joint.origin))
        while True:
            # TODO: Timeout?
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted', self._action_name)
                self._as.set_preempted()
                success = False
                break
            # TODO: fill in feedback
            # TODO: subscribe to /move_base/status
            state = self._move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                break
            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
                success = False
                break
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            rospy.loginfo('%s: Succeeded', self._action_name)
            self._as.set_succeeded(self._result)

def main():
    rospy.init_node('navigate_room', anonymous=False)

    server = NavigateRoomServer(rospy.get_name())

    rospy.spin()

if __name__ == '__main__':
    main()
