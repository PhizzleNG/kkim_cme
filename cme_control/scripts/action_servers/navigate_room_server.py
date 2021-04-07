#!/usr/bin/env python2.7
# -*- encoding: utf-8 -*-
'''
/opt/ros/kinetic/share/actionlib_msgs/msg/GoalStatus.msg
'''
# TODO: This would be better implemented as a local planner,
#       using the costmap to determine obstacles
from __future__ import print_function

import actionlib
import math
import os
import numpy
import re
import rospy
import sys
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from cme_control.srv import DoorClose, DoorOpen
from geometry_msgs.msg import Point, Pose, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
from cme_control.msg import NavigateRoomAction, NavigateRoomFeedback, NavigateRoomResult

sys.path.append(os.path.join(os.path.dirname(sys.path[0])))
from common import find_links, find_joints

def get_room_links(namespace='/world'):
    room_link_re = re.compile('room_([0-9]+)')
    return find_links(room_link_re, namespace=namespace)

def get_room_joints(namespace='/world'):
    room_joint_re = re.compile('room_([0-9]+)_joint')
    return {
        joint.name.rstrip('_joint'): joint
        for joint in find_joints(room_joint_re, namespace=namespace)
    }

def get_door_joints(namespace='/world'):
    door_joint_re = re.compile('door_([0-9]+)_frame_joint')
    return {
        joint.name.rstrip('_frame_joint'): joint
        for joint in find_joints(door_joint_re, namespace=namespace)
    }

# for each point in the planner, check if points are around a known door
# if they are, split the planner at that index and move the "door" goal towards the last point

def urdf_pose_to_pose(pose):
    geo_pose = Pose()
    geo_pose.position = Point(*pose.xyz)
    geo_pose.orientation = Quaternion(*quaternion_from_euler(*pose.rpy))

    return geo_pose

def get_points_in_area(input_points, reference_point, tolerance=1):
    # TODO: Tolerance should be robot tolerance + manipulator
    max_x = reference_point.x + tolerance
    min_x = reference_point.x - tolerance
    max_y = reference_point.y + tolerance
    min_y = reference_point.y - tolerance
    output_points = {}
    for index, point in enumerate(input_points):
        if point.position.x > max_x or point.position.x < min_x:
            continue
        if point.position.y > max_y or point.position.y < min_y:
            continue
        output_points[index] = point

    return output_points

def point_to_numpy(points):
    return numpy.array(
        [
            [point.x, point.y, point.z]
            for point in points
        ]
    )

def get_angle(from_point, to_point):
    if isinstance(from_point, Pose):
        from_point = from_point.position
    if isinstance(to_point, Pose):
        to_point = to_point.position
    if isinstance(from_point, Point):
        from_point = [from_point.x, from_point.y]
    if isinstance(to_point, Point):
        to_point = [to_point.x, to_point.y]
    return (numpy.arctan2(*from_point[::-1]) - numpy.arctan2(*to_point[::-1])) % (2 * numpy.pi)

def get_closest_path_intersection(path_poses, door_point):
    # TODO: This should find out if path intersects door
    #       If it does, we should find point in front of the door
    #       such that the door will not swing into the robot
    #       and is within the costmap
    near_points = get_points_in_area(path_poses, door_point)
    if not near_points:
        return None, None
    # TODO: Workaround, find direction of points and return reasonable path in front of door
    if not get_points_in_area(near_points.values(), door_point, tolerance=0.1):
        # no points found intersecting door
        return None, None
    return near_points.items()[0]

    if len(near_points) < 3:
        return near_points.items()[0]
    np_points = point_to_numpy(near_points.values())
    begin = np_points[0]
    end = np_points[-1]
    dist_sq = [
        numpy.abs(numpy.cross(pt-begin, begin-end)) / numpy.linalg.norm(pt/begin)
        for pt in np_points[1:-1]
    ]
    maxdist = max(dist_sq)
    index = dist_sq.index(maxdist)
    return near_points[index]

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
        goal.target_pose.header.frame_id = 'map'
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

        self._plan = None

        self._move_base = MoveBaseClient('move_base')

        self._sub_status = rospy.Subscriber(
            '/move_base/status', GoalStatusArray, self._status_cb, queue_size=10,
        )
        self._sub_plan = rospy.Subscriber(
            '/move_base/NavfnROS/plan', Path, self._plan_cb, queue_size=10,
        )

        # allow time to initialize
        self._move_base.wait_for_server()

        self._as.start()

        self.door_joints = get_door_joints()
        self.room_joints = get_room_joints()

        if not self.room_joints:
            rospy.logfatal('%s: Could not find any room joints!', self._action_name)
            rospy.signal_shutdown('fatal')
            return

        rospy.loginfo('%s: Found %d room%s!', self._action_name, len(self.room_joints),
            's' if self.room_joints != 1 else '')
        rospy.loginfo('%s: Found %d door%s!', self._action_name, len(self.door_joints),
            's' if self.door_joints != 1 else '')

        rospy.loginfo('%s: Started!', self._action_name)

    def _check_preempt(self):
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted', self._action_name)
            self._as.set_preempted()
            success = False
            return True
        return False

    def _succeed(self, success):
        if success:
            rospy.loginfo('%s: Succeeded', self._action_name)
            self._as.set_succeeded(self._result)
        return success

    def _status_cb(self, msg):
        pass

    def _plan_cb(self, msg):
        # TODO: If not active, pass
        self._plan = msg

    def send_goal(self, pose):
        if isinstance(pose, Point):
            _pose = Pose()
            _pose.position = pose
            pose = _pose
        pose.position.z = 0
        self._move_base.send_goal(pose)

    def open_door(self, door_name):
        # TODO: topic lookup
        rospy.loginfo('%s: opening %s', self._action_name, door_name)
        service_name = '/world/{}/open'.format(door_name)
        service = rospy.ServiceProxy(service_name, DoorOpen)
        service()

    def execute_cb(self, goal):
        # TODO: Errors should be a terminal status
        # TODO: Clean-up and restructure
        # TODO: door clearance
        # TODO: Cancel move_base goal: rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}
        r = rospy.Rate(5)
        success = False

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

        self._plan = None
        self.send_goal(urdf_pose_to_pose(room_joint.origin))

        # wait for path plan
        while True:
            if self._plan:
                if self._plan.poses:
                    break
            # TODO: Check if DWA planner fails?
            if self._check_preempt():
                return self._succeed(success)
            rospy.loginfo_throttle(30, "%s: waiting for path plan" % self._action_name)
            r.sleep()

        plan_poses = [p.pose for p in self._plan.poses]
        near_door_points = {}
        for name, door_joint in self.door_joints.items():
            position = urdf_pose_to_pose(door_joint.origin).position
            # TODO: simplify
            index, stop_point = get_closest_path_intersection(plan_poses, position)
            if stop_point:
                near_door_points[name] = index


        # TODO: This should be a queue
        for door, index in sorted(near_door_points.items(), key=lambda x: x[1]):
            rospy.loginfo('%s: navigating to %s', self._action_name, door)
            _pose = plan_poses[index]
            # make sure we face the door
            _pose.orientation = Quaternion(*quaternion_from_euler(
                0, 0,
                get_angle(
                    urdf_pose_to_pose(self.door_joints[door].origin),
                    plan_poses[index]
                )
            ))
            self.send_goal(_pose)
            self._move_base.wait_for_result()
            if self._move_base.get_state() != GoalStatus.SUCCEEDED:
                rospy.logerr('%s: Failed to navigate to %s, state: %d', self._action_name, door, self._move_base.get_state())
                success = False
                return self._succeed(success)
            self.open_door(door)

        self.send_goal(urdf_pose_to_pose(room_joint.origin))

        while True:
            # TODO: Timeout?
            # TODO: fill in feedback
            # TODO: subscribe to /move_base/status
            if self._check_preempt():
                return self._succeed(success)
            state = self._move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                success = True
                break
            if state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
                success = False
                break
            self._as.publish_feedback(self._feedback)
            r.sleep()

        return self._succeed(success)

def main():
    rospy.init_node('navigate_room', anonymous=False)

    server = NavigateRoomServer(rospy.get_name())

    rospy.spin()

if __name__ == '__main__':
    main()
