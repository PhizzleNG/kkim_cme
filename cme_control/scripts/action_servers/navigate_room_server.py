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
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from cme_control.srv import DoorClose, DoorOpen, LightOn, LightOff
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
    return (numpy.arctan2(*from_point[::-1]) - numpy.arctan2(*to_point[::-1])) % (2 * numpy.pi) - numpy.pi

def get_closest_path_intersection(path_poses, door_position, initial_tolerance=1, door_tolerance=0.2):
    # TODO: This should find out if path intersects door
    #       If it does, we should find point in front of the door
    #       such that the door will not swing into the robot
    #       and is within the costmap
    near_points = get_points_in_area(path_poses, door_position, tolerance=initial_tolerance)
    if not near_points:
        return None, None
    # TODO: Workaround, find direction of points and return reasonable path in front of door
    if not get_points_in_area(near_points.values(), door_position, tolerance=door_tolerance):
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

class StateError(Exception):
    pass

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

        self._current_goal = None
        self._current_target = None
        self._plan = None
        self._plan_queue = list()
        self._plan_poses = None

        self._r = rospy.Rate(5)

        self._move_base = MoveBaseClient('move_base')

        self._sub_status = rospy.Subscriber(
            '/move_base/status', GoalStatusArray, self._status_cb, queue_size=10,
        )
        self._sub_plan = rospy.Subscriber(
            '/move_base/NavfnROS/plan', Path, self._plan_cb, queue_size=10,
        )

        self._pub_cancel_goal = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # allow time to initialize
        self._move_base.wait_for_server()

        self._as.start()

        # TODO: Dynamic reconfigure
        self.door_radius_tolerance = rospy.get_param('door_radius_tolerance', 1)
        self.door_path_tolerance = rospy.get_param('door_path_tolerance', 0.15)

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

        rospy.on_shutdown(self._cancel_goal)

        rospy.loginfo('%s: Started!', self._action_name)

    def _check_preempt(self):
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted', self._action_name)
            self._as.set_preempted()
            return True
        return False

    def _cancel_goal(self):
        # TODO: Clear path in rviz
        self._pub_cancel_goal.publish(GoalID())

    def _succeed(self, success):
        # TODO: Send a proper goal state termination
        # TODO: Cancel move_base goal: rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}
        self._cancel_goal()
        if success:
            self._as.set_succeeded(self._result)
        elif not self._as.is_preempt_requested() and self._as.is_active():
            self._as.set_aborted()
        rospy.loginfo('%s: %s', self._action_name,
            "Succeeded" if success else "Failed")
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
        try:
            service()
        except rospy.service.ServiceException:
            rospy.logerr('%s doesn\'t exist!', door_name.title())

    def turn_on_light(self, room_id):
        # TODO: Check if light exists?
        light_id = room_id
        rospy.loginfo('%s: turning on light %s', self._action_name, light_id)
        service_name = '/world/light_{}/on'.format(light_id)
        try:
            service = rospy.ServiceProxy(service_name, LightOn)
            service()
        except rospy.service.ServiceException:
            rospy.logerr('Light %d doesn\'t exist!', light_id)
        except Exception:
            rospy.logerr('Encountered an issue turning on light %d!', light_id, exc_info=1)

    def get_room_origin(self, room):
        room_joint = self.room_joints.get(room, None)
        if not room_joint:
            rospy.logerr('%s: %s does not exist!', self._action_name, room)
            return None

        if not room_joint.origin:
            ropsy.logerr('%s: %s does not have an origin!', self._action_name, room)
            return None

        return room_joint.origin

    def wait_for_plan(self):
        # TODO: Check if DWA planner fails?
        if self._plan:
            if self._plan.poses:
                return True
        rospy.loginfo_throttle(30, "%s: waiting for path plan" % self._action_name)
        return False

    def find_points_near_doors(self, poses=None):
        if not poses:
            poses = [p.pose for p in self._plan.poses]
        self._plan_poses = [p.pose for p in self._plan.poses]
        near_door_points = {}
        for name, door_joint in self.door_joints.items():
            door_pose = urdf_pose_to_pose(door_joint.origin)
            # TODO: simplify
            index, stop_point = get_closest_path_intersection(
                self._plan_poses, door_pose.position, self.door_radius_tolerance, self.door_path_tolerance
            )
            if stop_point:
                near_door_points[name] = index
        return near_door_points

    def build_nav_point_queue(self, door_points, point_queue):
        for door, index in sorted(door_points.items(), key=lambda x: x[1]):
            _pose = self._plan_poses[index]
            door_pose = urdf_pose_to_pose(self.door_joints[door].origin)
            # make sure we face the door
            #_pose.orientation = Quaternion(*quaternion_from_euler(
            #    0, 0,
            #    get_angle(door_pose, _pose) + (numpy.pi / 2)
            #))

            # HACKY - get the point "in front of" the door
            if door_pose.orientation.w == 1.0:
                _pose.position.x = door_pose.position.x
                sign = 1 if door_pose.position.x > _pose.position.x else -1
                _pose.orientation = Quaternion(*quaternion_from_euler(
                    0, 0, sign * math.pi/2
                ))
            else:
                # Just assume that the door is rotated
                _pose.position.y = door_pose.position.y
                _pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))

            point_queue.append((_pose, door))

        return point_queue

    def _process_queue(self):
        if not self._current_goal:
            self._current_goal, self._current_target = self._path_queue.pop(0)
            self._feedback.current_goal = self._current_goal
            self.send_goal(self._current_goal)
            return False
        rospy.loginfo_throttle(60, '%s: navigating to %s' % (self._action_name, self._current_target))
        state = self._move_base.get_state()
        self._feedback.move_base_state = state
        if state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST]:
            rospy.logerr('%s: Failed to navigate to %s, state: %d',
                self._action_name, self._current_target, state)
            raise StateError(state)
        if state != GoalStatus.SUCCEEDED:
            return False
        if self._current_target.startswith('door'):
            self.open_door(self._current_target)
        self._current_goal = None
        self._current_target = None
        return True

    def process_queue(self):
        try:
            self._process_queue()
            return False
        except StateError:
            # move_base failed
            raise
        except IndexError:
            # list queue is empty
            pass
        return True

    def execute_fn(self, fn):
        # TODO: Timeout?
        try:
            while not rospy.is_shutdown():
                if self._check_preempt():
                    return self._succeed(False)
                if fn():
                    # True will indicate fn() is complete, breaking the loop
                    return True
                self._as.publish_feedback(self._feedback)
                self._r.sleep()
        except StateError:
            return False
        except Exception:
            self._succeed(False)
            raise
        return False

    def execute_cb(self, goal):
        # TODO: Errors should be a terminal status
        # TODO: door clearance
        self._current_goal = None
        self._current_target = None
        self._plan_queue = list()
        self._plan_paths = None

        room = 'room_' + str(goal.room)
        room_origin = urdf_pose_to_pose(self.get_room_origin(room))
        if not room_origin:
            return

        rospy.loginfo(
            '%s: Attempting to go to room %d',
            self._action_name, goal.room,
        )

        # Start time
        start_time = rospy.Time.now()

        # Ensure the plan and queue are empty
        self._plan = None
        self._path_queue = list()

        # Send target room goal for the full path to be generated
        self.send_goal(room_origin)

        self._feedback.total_goals = len(self._path_queue)

        if not self.execute_fn(self.wait_for_plan):
            return self._succeed(False)

        # Cancel initial goal and continue with our own plan
        self._cancel_goal()

        # Make sure we retain the intial _plan
        _plan = self._plan

        self._result.goal = _plan.poses[-1].pose

        # Check the path for doors, break the path up if needed and queue goals
        near_door_points = self.find_points_near_doors()
        self.build_nav_point_queue(near_door_points, self._path_queue)

        # Make sure target goal is last
        self._path_queue.append((room_origin, room))

        self._feedback.total_goals = len(self._path_queue)

        if not self.execute_fn(self.process_queue):
            return self._succeed(False)

        # We're at the room, turn on the light
        if not goal.skip_light:
            self.turn_on_light(goal.room)

        return self._succeed(True)

def main():
    rospy.init_node('navigate_room', anonymous=False)

    server = NavigateRoomServer(rospy.get_name())

    rospy.spin()

if __name__ == '__main__':
    main()
