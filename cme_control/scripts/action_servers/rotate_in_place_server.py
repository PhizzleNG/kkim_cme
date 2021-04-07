#!/usr/bin/env python2.7
# -*- encoding: utf-8 -*-
'''
'''
from __future__ import print_function

import actionlib
import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cme_control.msg import RotateInPlaceAction, RotateInPlaceFeedback, RotateInPlaceResult

class RotateInPlaceServer(object):
    _feedback = RotateInPlaceFeedback()
    _result = RotateInPlaceResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            RotateInPlaceAction,
            execute_cb=self.execute_cb,
            auto_start = False,
        )

        self._start_angle = 0
        self._odom = None
        self._sub_odom = rospy.Subscriber(
            '/odometry/filtered', Odometry, self._odom_cb, queue_size=1,
        )

        self._pub_cmd_vel = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=10,
        )

        rospy.on_shutdown(lambda: self.publish_twist(xyz='0 0 0', rpy='0 0 0'))

        # allow time to initialize
        rospy.sleep(0.5)
        while self._sub_odom.get_num_connections() == 0:
            rospy.loginfo_throttle(60, "Waiting for publisher on " + self._sub_odom.name)
            rospy.sleep(1)

        self._as.start()

        rospy.loginfo("%s: Started!", self._action_name)

    def _odom_cb(self, msg):
        self._odom = msg

    def publish_twist(self, xyz='0 0 0', rpy='0 0 0'):
        vel = Twist()
        vel.linear.x, vel.linear.y, vel.linear.z = map(int, xyz.split(' '))
        vel.angular.x, vel.angular.y, vel.angular.z = map(int, rpy.split(' '))

        self._pub_cmd_vel.publish(vel)

    def execute_cb(self, goal):
        r = rospy.Rate(5)
        success = True

        rospy.loginfo(
            '%s: Attempting to rotate for %d second%s',
            self._action_name, goal.seconds, 's' if goal.seconds != 1 else ''
        )

        # Start time
        start_time = rospy.Time.now()

        # Start rotating
        increment = False
        while (rospy.Time.now() - start_time).secs < goal.seconds:
            # Rotate
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted', self._action_name)
                self._as.set_preempted()
                success = False
                break
            _, _, yaw = euler_from_quaternion(
                [self._odom.pose.pose.orientation.x, self._odom.pose.pose.orientation.y,
                 self._odom.pose.pose.orientation.z, self._odom.pose.pose.orientation.w]
            )
            if not self._start_angle:
                self._start_angle = yaw
            if increment and yaw + math.pi > self._start_angle + math.pi:
                self._result.rotations += 1
                increment = False
            elif not increment and yaw + math.pi < self._feedback.angle + math.pi:
                increment = True
            self._feedback.angle = yaw
            self.publish_twist(xyz='0 0 0', rpy='0 0 1')
            self._as.publish_feedback(self._feedback)
            r.sleep()

        self.publish_twist(xyz='0 0 0', rpy='0 0 0')

        if success:
            rospy.loginfo('%s: Succeeded with %d rotations', self._action_name, self._result.rotations)
            self._as.set_succeeded(self._result)

def main():
    rospy.init_node('rotate_in_place', anonymous=False)

    server = RotateInPlaceServer(rospy.get_name())

    rospy.spin()

if __name__ == '__main__':
    main()
