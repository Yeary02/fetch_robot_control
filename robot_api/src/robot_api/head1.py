#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped
from actionlib_msgs.msg import GoalStatus
import math

LOOK_AT_ACTION_NAME = '/head_controller/point_head'  # Update with actual action name
PAN_TILT_ACTION_NAME = '/head_controller/follow_joint_trajectory'  # Update with actual action name
PAN_JOINT = 'head_pan_joint'  # Update with actual joint name
TILT_JOINT = 'head_tilt_joint'  # Update with actual joint name

class Head(object):
    MIN_PAN = -math.pi  # Minimum pan angle, in radians. Update with actual limit
    MAX_PAN = math.pi  # Maximum pan angle, in radians. Update with actual limit
    MIN_TILT = -math.pi/4  # Minimum tilt angle, in radians. Update with actual limit
    MAX_TILT = math.pi/2  # Maximum tilt angle, in radians. Update with actual limit

    def __init__(self):
        self.look_at_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, FollowJointTrajectoryAction)
        self.pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        
        rospy.loginfo("Waiting for head controller action servers...")
        self.look_at_client.wait_for_server()
        self.pan_tilt_client.wait_for_server()
        rospy.loginfo("Head controller action servers found")

    def look_at(self, frame_id, x, y, z):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.point.x = x
        point.point.y = y
        point.point.z = z

        # You need to fill in the details of creating and sending a goal
        # to the look_at action server, depending on how it's implemented.
        rospy.logerr('look_at method not implemented.')

    def pan_tilt(self, pan, tilt):
        if not (self.MIN_PAN <= pan <= self.MAX_PAN) and (self.MIN_TILT <= tilt <= self.MAX_TILT):
            rospy.logerr("Requested pan/tilt angles are out of bounds.")
            return

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = [PAN_JOINT, TILT_JOINT]
        
        point = JointTrajectoryPoint()
        point.positions = [pan, tilt]
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)
        goal.trajectory.points.append(point)

        self.pan_tilt_client.send_goal(goal)
        rospy.loginfo("Setting pan/tilt angles to pan: {:.2f}, tilt: {:.2f}".format(pan, tilt))
        self.pan_tilt_client.wait_for_result()
        rospy.loginfo("Pan/tilt angles set.")

