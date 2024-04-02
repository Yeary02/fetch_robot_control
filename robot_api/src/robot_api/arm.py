import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class Arm(object):
    """Arm controls the robot's arm."""

    def __init__(self):
        # Assuming you've already created an actionlib client for the arm's action server
        self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for arm controller action server...")
        self.client.wait_for_server()
        rospy.loginfo("Arm controller action server found!")

    def move_to_joints(self, arm_joints):
        # Retrieve the joint names and joint positions using the methods defined in ArmJoints
        joint_names = arm_joints.names()
        joint_positions = arm_joints.values()

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rospy.Duration(5.0)  # Trajectories take 5 seconds each

        # Create a goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points.append(point)

        # Send the goal to the action server
        self.client.send_goal(goal)
        rospy.loginfo("Sending goal to arm controller...")
        self.client.wait_for_result()
        rospy.loginfo("Goal execution completed.")
