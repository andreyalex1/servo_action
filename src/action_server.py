#! /usr/bin/env python3

import rospy
import actionlib
import time
from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction 
from trajectory_msgs.msg import JointTrajectory

from moveit_msgs.msg import ExecuteTrajectoryActionGoal
from moveit_msgs.msg import ExecuteTrajectoryActionFeedback
from moveit_msgs.msg import ExecuteTrajectoryActionResult
from moveit_msgs.msg import ExecuteTrajectoryAction

from trajectory_msgs.msg import JointTrajectory


class servo_action_server:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction, self.execute, False)
        self.server.start()

    def execute(self, goal):
        print("123")
        print(goal.data)
   #     time.delay(3)
        self.server.set_succeeded()

class servo_execute_trajectory_action:
    def __init__(self):
        self.server = actionlib.SimpleActionServer("execute_trajectory", ExecuteTrajectoryAction, self.execute, False)
        self.server.start()
        self.pub = rospy.Publisher("Servo_Control", JointTrajectory, queue_size=1)

    def execute(self, goal):
        self.pub.puplish(goal.trajectory.joint_trajectory)
        self.server.set_succeeded()

if (__name__ == "__main__"):
    print("456")
    rospy.init_node("servo_action_server") 
    server = servo_action_server()
    server_2 = servo_execute_trajectory_action()
    rospy.spin()