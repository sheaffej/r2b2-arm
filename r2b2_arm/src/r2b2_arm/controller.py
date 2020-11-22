# http://docs.ros.org/en/noetic/api/actionlib/html/classactionlib_1_1SimpleActionServer.html

import time

import rospy
from actionlib import SimpleActionServer

from control_msgs.msg import (
    FollowJointTrajectoryAction, FollowJointTrajectoryGoal, 
    # FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
)


class R2B2ArmController:

    def __init__(self):
        self.goal_id = None

        self.action_server = SimpleActionServer(
            'r2b2_arm', FollowJointTrajectoryAction, self.execute, False
        )
        self.action_server.start()

    def execute(self, goal: FollowJointTrajectoryGoal):
        print(goal)

        if self.is_executing:
            # We need to prempt?
            raise RuntimeError("execute() called while already executing")

        start_time = goal.trajectory.header.stamp
        join_names = goal.trajectory.joint_names
        traj_points = goal.trajectory.points

        for i in range(20):
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Prempt requested")
                self.action_server.set_preempted()
                return
            
            print(f"Doing stuff for {i} secs")
            time.sleep(1.0)

        # Loop
            # If we have completed the current point
                # Get the next point
                # If at the goal pose, send result message
                    # return
        
            # Calculate where to move each joint for the next loop

            # Command each joint to move

            # Optionally send current state of joints as feedback

            # Sleep until next loop

        print("Done. Setting result as succeeded")
        self.action_server.set_succeeded()
