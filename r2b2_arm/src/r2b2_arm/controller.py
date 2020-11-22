# http://docs.ros.org/en/noetic/api/actionlib/html/classactionlib_1_1SimpleActionServer.html

import rospy
from actionlib import SimpleActionServer

from control_msgs.msg import (
    FollowJointTrajectoryAction, FollowJointTrajectoryGoal, 
    # FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
)


class R2B2ArmController:

    def __init__(self):
        self.goal_id = None
        self.is_executing = False

        self.action_server = SimpleActionServer(
            'r2b2_arm', FollowJointTrajectoryAction, self.execute, False
        )
        self.action_server.start()

    def execute(self, action_goal: FollowJointTrajectoryGoal):
        if self.is_executing:
            # We need to prempt?
            raise RuntimeError("execute() called while already executing")

        print(action_goal)
        # start_time = action_goal.

            # if self.server.is_preempt_requested():
            #     rospy.loginfo('%s: Preempted' % self._action_name)
            #     self._as.set_preempted()
            #     success = False
            #     break


        # action_goal.trajectory.points.

        self.action_server.set_succeeded()
