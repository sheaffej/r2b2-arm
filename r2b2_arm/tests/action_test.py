#! /usr/bin/env python3
# http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action#The_trajectory_message

import rospy

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction, FollowJointTrajectoryGoal,
    # FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def action_client():
    client = actionlib.SimpleActionClient("r2b2_arm", FollowJointTrajectoryAction)

    # Waits until the action server has started up and started listening for goals.
    while not client.wait_for_server(timeout=rospy.Duration(1.0)):
        rospy.logwarn("Waiting for server")
    rospy.loginfo("Server connected")

    joint_names = [
        "base_rot_j0", "arm_base_j1", "arm_mid_j2", "arm_rot_j3", "wrist_rot_j4"
    ]
    points = [
        # Start [0., 120., 120., -90., 0. ]
        JointTrajectoryPoint(  # Initial lift from stow position
            positions=[0., 90., 90., -90., 0.],
            time_from_start=rospy.Duration(1.)
        ),
        JointTrajectoryPoint(  # Project forward
            positions=[0., 0., 45., 0., 0.],
            time_from_start=rospy.Duration(1.)
        ),
        JointTrajectoryPoint(  # Project forward further
            positions=[0., -45., 15., 0., 0.],
            time_from_start=rospy.Duration(1.)
        ),
        JointTrajectoryPoint(  # Adjust to left a little
            positions=[-30., -45., 15., 0., 0.],
            time_from_start=rospy.Duration(1.)
        ),
        JointTrajectoryPoint(  # Bend down
            positions=[-30., -45., 15., 0., -45.],
            time_from_start=rospy.Duration(1.)
        ),
    ]

    trajectory = JointTrajectory(
        header=None,
        joint_names=joint_names,
        points=points
    )

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal(
        trajectory=trajectory,
        path_tolerance=None,
        goal_tolerance=None,
        goal_time_tolerance=None
    )

    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

    # Sends the goal to the action server.
    rospy.loginfo("Sending goal")
    client.send_goal(goal)
    rospy.loginfo("Sent goal")

    # Waits for the server to finish performing the action.
    while not client.wait_for_result(timeout=rospy.Duration(1.0)):
        rospy.loginfo("Waiting for result")
    rospy.loginfo("Results are ready")

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_action_client')
        result = action_client()
        # print("Result:", ', '.join([str(n) for n in result.sequence]))
        print(f"Results: {result}")

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
