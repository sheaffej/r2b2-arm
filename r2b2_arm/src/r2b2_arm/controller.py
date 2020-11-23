# http://docs.ros.org/en/noetic/api/actionlib/html/classactionlib_1_1SimpleActionServer.html

import time
from typing import List
from math import ceil

import rospy
from actionlib import SimpleActionServer

from control_msgs.msg import (
    FollowJointTrajectoryAction, FollowJointTrajectoryGoal, 
    JointTolerance
    # FollowJointTrajectoryResult, FollowJointTrajectoryFeedback
)
from trajectory_msgs.msg import JointTrajectoryPoint


def is_at_waypoint(joint_positions: List[float],
                   target_point: JointTrajectoryPoint,
                   path_tolerance: List[JointTolerance]):
    if target_point is None:
        return False

    # For each joint position in the waypoint
    for joint_idx, joint_target in enumerate(target_point.positions):
        # print(joint_idx)
        # If the joint is not within the tolerance of the target
        if abs(joint_positions[joint_idx] - joint_target) > path_tolerance[joint_idx]:
            return False
    return True


def calc_next_position(cur_position: float, target_position: float, loops_left: int):
    # loops_left will be > 0 since ceil() is used in calling method

    position_diff = target_position - cur_position
    print(f"Calc: cur_position {cur_position}, target_position {target_position}, position_diff {position_diff}, loops_left {loops_left}")

    position_this_loop = (position_diff / loops_left) + cur_position
    return position_this_loop


def calc_next_positions(joint_positions: List[float],
                        target_point: JointTrajectoryPoint,
                        goal_start_time: rospy.Time, loop_hz):

    time_left = (goal_start_time + target_point.time_from_start) - rospy.Time.now()
    if time_left < rospy.Duration(0):
        time_left = rospy.Duration(0)
    print(f"time_left {time_left.to_sec()}")

    loops_left = time_left.to_sec() / (1 / loop_hz)
    print(f"loops_left {loops_left}")
    loops_left = ceil(loops_left)
    print(f"loops_left {loops_left}")

    new_positions = [0] * len(joint_positions)

    # For each joint
    for joint_idx in range(len(joint_positions)):
        new_positions[joint_idx] = calc_next_position(
            joint_positions[joint_idx], target_point.positions[joint_idx], loops_left
        )
        print(f"Joint {joint_idx} is at {joint_positions[joint_idx]} and needs to be at {target_point.positions[joint_idx]} so it will move to {new_positions[joint_idx]}")
    return new_positions


def command_joints(positions: List[float]):
    for joint_idx, position in enumerate(positions):
        print(f"Moving joint {joint_idx} to {position}")


class R2B2ArmController:

    def __init__(self, initial_joint_positions: List[float], path_tolerance: List[float], loop_hz: float):
        self.goal_id = None
        self.joint_positions = initial_joint_positions
        self.path_tolerance = path_tolerance
        self.loop_hz = loop_hz

        self.action_server = SimpleActionServer(
            'r2b2_arm', FollowJointTrajectoryAction, self.execute, False
        )
        self.action_server.start()

    def execute(self, goal: FollowJointTrajectoryGoal):
        print(goal)

        # Check for same number of joints
        if len(goal.trajectory.joint_names) != len(self.joint_positions):
            message = f"Trajectory has {len(goal.trajectory.joint_names)} joints but controller has {len(self.joint_positions)}"
            rospy.logerr(message)
            self.action_server.set_aborted(test=message)

        start_time = goal.trajectory.header.stamp
        # join_names = goal.trajectory.joint_names
        traj_points = goal.trajectory.points

        if len(goal.path_tolerance) > 0:
            self.path_tolerances = goal.path_tolerance

        goal_end_time = goal.trajectory.points[len(goal.trajectory.points) - 1].time_from_start + start_time

        # cur_point = traj_points.pop(0)  # type: JointTrajectoryPoint
        cur_point = None  # type: JointTrajectoryPoint

        while self.action_server.is_active():
            # The execute() method will run one goal at a time until completion.
            # So if there is another goal waiting we should prempt and return to
            # work on the new goal.
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Prempt requested")
                self.action_server.set_preempted()
                return

            # If we have completed the current point (or no point yet)
            if cur_point is None or is_at_waypoint(self.joint_positions, cur_point, self.path_tolerance):
                if len(traj_points) > 0:
                    # Get the next point
                    cur_point = traj_points.pop(0)  # type: JointTrajectoryPoint
                    rospy.loginfo(f"New waypoint for time {cur_point.time_from_start}")
                    rospy.loginfo(cur_point.positions)
                else:
                    # We have finished the trajectory
                    rospy.loginfo("Trajectory complete. Setting result as succeeded")
                    self.action_server.set_succeeded()
                    return

            if rospy.Time.now() >= goal_end_time:
                self.action_server.set_aborted(
                    text=f"Goal time tolerance exceeded: goal({goal_end_time}), now({rospy.Time.now()})"
                )

            # Calculate where to move each joint for the next loop
            next_positions = calc_next_positions(
                self.joint_positions, cur_point, start_time, self.loop_hz
            )
            print(f"Next positions: {next_positions}")

            # Command each joint to move
            command_joints(next_positions)
            self.joint_positions = next_positions

            # Optionally send current state of joints as feedback

            # Sleep until next loop
            time.sleep(1 / self.loop_hz)
            print()

        print("Goal is no longer active")
