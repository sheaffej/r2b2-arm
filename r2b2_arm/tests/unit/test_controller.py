from math import pi
import time
from typing import List
import unittest

import r2b2_arm.controller

import rospy
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTolerance


class TestController(unittest.TestCase):
    def setUp(self):
        pass

    def test_calc_next_joint_position(self):
        self.assertAlmostEqual(
            r2b2_arm.controller.calc_next_joint_position(
                cur_position=0, target_position=100, loops_left=10
            ), 10
        )

        self.assertAlmostEqual(
            r2b2_arm.controller.calc_next_joint_position(
                cur_position=0, target_position=pi / 2, loops_left=3
            ), pi / 2 / 3
        )

        self.assertAlmostEqual(
            r2b2_arm.controller.calc_next_joint_position(
                cur_position=0, target_position=-pi / 2, loops_left=3
            ), -pi / 2 / 3
        )

        self.assertAlmostEqual(
            r2b2_arm.controller.calc_next_joint_position(
                cur_position=pi / 2, target_position=-pi / 2, loops_left=2
            ), 0
        )

        self.assertAlmostEqual(
            r2b2_arm.controller.calc_next_joint_position(
                cur_position=-pi / 2, target_position=0, loops_left=3
            ), ((-pi / 2) / 3) * 2
        )

    def test_calc_first_point(self):
        joint_positions = [0., 120., 120., -90., 0.]
        target_point = JointTrajectoryPoint(
            positions=[0., 90., 90., -90., 0.],
            time_from_start=rospy.Duration.from_sec(5.0)
        )
        goal_start_time = rospy.Time.from_sec(int(time.time()))
        point_start_time = goal_start_time
        loop_hz = 10
        path_tolerance = [0.] * len(joint_positions)
        result = self._loop(
            target_point=target_point,
            joint_positions=joint_positions,
            goal_start_time=goal_start_time,
            point_start_time=point_start_time,
            loop_hz=loop_hz,
            path_tolerance=path_tolerance
        )
        self.assertListEqual(result["joint_positions"], target_point.positions)

    def test_calc_second_point(self):
        joint_positions = [0., 90., 90., -90., 0.]
        target_point = JointTrajectoryPoint(
            positions=[0., 0., 45., 0., 0.],
            time_from_start=rospy.Duration.from_sec(10.0)
        )
        goal_start_time = rospy.Time.from_sec(int(time.time()))
        point_start_time = goal_start_time + rospy.Duration.from_sec(5.0)
        loop_hz = 10
        path_tolerance = [0.] * len(joint_positions)
        result = self._loop(
            target_point=target_point,
            joint_positions=joint_positions,
            goal_start_time=goal_start_time,
            point_start_time=point_start_time,
            loop_hz=loop_hz,
            path_tolerance=path_tolerance
        )
        self.assertListEqual(result["joint_positions"], target_point.positions)

    def test_calc_third_point(self):
        joint_positions = [0., 0., 45., 0., 0.]
        target_point = JointTrajectoryPoint(
            positions=[0., -45., 15., 0., 0.],
            time_from_start=rospy.Duration.from_sec(15.0)
        )
        goal_start_time = rospy.Time.from_sec(int(time.time()))
        point_start_time = goal_start_time + rospy.Duration.from_sec(5.0)
        loop_hz = 10
        path_tolerance = [0.] * len(joint_positions)
        result = self._loop(
            target_point=target_point,
            joint_positions=joint_positions,
            goal_start_time=goal_start_time,
            point_start_time=point_start_time,
            loop_hz=loop_hz,
            path_tolerance=path_tolerance
        )
        self.assertListEqual(result["joint_positions"], target_point.positions)

    def _loop(self, target_point: JointTrajectoryPoint, joint_positions,
              goal_start_time: rospy.Time, point_start_time: rospy.Time,
              loop_hz, path_tolerance: List[JointTolerance]):

        point_target_time = goal_start_time + target_point.time_from_start
        sim_time = point_start_time

        while True:
            # Are we past the point target time?
            if sim_time >= point_target_time:
                break

            # Are we at the waypoint?
            if r2b2_arm.controller.is_at_waypoint(
                joint_positions=joint_positions,
                target_point=target_point,
                path_tolerance=path_tolerance
            ):
                break

            # Advance the joints
            joint_positions = r2b2_arm.controller.calc_next_positions(
                joint_positions=joint_positions,
                target_point=target_point,
                goal_start_time=goal_start_time,
                loop_hz=loop_hz,
                now_time=sim_time
            )

            # Advance the sim clock
            sim_time = sim_time + rospy.Duration(1 / loop_hz)

        return {
            "joint_positions": joint_positions,
            "sim_time": sim_time
        }
