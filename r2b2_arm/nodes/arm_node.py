#! /usr/bin/env python3
# http://docs.ros.org/en/noetic/api/actionlib/html/classactionlib_1_1SimpleActionServer.html

import rospy
# import actionlib
# from control_msgs.msg import FollowJointTrajectoryAction

from r2b2_arm.controller import R2B2ArmController

DEFAULT_NODE_NAME = "arm_controller"

# Subscribes

# Publishes

# Default Parameters
DEFAULT_LOG_LEVEL = "info"


def parse_log_level(levelstr):
    map = {
        "debug": rospy.DEBUG,
        "info": rospy.INFO,
        "warn": rospy.WARN,
        "error": rospy.ERROR,
        "fatal": rospy.FATAL
    }
    return map.get(levelstr.lower())


if __name__ == '__main__':
    log_level = parse_log_level(rospy.get_param("~log_level", DEFAULT_LOG_LEVEL))
    rospy.init_node(DEFAULT_NODE_NAME, log_level=log_level)
    node_name = rospy.get_name()
    rospy.loginfo("Node is starting")

    initial_joint_positions = [0., 120., 120., -90., 0.]
    default_path_tolerance = [0.1] * len(initial_joint_positions)
    loop_hz = 10

    controller = R2B2ArmController(
        initial_joint_positions=initial_joint_positions,
        path_tolerance=default_path_tolerance,
        loop_hz=loop_hz
    )
    rospy.loginfo("Controller created")

    rospy.spin()
