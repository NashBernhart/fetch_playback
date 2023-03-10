#!/usr/bin/env python

import rospy
import json
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface

if __name__ == '__main__':
    rospy.init_node("state_playback")

    move_group = MoveGroupInterface("arm_with_torso", "base_link")

    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]

    with open('output.json') as user_file:
        poses_object = json.load(user_file)

    poses = poses_object["poses"]
    durations = poses_object["durations"]

    for idx, pose in enumerate(poses):

        move_group.moveToJointPosition(joint_names, pose, wait=True)
        rospy.sleep(durations[idx])

    move_group.get_move_action().cancel_all_goals()
    