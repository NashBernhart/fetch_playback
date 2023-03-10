#!/usr/bin/env python

import rospy
import json
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from sensor_msgs.msg import JointState

def add_a_pose(pose_list, time_list, group):
    pose = []
    for joint_angle in group.get_currrent_joint_values():
        pose.append(round(joint_angle, 3))
        pose_list.append(pose)
    print("\nEnter how long you would like to hold this pose for\n")
    pose_dur = input("Time in seconds: ")
    time_list.append(pose_dur)

def save_movement(pose_list, time_list):
    rospy.loginfo('writing to file')
    #file_name = input('name your movement file: ')
    dictionary = {
        "poses": pose_list,
        "durations": time_list
    }
    json_object = json.dumps(dictionary, indent=4)
    with open("output.json", 'w') as outfile:
        outfile.write(json_object)

def playback():
    # create the joint names arr
    joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
    #open the file to be read from and load into json object
    with open('output.json') as user_file:
        poses_object = json.load(user_file)
    # get the arrays of poses to move to and times to hold
    poses = poses_object["poses"]
    durations = poses_object["durations"]
    # perform the actions
    for idx, pose in enumerate(poses):
        move_group.moveToJointPosition(joint_names, pose, wait=True)
        rospy.sleep(durations[idx])
    # finished. cancel all movements
    move_group.get_move_action().cancel_all_goals()


if __name__ == '__main__':
    rospy.init_node("state_playback")

    # init the move group and robot commander objects
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    robot = moveit_commander.RobotCommander()
    group_name = "arm_with_torso"
    group = moveit_commander.MoveGroupCommander(group_name)

    # init the planning scene
    planning_scene = PlanningSceneInterface("base_link")
    planning_scene.removeCollisionObject("my_front_ground")
    planning_scene.removeCollisionObject("my_back_ground")
    planning_scene.removeCollisionObject("my_right_ground")
    planning_scene.removeCollisionObject("my_left_ground")
    planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
    planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
    planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
    planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
    
    control_selection = 0
    while(True):
        print("\n\nEnter 1 to record a movement\nEnter 2 to playback a pose\nEnter 3 to quit\n")
        control_selection = input("Choose an option: ")

        if control_selection == 1:
            # init the two arrays that will be used to store the movement
            # pose_arr is an array of arrays where each sub array is a pose. duration array is how long to hold each pose
            pose_arr = []
            duration_arr =[]
            while(True):
                print("\nEnter 1 to save a pose in the movement. Enter 2 to finish and save the movement to a file.\nEnter 3 to quit without saving the movement\n")
                pose_selection = input("Selection: ")
                if pose_selection == 1:
                    #add_a_pose(pose_arr, duration_arr, group)
                    pose = []
                    joints_arr = group.get_currrent_joint_values()
                    for joint_angle in joints_arr:
                        pose.append(round(joint_angle, 3))
                        pose_arr.append(pose)
                    print("\nEnter how long you would like to hold this pose for\n")
                    pose_dur = input("Time in seconds: ")
                    duration_arr.append(pose_dur)
                elif pose_selection == 2:
                    save_movement(pose_arr, duration_arr)
                elif pose_selection == 3:
                    break
                else:
                    print("Invalid selection. Choose 1 or 2\n")

        elif control_selection == 2:
            playback()

        elif control_selection == 3:
            break

        else:
            print("\nInvalid selection\n")
