#!/usr/bin/env python

import rospy
import json
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from sensor_msgs.msg import JointState

def add_a_pose(pose_list, time_list, move_group):
    # arr to hold rounded values of joint angles
    pose = []
    # arr holding unrounded values of joint angles returned from get_current_joint_values
    joints_arr = move_group.get_current_joint_values()
    pose_list.append(joints_arr)
    print("\nEnter how long you would like to hold this pose for\n")
    pose_dur = input("Time in seconds: ")
    time_list.append(pose_dur)

def save_movement(pose_list, time_list):
    rospy.loginfo('writing to file')
    #file_name = "placeholder"
    file_name = raw_input('\nname your movement file: ')
    print(file_name)
    dictionary = {
        "poses": pose_list,
        "durations": time_list
    }
    json_object = json.dumps(dictionary, indent=4)
    with open(file_name + '.json', 'w') as outfile:
        outfile.write(json_object)

def playback():
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # create the joint names arr
    joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
    #open the file to be read from and load into json object
    file_name = raw_input('\nEnter the name of the movement file you wish to replay: ')
    with open(file_name + '.json') as user_file:
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

    # dont think this is being used. delete if confirmed not needed
    robot = moveit_commander.RobotCommander()

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
    
    # outer loop controlling create movement or play movement
    while(True):
        print("\n\nEnter 1 to record a movement\nEnter 2 to playback a pose\nEnter 3 to quit\n")
        control_selection = input("Choose an option: ")

        # sub loop controlling add a movement feature
        if control_selection == 1:
            # init the two arrays that will be used to store the movement
            # pose_arr is an array of arrays where each sub array is a pose. duration array is how long to hold each pose
            pose_arr = []
            duration_arr =[]
            # create movegroupcommander object
            group_name = "arm_with_torso"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            gripper_group_name = "gripper"
            gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
            while(True):
                print("\nEnter 1 to save a pose in the movement.\n Enter 2 to finish and save the movement to a file.\nEnter 3 to quit without saving the movement\n")
                pose_selection = input("Selection: ")
                if pose_selection == 1:
                    add_a_pose(pose_arr, duration_arr, move_group)
                elif pose_selection == 2:
                    save_movement(pose_arr, duration_arr)
                    break
                elif pose_selection == 3:
                    break
                else:
                    print("Invalid selection. Choose 1 or 2\n")

        # play back a movement from file
        elif control_selection == 2:
            playback()

        elif control_selection == 3:
            break

        else:
            print("\nInvalid selection\n")
