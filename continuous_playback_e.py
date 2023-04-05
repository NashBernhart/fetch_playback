#!/usr/bin/env python

import signal
import time
import rospy
import json
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import  Pose, Point, Quaternion
from std_msgs.msg import String

VELOCITY = 0.19

def signal_handler(signal, frame):
    global interrupted
    interrupted = True

signal.signal(signal.SIGINT, signal_handler)

interrupted = False

def save_movement(p_x, p_y, p_z, q_x, q_y, q_z, q_w, time_in_seconds):
    rospy.loginfo('writing to file')
    #file_name = "placeholder"
    file_name = raw_input('\nname your movement file: ')
    dictionary = {
        "sampling_rate": time_in_seconds,
        "positions": {"x": p_x, "y": p_y, "z": p_z},
        "orientations": {"x": q_x, "y": q_y, "z": q_z, "w": q_w}
    }
    json_object = json.dumps(dictionary, indent=4)
    with open(file_name + '.json', 'w') as outfile:
        outfile.write(json_object)

def playback():
    group = moveit_commander.MoveGroupCommander("arm_with_torso")
    command_pub = rospy.Publisher('/command', String, queue_size=10)
    #move_group = MoveGroupInterface("arm_with_torso", "base_link")
    # create the joint names arr
    #joint_names = ["torso_lift_joint", "shoulder_pan_joint",
    #               "shoulder_lift_joint", "upperarm_roll_joint",
    #               "elbow_flex_joint", "forearm_roll_joint",
    #               "wrist_flex_joint", "wrist_roll_joint"]
    #open the file to be read from and load into json object
    file_name = raw_input('\nEnter the name of the movement file you wish to replay: ')
    with open(file_name + '.json') as user_file:
        poses_object = json.load(user_file)
    # get the arrays of poses to move to and times to hold
    p_x = poses_object["positions"]["x"]
    p_y = poses_object["positions"]["y"]
    p_z = poses_object["positions"]["z"]
    q_x = poses_object["orientations"]["x"]
    q_y = poses_object["orientations"]["y"]
    q_z = poses_object["orientations"]["z"]
    q_w = poses_object["orientations"]["w"]

    sampling_rate = poses_object["sampling_rate"]
    waypoints = []
    for idx in range(len(p_x)):
        new_pose = Pose(Point(p_x[idx], p_y[idx], p_z[idx]),Quaternion(q_x[idx], q_y[idx], q_z[idx], q_w[idx]))
        waypoints.append(new_pose)
    #(plan, fraction) = group.compute_cartesian_path(waypoints, 0.1, 0.00)
    #plan = group.retime_trajectory(robot.get_current_state(),
    #                                        plan,
    #                                        velocity_scaling_factor = VELOCITY,
    #                                        )
    #command_pub.publish("start")
    #group.execute(plan, wait=True)
    #command_pub.publish("stop")
    #print(waypoints)


    group.set_pose_targets(waypoints, end_effector_link="gripper_link")
    group.plan()
    group.go(wait=True)


    # perform the actions
    #for idx, pose in enumerate(poses):
    #    move_group.moveToJointPosition(joint_names, 
    #                                   pose, 
    #                                   wait=True,
    #                                   max_velocity_scaling_factor=0.2)
        # rospy.sleep(sampling_rate)
    # finished. cancel all movements
    #group.get_move_action().cancel_all_goals()


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
        print(robot.get_group_names())
        print("\n\nEnter 1 to record a movement\nEnter 2 to playback a pose\nEnter 3 to quit\n")
        control_selection = input("Choose an option: ")

        ## sub loop controlling add a movement feature
        if control_selection == 1:
            ## init the two arrays that will be used to store the movement
            ## pose_arr is an array of arrays where each sub array is a pose. duration array is how long to hold each pose
            #pose_arr = []
            p_x = []
            p_y = []
            p_z = []
            q_x = []
            q_y = []
            q_z = []
            q_w = []
            group_name = "arm_with_torso"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            sampling_rate = input("Enter your sampling rate in Hz: ")
            rate_in_seconds = 1.0 / sampling_rate

            print("\nRecording movement...\n")
            print("\nPress ctrl+c to stop recording and save to file\n")
            while(True) :
                ## Save a movement at defined rate
                ee_pose = move_group.get_current_pose("gripper_link")
                #print(ee_pose.pose.position.x)
                p_x.append(ee_pose.pose.position.x)
                p_y.append(ee_pose.pose.position.y)
                p_z.append(ee_pose.pose.position.z)
                q_x.append(ee_pose.pose.orientation.x)
                q_y.append(ee_pose.pose.orientation.y)
                q_z.append(ee_pose.pose.orientation.z)
                q_w.append(ee_pose.pose.orientation.w)
                # pose_arr.append(joints_arr)
                time.sleep(rate_in_seconds)

                ## keyboard interupt. stop collecting data and save to file
                if interrupted:
                    print("\nSaving movement...\n")
                    save_movement(p_x[1:], p_y[1:], p_z[1:], q_x[1:], q_y[1:], q_z[1:], q_w[1:], rate_in_seconds)
                    interrupted = False
                    break
            
        ## play back a movement from file
        elif control_selection == 2:
            playback()

        elif control_selection == 3:
            break

        else:
            print("\nInvalid selection\n")