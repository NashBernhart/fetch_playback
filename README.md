# Fetch Playback

fetch_playback is a ROS node to implement a 'programming by demonstration' functinoality on Fetch running Ubuntu 18 and ROS Melodic.
This was inspired and draws from the fetch_pbd repo from Sarah Elliott at the University of Washington

## Requirements

Fetch runing Ubuntu 18 and ROS Melodic.
MoveIt is required to be installed and running when using this program.
In order to use the continuous_playback_dist version, numpy must be installed.

## Installation

Create a package in your catkin_ws/src directory called fetch_playback that depends on rospy
```bash
cd ~/catkin_ws
catkin_create_package fetch_playback rospy
catkin_make
```
Place the python files in the src folder of your package
Run catkin_make again.

## Usage
Launch the MoveGroup package on Fetch

Source the setup file in catkin_ws
```bash
cd ~/catkin_ws
source devel/setup.bash
```

Run the program with the following command
```bash
rosrun fetch_playback continuous_playback_time.py
```

From there follow the menu prompts to perform the desired actions

### Recording a movement

From the menu, enter 1 to record a movement.
When prompted, enter the desired sampling rate in Hertz at which to record poses.
Slower rates are optimal as large data sets will have trouble when computing a path.
The suggested rate is 0.25 to 0.5.
At these rates, a pose will be recorded every 4 seconds and 2 seconds respectively.

You are now recording. Grab the end effector and move the arm in the trajectory you wish to record.
While recording, keep in mind the sampling rate to ensure all the desired waypoints are reached.

It is recommended that you bring the robot back to an initial pose from which to start the recording and playback from for the sake of consistency. 

Press ctrl+c when finished recording to save the pose.
You will be prompted to name the file. The '.json' file extension will be automatically appended and should not be entered.
The JSON file that the movement can be replayed from will be saved to the directory where the node was launched.


### Playback a movement

From the menu, enter 2 to play a movement back.
Ensure nothing is in the way of the expected path of the robot arm.
Enter the name of the file that you recorded. The '.json' file extension will be automatically appended and should not be included.
The file must be in the directory that the node was launched from.
The robot will then try to calculate a trajectory through the given waypoints in the JSON file.
If it is successful, the movement will be replayed. If not, the program will return to the menu.


### Playing back from a pre-calculated trajectory

When playing a movement back if the calculation of the trajectory is successful, you will be given the option of saving the trajectory.
The trajctory message that was just played will exist as a trajectory message and can be saved in a .yaml.
If you choose to do so, you will be prompted to enter a name. The file extension '.yaml' will be automatically appended and should not be included.
You can play this trajectory back without having to recalculate the path with a JSON file by entering 3 at the menu and entering the name of the yaml file.

### Using the continuous_playback_dist version

This version of the node operates in much the same manner as the time based version.
This version will record a new pose based on difference from the previous pose instead of on a time interval.
The difference is set to 0.1m
The structure of this version was motivated by seeing this method used in the fetch_pbd node from Sarah Elliott at the University of Washington.
Usage is the same except that it will not prompt the user for a sampling rate.


## Troubleshooting

You may run into issues where Fetch is unable to calculate the trajectory and playback the movement.
There are two main issues that could be causing this.

### Obstructions in path

While running MoveIt, Fetch is observing the surroundings and recording obstacles in the octomap.
If there are obstacles in the path you wish to plan, Fetch will be unable to plan the trajectory.

This can be viewed in RVIZ.

While moving the end effector you may have been in the view of the robot causing it to record an obstacle in front of it.
Before trying to replay the movement, stand behind the robot and move the head camera around slowly to update the octomap so that Fetch knows there are no obstacles.

The other issue may be glare from bright lights in the room with Fetch.
During testing, glare from birght lights caused Fetch to interpret obstacles in front of it that were not there.
The solution was to move Fetch to a darker room to avoid glare.

Use RVIZ to confirm there are no obstacles in the octomap before trying to replay a movement.

### Data set too large

The MoveIt method being utilized to create the trajectory is called 'calculate_cartesian_path' that takes a list of poses.
In testing, this method appeared to have issues with large data sets. The max amount of poses that were successful in testing was 15-20.
If you find the issues calculating the path after making sure there are no obstructions, try slower sampling rates or or shorter recording times.


## Activity Diagram
<img width="578" alt="Screenshot 2023-05-21 at 1 32 33 AM" src="https://github.com/NashBernhart/fetch_playback/assets/99688169/23740fb0-dc5f-42ac-8d9c-e96edc876195">
