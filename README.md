# ARDrone-New
This repository builds on the previous repository named ARDrone. This builds on the movement framework developed last summer, and utilizes the camera to follow a person. 
Download github repo: https://github.com/AutonomyLab/ardrone_autonomy.git


Other requirements:

ROS

Open CV 3.0 compiled with extra modules (for object trackers)

Launch file: Establishes connection with AR Drone through wifi with standard IP address for AR Drones and gets Navdata.

Command: roslauch launch_drone.launch

Control_drone.py: Creates publishers for takeoff, landing, and position commands. Commands roll,pitch,yaw, and z by 0.75. Current implementation: launches the drone, asks angle from user, calculates time needed to turn to the angle, turns to the angle, then flies forward or 0.75 seconds. Process is repeated until user commands drone to land
