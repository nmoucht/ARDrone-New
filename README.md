# ARDrone-New
This repository builds on the previous repository named ARDrone. This builds on the movement framework developed last summer, and utilizes the camera to follow a person. 

Download github repo: https://github.com/AutonomyLab/ardrone_autonomy.git


Other requirements:

ROS Indigo/Kinetic

Open CV 3.0 compiled with extra modules (for object trackers)

There are a few smaller functions that can be downloaded by using sudo apt-get _____

Launch file: Establishes connection with AR Drone through wifi with standard IP address for AR Drones and gets Navdata.

Command: roslauch launch_drone.launch

finalDroneCommand.py: Creates publishers for takeoff, landing, and position commands. Current implementation: Finds a person in frame using a cascade classifier (using haar features) by locating a face, creates a bounding box and initializes the object tracker. Then, another classifier (using a histogram of oriented gradients and a linear svm) is used to determine the height of the person. Once this initialization is complete, the object tracker is constantly updated, and if the angle from the center of the bounding box exceeds 5 degrees from the center of frame, the drone continually turns until the center of the bounding box is within 5 degrees. Every 3 frames the "pedestrian classifier" is used for two reasons: first, to see if the ratio of the current height to the initialized height has increased enough or decreased enough to move the drone backward or forward, and second, to check for tracking failure, in which case is corrects the bounding box. The process of the drone turning and moving forward or backwards continues until the user stops the program.

displayVidFeed.py: Gets raw image from drone and converts it using cvBridge, and displays each image frame by frame.
