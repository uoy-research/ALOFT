# Aloft: Self-Adaptive Drone Controller Testbed

Aloft is an artifact designed to give the self-adaptive systems research community to investigate self-adaptive drone controllers in mine operations. The artifact provides a simulation setup containing two mine environments, constructed from scans taking of constructed mock mines, and 3D data points of one of the mines (located in **3D_points**). A signal loss exemplar is also included. The scenario is where the drone is deployed and manually piloted for a surveying task, but during this task loses signal with home base. The drone must now autonomously fly home and safely land. With battery life a concern and wanting to avoid the drone being retrieved by a mine operator, it is desirable for the drone to return quickly. However there may be mine operators present which should force the drone to fly at a slower speed. 

Aloft contains a self-adaptive controller to address the scenario described based on the two layered architecture of a manged and managing system. The managing system utilises a MAPE-K feedback loop. The specific drone used is the PX4-Vision which is equipped with IMU sensors, and a 3D-depth camera. All models for the exemplar are contained in the folder **catkin_ws/src/drone_controller/models/**.

In Aloft the simulation for the exemplar can be conducted with a few commands. In terminal 1:

`roslaunch drone_controller mine_1_setup.launch darknet_active:=1`

This will load Gazebo with the mock mine, the PX4-Vision and mine operator. The command will also being the PX4-Avoidance packages and will open RViz displaying what the drone is currently perceiving and its current goal. When `darknet_active:=1` is set the *darknet_ros* node will run to perform YOLO object detection. A separate window will display showing the bounding boxes produced by *darknet_ros*. By default *darknet_ros* will not start.

In terminal 2:

`rosrun drone_controller offb_nav 1.0`

This starts the nodes responsible for the managing of the current waypoint and velocity control. This command requires an input argument to determine the max speed. In this example it is set to **1.0**. This node will arm and set the drone to operate in OFFBOARD_MODE. The drone should now fly up waiting for its next waypoint.

To conduct the exemplar of the To get the drone into position, in terminal 3:

`rosrun drone_controller drone_sim_mine_1_setup.py`

The drone will now move to the top-left tunnel. You can change the drone's journey through editing the waypoints in the script **catkin_ws/src/drone_controller/drone_trial.py**. Finally to simulate the return-to-home function with a human present in terminal 3:

`rosrun drone_controller drone_trial.py 1`

The input argument is set to 1 to indicate that the simulation should place the human in the entrance to the mine. You can change the human's starting position through editing the script **catkin_ws/src/drone_controller/drone_trial.py**. The drone will first update its maximum velocity to 0.5 and begin to follow the waypoints home. When it detects the human it shall update its maximum velocity to 0.2 till the end of the mission. When it has reached the final waypoint (0,0), the drone will switch to AUTO-LAND_MODE and land. The terminal will output whether the drone had suffered a collision, at what time it detected a human (if it did detect a human), and how long it took to return home. A video demonstrating the end-to-end mission can be found here: 


Aloft provides a starting point to allow users to expand the controller and problem complexity. Users can inject their own self-adaptive controller as separate ROS package and compare against the provided as a baseline comparison, or build upon the one provided. Examples of making the mine problem more complex includes adding wind (https://github.com/ethz-asl/rotors_simulator/blob/master/rotors_gazebo_plugins/src/gazebo_wind_plugin.cpp) and changing with the sensors accuracy (achieved by editing source files; a good starting place would be **catkin_ws/src/drone_controller/src/NavDrone.cpp**). Furthermore the users can use their developed controller and test in a 2nd simlated mock mine. They can load this mine by using the command instead of the command listed for terminal 1:

`roslaunch drone_controller mine_2_setup.launch`


