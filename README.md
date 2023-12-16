# Aloft: Self-Adaptive Drone Controller Testbed

Aloft is an artifact designed to give the self-adaptive systems research community to investigate self-adaptive drone controllers in mine operations. The artifact provides a simulation setup containing two mine environments, constructed from scans taking of constructed mock mines, and 3D data points of one of the mines. A signal loss exemplar is also included. The scenario is where the drone is deployed and manually piloted for a surveying task, but during this task loses signal with home base. The drone must now autonomously fly home and safely land. With battery life a concern and wanting to avoid the drone being retrieved by a mine operator, it is desirable for the drone to return quickly. However there may be mine operators present which should force the drone to fly at a slower speed. 

Aloft contains a self-adaptive controller to address the scenario described based on the two layered architecture of a manged and managing system. The managing system utilises a MAPE-K feedback loop. 

In Aloft the simulation for the exemplar can be conducted with a few commands. In terminal 1:
[roslaunch drone_controller mine_1_setup.launch]
This will load Gazebo with the 
