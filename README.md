# Read me
## creating the workspace
- mkdir -p ur5_ws/src
- cd ur5_ws/src
- clone this repo in src folder. Move all the contents from cloned ur5_ws folder to src folder.
- cd ..
- catkin_make


## Load the gazebo simulation
roslaunch ur_gazebo ur5.launch

## Running the moveit package
roslaunch my_robot_moveit_config my_robot_planning_execution.launch 

## Moving the arm to custom position
rosrun ur_description manual_control_ur5.py (Tested on the original ur5 in gazebo)
