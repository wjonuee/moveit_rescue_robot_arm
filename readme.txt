Using following command can launch the function of motion planning of rescue robot arm based on moveit:

$roscore

$roslaunch rescue_robot_arm_moveit_package rescue_robot_arm.SLDASM_demo.launch

$rosrun rescue_robot_arm_tcp2ros rescue_robot_arm.SLDASM_driver_with_goal




catkin_ws_wjonuee ros workspace have 3 ros package:

#################  package: rescue_robot_arm.SLDASM  #################
This package is the urdf file of rescue robot arm , using following command can display arm in ROS RViz:

$roslaunch rescue_robot_arm.SLDASM display.launch


#################  package: rescue_robot_arm_moveit_package  #################  
This package relizing the moveit motion planning of rescue robot arm, using following command:

$roslaunch rescue_robot_arm_moveit_package rescue_robot_arm.SLDASM_demo.launch 

if without real arm, can using following cammand to launch the simulation of moveit motion plannig.
 
$roslaunch rescue_robot_arm_moveit_package demo.launch


#################  package: rescue_robot_arm_tcp2ros  ################# 
This package is the bridge of PC/104 software with rescue_robot_arm_moveit_package, relize communication between ROS node with the socket server pthread of PC/104 software,using following command:

rosrun rescue_robot_arm_tcp2ros rescue_robot_arm.SLDASM_driver_with_goal
