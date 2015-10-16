# minimal_robot_gripper
Simple robot gripper demonstration in ROS (5th homework for ROS class)


The folder ps5_yxl1450 is the package that can be compiled in ROS environment. My ROS version is Indigo, you can tested it in other versions. The video "gripper_robot_grasp_test.mp4" basicly has demonstrated the structure of this simple robot gripper and how it works.


To get program running, first compile it using catkin_make in your ros workspace. Then running these scripts in seperate terminals:
  $ roscore
  $ rosrun gazebo_ros gazebo
  $ rosrun gazebo_ros spawn_model -file gripper_robot.urdf -urdf -model gripper_robot (in the urdf directory)
  $ rosrun ps5_yxl1450 gripper_robot_controller
  $ rosrun ps5_yxl1450 gripper_robot_trajectory_action_server
  $ rosrun ps5_yxl1450 gripper_robot_trajectory_action_client


As the video demonstrate, you can play with the beer or table to see how the capability of a simple robot gripper. However the beer is weirdly behaved because the gravity has been set to -0.1m/s^2 for a better control of robot joints. You can play with those parameters also. More detailed description can be found in the pdf file "ps5_writeup_yxl1450.pdf".

