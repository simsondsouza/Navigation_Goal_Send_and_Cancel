https://github.com/SimsonDsouza/Void-Robotics-Internship-Application-Task/assets/66258060/27ccdd25-8b92-4cb7-85a0-0f0a3dfb4d48

### Task:
- The task was to create a C++ ROS2 node that when run will stop the robot.


### To accomplish this Task:
- The robot used in this taks is "turtlebot3 burger model". The necessary gazebo simulation and navigation files are cloned from the open-source turtlebot3 github repository.
- Turtlbot3 Packages Used:
  
  a) turtlebot3_simulation: It is used for gazebo simulation.
  
  b) turtlebot3: It is used for navigation.
  
- Custom package named "my_robot_control" is created.
- Within the package, src folder, two c++ scripts are created:
  
  a) send_goal.cpp: Action client is created to send goal.
  
  b) goal_cancel.cpp: Action client is create to cancel the active goal. Thus, on running this node, the robot stops as the goal gets cancelled.


### Run the following commands in the terminal to test:

Note:
- Change to the workspace location in which this project files are present
  
A) Terminal 1
- ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  
B) Terminal 2
- ros2 launch turtlebot3_navigation2 navigation2.launch.py
  
C) Terminal 3
- ros2 run my_robot_control send_goal.cpp
  
D) Terminal 4
- ros2 run my_robot_control goal_cancel.cpp


### List of resources in use to complete this task:

A) To setup Turtlebot3 simulation
- https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/
  
B) ROS2 Humble Documentation (To undersand Action Client, Action Server and other important topics)
- thttps://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
  
C) For sending goal, I used the following code as reference
- https://qiita-com.translate.goog/porizou1/items/cb9382bb2955c144d168?_x_tr_sl=ja&_x_tr_tl=en&_x_tr_hl=en&_x_tr_pto=sc
  
D) To find active goal feedback topic and determine the logic to cancel the active goal
- https://get-help.theconstruct.ai/t/check-if-a-computepaththroughposes-action-goal-has-been-achieved-from-another-node/22639/2
  
E) To understand how to push code to Github repository
- https://automaticaddison.com/how-to-upload-a-ros-2-project-to-github/
