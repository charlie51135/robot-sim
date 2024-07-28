Install GMapping Ros library

`sudo apt install ros-noetic-slam-gmapping`

`sudo apt install ros-noetic-slam-toolbox`

Must set scan topic, odom frame, and base_link frame parameters.


For slam toolbox, copy params file

`cp /opt/ros/noetic/share/slam_toolbox/config/mapper_params_online_async.yaml catkin_ws/src/my_robot/config/`



Launch:
roslaunch slam_toolbox online_async.launch