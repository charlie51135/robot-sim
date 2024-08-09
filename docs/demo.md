PC:

roslaunch my_robot romi_base.launch
roslaunch slam_toolbox online_async.launch


PI:

ssh charlie@192.168.8.234
Pass: rpi51135


roslaunch romi_base romi_hw.launch
roslaunch romi_camera romi_camera_node.launch
roslaunch ldlidar ld19.launch