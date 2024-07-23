# Raspberry Pi Camera V2 (working)
Add `start_x=1` in boot/firmware/config.txt  

Use compressed image transport  
`sudo apt  install ros-noetic-compressed-image-transport`

# USB webcam (working)

`sudo apt-get install ros-noetic-vision-opencv`  


Create new package for camera node  
`cd ~/catkin_ws/src`  
`catkin_create_pkg romi_camera rospy roscpp image_transport cv_bridge sensor_msgs std_msgs`  

Make node.cpp file executable  

Update CmakeLists  
Add `find_package(OpvenCV)`  
Add `${OpenCV_INCLUDE_DIRS} to include_directories`  
`add_executable(camera_node src/camera_node.cpp)`  
`target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})`  



# Arducam 12MP IMX708 (never recognized, need updated kernel)

https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/12MP-IMX708/

`sudo nano /boot/config.txt`  
Find the line: camera_auto_detect=1, update it to:  
`camera_auto_detect=0`  
Find the line: [all], add the following item under it:  
`dtoverlay=imx708`  
Save and reboot  

`cd`  
`mkdir camera && cd camera`  
`wget -O install_pivariety_pkgs.sh https://github.com/ArduCAM/Arducam-Pivariety-V4L2-Driver/releases/download/install_script/install_pivariety_pkgs.sh
chmod +x install_pivariety_pkgs.sh`  

`./install_pivariety_pkgs.sh -p libcamera`  
`./install_pivariety_pkgs.sh -p libcamera_apps`  
`./install_pivariety_pkgs.sh -p imx708_b0311_kernel_driver`  
`sudo reboot`  



sudo apt install python3-pip  
sudo pip install meson --upgrade  
pip install ply  

git clone https://git.libcamera.org/libcamera/libcamera.git  
cd libcamera  
meson setup build  
sudo ninja -C build install  

export PATH="$PATH:/usr/local/bin"  

Path issues:  

cam: error while loading shared libraries: libcamera.so.0.2: cannot open shared object file: No such file or directory  
`find / -name "libcamera.so.0.2" 2>/dev/null`  
`export LD_LIBRARY_PATH=/home/charlie/camera/libcamera/build/src/libcamera/:$LD_LIBRARY_PATH`  


cam: error while loading shared libraries: libcamera-base.so.0.2: cannot open shared object file: No such file or directory  
`find /home/charlie/camera/libcamera/build -name "libcamera-base.so.0.2"`  
`export LD_LIBRARY_PATH=/home/charlie/camera/libcamera/build/src/libcamera/base/:$LD_LIBRARY_PATH`  


```
charlie@robot:~$ cam -l  
[0:55:44.815922590] [4961]  INFO Camera camera_manager.cpp:284 libcamera v0.2.0+83-666f17af  
Available cameras:  
```

Don't think V3 cameras will work.  
https://forums.raspberrypi.com/viewtopic.php?t=347172  


Plugging in Logitech webcam shows up:  
```
cam -l  
cam -c1 -C1 -F  
```

Giving my user video permissions:
`sudo usermod -aG video charlie` 