# Set up ros_control for diff drive robot


## Overview


<div style="display: flex; justify-content: left; align-items: left;">
   <div style="text-align: center; margin: 10px;">
      <img src="files/gazebo_ros_control.png" width="100%" height="100%"/>
      <p><a href="http://wiki.ros.org/ros_control">http://wiki.ros.org/ros_control</a></p>
   </div>
</div>



## Notes:
For simulation, the diff_drive_controller communicates with gazebo_ros_control.
For the real robol, a controller manager is used to handle the resources of hardware_interace.

diff_drive_controller only publishes the transform between odom and base_link. The robot_state_publisher publishes the rest of the transformations.

Install ROS controller packages
`sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-rosparam-shortcuts`

Create new robot_base package
`cd ~/catkin_ws/src`  
`catkin_create_pkg romi_base roscpp rospy hardware_interface controller_manager diff_drive_controller sensor_msgs rosparam_shortcuts`  

Search for active controllers  
`rosservice list | grep controller_manager`  




# WiringPi for i2c comm (wiringPiI2C.h)
https://github.com/WiringPi/WiringPi  

To install wiringPi:  
`git clone https://github.com/WiringPi/WiringPi.git`  
`cd WiringPi`  
`./build`  
To un-install wiringPi:  
`./build uninstall`  

Updated CMakeLists.txt  
```
link_directories(
  /usr/local/lib
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(romi_base PRIVATE Threads::Threads)
target_link_libraries(romi_base PRIVATE ${catkin_LIBRARIES} wiringPi wiringPiDev)
```


Wrote new cpp AStar class to read and write data according to the data structure on the romi.
```
// https://docs.python.org/3.6/library/struct.html#format-characters
// ? - bool           - 1 byte
// c - char           - 1 byte
// B - unsigned char  - 1 byte
// h - short          - 2 bytes
// H - unsigned short - 2 bytes
// f - float          - 4 bytes
// s - char[]         - preceding # is bytes

struct Data
{                                 // Address (byte)
  bool yellow, green, red;        // 0 1 2
  bool buttonA, buttonB, buttonC; // 3 4 5 

  int16_t leftMotor, rightMotor;  // 6-7 8-9
  uint16_t batteryMillivolts;     // 10-11
  uint16_t analog[6];             // 12-23

  bool playNotes;                 // 24
  char notes[14];                 // 25-38

  int16_t leftEncoder, rightEncoder; // 39-40 41-42
};
```

WiringPi is only for arm architecture, so catkin_make would not build on desktop x86. CMakeLists was edited for the romi_base package to check the architecture before linking to WiringPi. 


### Potential issue:
Very occasionally incorrectly read the value 20 from the right encoder. While it was infrequent and corrected one read later, there was still a flicker in RViz. Over time, these small glitches may develop into error in the reported position of the robot. Looking with an oscilliscope the problem became clear. SMBus protocol, as used by WiringPi, involves 3 bytes of data for a write, while a read 16bit requires 5.
Write: (S) Addr wr, reg, data (P)
Read: (S) Addr wr, reg, (S) addr rd, data low, data high (P)
The difference between write and read is that the read sends a second start signal before the address to read (3rd byte).
As seen on the oscilliscope, when the master is initiating a read command, the second start bit is missed, and interpreted as the first bit of data to write. Instead of interpreting the read command at address 41-[start, 00101000, ack], the romi board interprets it as a write command of 20- [00010100, ack].
Since 20 was just written into the right encoder register, the subsequent read will be 20, after which is correctly updated by the main loop on the romi board. 


### The solution:
Looking at the Arduino PololuRPiSlave library, the recieve data function sets AND retains an index starting at the recieved address. This value is temporarily stored and auto incremented on each read. Therefore, the second start bit could be completely avoided by not using the formal SMBus protocol. A simple write address followed by a series of reads is sufficient. 

Instead of reading 16 bytes at the desired address:
```
left = wiringPiI2CReadReg16(fd, 39);
right = wiringPiI2CReadReg16(fd, 41);
```
the address of the first byte could be written then the next 4 bytes could be read and reassembled back into the 16 bit numbers. A small delay was added between the write and read as recommended by Pololu since the AVR's TWI modue cannot handle a quick transition. (fig18)
```
wiringPiI2CWrite(fd,39);
delayMicroseconds(100);
left = wiringPiI2CRead(fd) + (wiringPiI2CRead(fd) << 8);
right = wiringPiI2CRead(fd) + (wiringPiI2CRead(fd) << 8);
```

# Other solved issues:

Issue: Stuck diff_drive_controller loading:  
Solution: Added async spinner to hw_interface  
Removed namespace from controllers.yaml  
Make sure gazebo ros controller is not active. Restart roscore if needed  
Remap joy cmd_vel to diff_drive_controller/cmd_vel  
Make sure Romi AStar board is turned on...  


Issue: Robot showing up and moving (but glitching back to origin). Missing TF from left_wheel and right_wheel to odom.  
Solution: romi_hw_interface subscribed to /joint_states while robot state publisher was publishing to /robot/joint_states  


Issue: Romi only moves forward for a bit before stopping.  
Solution: Joystick was in event mode, meaning is the state was not changing, no new messages would be published. fixed by adding autorepeat_rate paramter to the joy node. This parameter specifies the frequency to republish the last state if no changes are detected. 


Issue: Using Joy node to publish cmd_vel, the angular velocities for each wheel are within [-14, 14] on the x-axis but only between [-1, 1] on the y-axis. The result is that turning speeds are incredibly slow or even clipped to zero. 
Solution: Added a scaling factor of 7 to the y-axis.

Issue:  
<video src="files/simplescreenrecorder-2024-04-02_14.28.48.mp4" width="320" height="240" controls></video>

Solution: Reviewing debug statements showed the glitching happening when the encoder values underflow.
Fixed by adding a calculate delta function that accounts for overflows and underflows.


The robot's position was now consistant, but there was still some slight glitching.  
Viewing the logs of the encoder values showed occasional readings of -1.  

![files](files/bad_encoder_reading.png)
<!-- <img src="files/bad_encoder_reading.png" style="height: 500px; width:750px;"/> -->
While the encoder values on the romi are uint16 (2 bytes), the WiringPi library converts them to int16 (now 4 btyes) by anding with 0xFFFF. The result is that sign extension is lost and the readings are now in range [0, 65535]. If there is an error reading from the board, the function returns -1.  
This was temporarily fixed by waiting until a valid reading is received.
The perminate fix was to read the bytes individually and reassemble with a bit shift.