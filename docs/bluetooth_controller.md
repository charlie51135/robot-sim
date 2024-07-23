# Using bluetooth controller

## Linux controller support from xpadneo

https://github.com/atar-axis/xpadneo

After setup, controller will connect automatically.

Testing the joystick: http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick




## Install teleop-twist-joy

`sudo apt-get install ros-noetic-teleop-twist-joy`

## Using the joystick

1. `rosparam set joy_node/dev "/dev/input/js0"`
2. `rosrun joy joy_node`
3. `rosrun teleop_twist_joy teleop_node`

## Updating the launch file
```
<param name="joy_node/dev" value="/dev/input/js0"/>
<node name="joy_node" pkg="joy" type="joy_node" output="screen"/>
<node name="teleop_twist_joy" pkg="teleop_twist_joy" type="teleop_node" output="screen"/>
```

## Can only connect to one device at a time

If connected but no ouput using jstest, repair using xpadneo