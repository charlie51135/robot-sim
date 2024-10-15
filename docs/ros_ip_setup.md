# Setup IP addresses for ROS WiFi communication

In order for the robot and PC to communicate properly in a ROS network, you need to update the IP addresses. Modify the following IP address parameters for ***X*** PC and ***Y*** Raspberry Pi.

1. Edit PC (master) IP config:  
        Open the file `my_robot/launch/romi_base.launch`  
        </br>
        Update the following environment variables. Replace the X.X.X.X with the IP address of your PC (master)
        
        export ROS_MASTER_URI=http://X.X.X.X:11311
        export ROS_IP=X.X.X.X

2. Edit the Raspberry Pi IP config:  
        `nano ~/.bashrc`  
        </br>
        Update the following environment variables located at the bottom of the .bashrc file. Replace the X.X.X.X with the IP address of your PC (master) and the Y.Y.Y.Y with the IP address of the Raspberry Pi.

        export ROS_MASTER_URI=http://X.X.X.X:11311  
        export ROS_IP=Y.Y.Y.Y

   Source the updated .bashrc file:   
        `source ~/.bashrc`

</br>
Helpful: Install remote SSH plugin on VScode for easier editing and terminal access  


### Potential issue:   
> RLException: ERROR: could not contact master [http://192.168.X.X:11311/]  

1. View all open ports  
`sudo lsof -i -P -n`  

1. VScode for RPi was autoforwarding the port, killed process  

### Additional resources:  
* http://wiki.ros.org/ROS/Tutorials/MultipleMachines