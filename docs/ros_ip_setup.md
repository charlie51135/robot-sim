# Setup IP addresses for ROS WiFi communication

Modify the following IP address parameters for ***X*** PC and ***Y*** Raspberry Pi. They are located at the bottom of the .bashrc file.

1. Edit PC (master) .bashrc file:  
        `nano ~/.bashrc`
        
        export ROS_MASTER_URI=http://X.X.X.X:11311
        export ROS_IP=X.X.X.X

2. Edit Raspberry Pi .bashrc file:
        `nano ~/.bashrc`  

        export ROS_MASTER_URI=http://X.X.X.X:11311  
        export ROS_IP=Y.Y.Y.Y

3. Source the updated .bashrc files  
        `source ~/.bashrc`

Helpful: Install remote SSH plugin on VScode for easier editing and terminal access  


### Potential issue:   
> RLException: ERROR: could not contact master [http://192.168.X.X:11311/]  

1. View all open ports  
`sudo lsof -i -P -n`  

1. VScode for RPi was autoforwarding the port, killed process  

### Additional resources:  
* http://wiki.ros.org/ROS/Tutorials/MultipleMachines