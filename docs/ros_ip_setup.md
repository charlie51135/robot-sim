# Setup IP addresses for X ROS_MASTER

1. Edit master .bashrc file:  
        `nano ~/.bashrc`

        export ROS_MASTER_URI=http://X.X.X.X:11311
        export ROS_IP=X.X.X.X

2. Edit Raspberry Pi .bashrc file:
        `nano ~/.bashrc`  

        export ROS_MASTER_URI=http://X.X.X.X:11311  
        export ROS_IP=Y.Y.Y.Y

Helpful: Install remote SSH plugin on VScode for easier editing and terminal access  


Potential Issue:   
> RLException: ERROR: could not contact master [http://192.168.X.X:11311/]  

1. View all open ports  
`sudo lsof -i -P -n`  

2. VScode for RPi was autoforwarding the port, killed process  