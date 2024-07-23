## WiFi Travel Router

With the ROS_MASTER_URI and ROS_IP being environment variables, they need to be updated on all devices for each new network.   

To simplify this process, a travel router can be used to provide a static IP to all connected devices. The router can then be connected to any WiFi/Ethernet connection.

A benefit of this setup is that an internet connection is not required, the travel router will create its own AP.

The router used is the [GL.iNet GL-MT3000](https://www.amazon.com/GL-iNet-GL-MT3000-Pocket-Sized-Wireless-Gigabit/dp/B0BPSGJN7T).  
A cheaper alternative router is the [GL.iNet GL-MT300N](https://www.amazon.com/GL-iNET-GL-MT300N-V2-Repeater-300Mbps-Performance/dp/B073TSK26W).


## Raspberry Pi as AP testing

A few different methods to set up the Raspberry Pi as an AP were tested with difficulty. The WiFi signal was broadcasted by the Pi, but the DCHP addressing did not work so when a conneciton attempt was made, no IP address was assigned to the device. This appeared to be an issue with the dnsmasq configuration, for which most the documentation is for Raspberry Pi OS and not Ubuntu on Raspberry Pi.

`iptables -V`   
>iptables v1.8.4 (legacy)  


https://gist.github.com/ExtremeGTX/ea1d1c12dde8261b263ab2fead983dc8  

If AP doesnt show up, hostapd not working   
`sudo systemctl status hostapd`  

If no IP, dnsmasq not working  
`sudo systemctl status dnsmasq`  
