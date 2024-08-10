# How to setup Ubuntu 20.04 to boot from USB on Raspberry Pi
References:

* https://forums.raspberrypi.com/viewtopic.php?t=278791  
* https://askubuntu.com/questions/1254810/booting-ubuntu-server-20-04-on-pi-4-from-usb  
* https://wiki.ros.org/noetic/Installation/Ubuntu  
* https://www.pololu.com/blog/663/building-a-raspberry-pi-robot-with-the-romi-chassis  



## Install ubuntu 20.04 server image using Raspberry Pi Imager
1. Setup name, user, and WiFi in configuration manager.  
	To add WiFi manually: `sudo nano /etc/netplan/50-cloud-init.yaml`

2. On linux system: open system-boot drive in terminal  
	`cd /media/*user*/system-boot`  

	Decompress vmlinuz  
	`sudo dd if=vmlinuz bs=1 | zcat > vmlinux`

3. Update under [pi4] to  
	`nano config.txt`
	```
	max_framebuffers=2
	dtoverlay=vc4-fkms-v3d
	boot_delay
	kernel=vmlinux
	initramfs initrd.img followkernel
	```

	Save file

4. Create decompression script  
	`nano auto_decompress_kernel`
	```bash
	#!/bin/bash -e

	#Set Variables
	BTPATH=/boot/firmware
	CKPATH=$BTPATH/vmlinuz
	DKPATH=$BTPATH/vmlinux

	#Check if compression needs to be done.
	if [ -e $BTPATH/check.md5 ]; then
		if md5sum --status --ignore-missing -c $BTPATH/check.md5; then
		echo -e "\e[32mFiles have not changed, Decompression not needed\e[0m"
		exit 0
		else echo -e "\e[31mHash failed, kernel will be compressed\e[0m"
		fi
	fi

	#Backup the old decompressed kernel
	mv $DKPATH $DKPATH.bak

	if [ ! $? == 0 ]; then
		echo -e "\e[31mDECOMPRESSED KERNEL BACKUP FAILED!\e[0m"
		exit 1
	else 	echo -e "\e[32mDecompressed kernel backup was successful\e[0m"
	fi

	#Decompress the new kernel
	echo "Decompressing kernel: "$CKPATH".............."

	zcat $CKPATH > $DKPATH

	if [ ! $? == 0 ]; then
		echo -e "\e[31mKERNEL FAILED TO DECOMPRESS!\e[0m"
		exit 1
	else
		echo -e "\e[32mKernel Decompressed Succesfully\e[0m"
	fi

	#Hash the new kernel for checking
	md5sum $CKPATH $DKPATH > $BTPATH/check.md5

	if [ ! $? == 0 ]; then
		echo -e "\e[31mMD5 GENERATION FAILED!\e[0m"
		else echo -e "\e[32mMD5 generated Succesfully\e[0m"
	fi

	#Exit
	exit 0
	```
	Save file

5. Find and mount linux parition:  *(for convention only, can skip to step 6 then unmount both system-boot and writable instead)*  
	* `sudo fdisk -l` (see which is linux partition)  
	* `sudo mkdir /media/usb-drive`  
	* `sudo mount /dev/sda2 /media/usb-drive/`  
	* `mount | grep sda2`  
	* `cd /media/usb-drive`

6. Create decompression script:  
	* `cd etc/apt/apt.conf.d/`  
	* `sudo nano 999_decompress_rpi_kernel`  
	* `DPkg::Post-Invoke {"/bin/bash /boot/firmware/auto_decompress_kernel"; };`

7. Make executable and unmount:  
	* `sudo chmod +x 999_decompress_rpi_kernel`  
	* `cd`  
	* `sudo umount /media/usb-drive`


## Plug into rpi and boot

Temp fix for internet connection: `sudo dhclient -v`  
Perm fix?:
`sudo crontab -e`  
`@reboot dhclient -v`

Check if using WiFi or lan: `ip route`  
To change default from wlan0 to eth0 
`sudo apt install net-tools` (optional)  
`ip route`  
`sudo route delete default gw <IP Address> <Adapter>`  
`sudo route add default gw <IPAddress> <Adapter>`  
Recheck with `ip route` and see improved speeds with
`ping -c 5 www.google.com`


`sudo apt update`  
`sudo apt upgrade`  
*Cache lock: `sudo reboot`*  

Add no password commands with:
`visudo` (optional)  
Add the following at the end:
```
<user> ALL = (root) NOPASSWD: /sbin/halt
<user> ALL = (root) NOPASSWD: /sbin/shutdown
<user> ALL = (root) NOPASSWD: /sbin/reboot
```
Save file

`sudo reboot`

-------------------------------------------------------------------------------------------------
## Enable i2c

1. Download latest version of raspi-config  
	`wget https://archive.raspberrypi.org/debian/pool/main/r/raspi-config/raspi-config_20240313_all.deb -P /tmp`  

2. Attempt install:  
	`sudo dpkg -i /tmp/raspi-config_20240313_all.deb`

3. Get missing dependencies:  
	`sudo apt-get install lua5.1 alsa-utils`  
	`sudo apt --fix-broken install`

4. Re-attempt install:  
	`sudo dpkg -i /tmp/raspi-config_20240313_all.deb`  
	`sudo raspi-config`
	>\> interface options > i2c > enable

5. Check i2c clock speed:  
	`hexdump -Cv < /sys/class/i2c-adapter/i2c-1/of_node/clock-frequency`
	>(186a0 = 100kHz, 61a80 = 400kHz)

6. Change i2c clock from 100khz to 400khz:  
	`sudo nano /boot/firmware/config.txt`
	```
	dtparam=i2c_arm=on
	dtparam=i2c_arm_baudrate=400000
	```

7. Add user to dialout group  
	`sudo usermod -aG i2c,dialout <user>`  

	`sudo reboot`

8. Check that i2c-1 is available
	`l -tr /dev`

	Recheck i2c clock speed

## Install additional required libraries
`sudo apt-get install python3 python3-flask python3-smbus`  (Only needed for Pololu Python AStar scripts)

`wget https://github.com/pololu/pololu-rpi-slave-arduino-library/archive/2.0.0.tar.gz`  

`tar -xzf <version>.tar.gz`  

`mv pololu-rpi-slave-arduino-library-2.0.0 pololu-rpi-slave-arduino-library`


## Install ROS

`sudo add-apt-repository universe`  
`sudo add-apt-repository restricted`  
`sudo add-apt-repository multiverse`  
> all of these were already enabled

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`  

`sudo apt install curl`  

`curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`  

`sudo apt update`  

`sudo apt install ros-noetic-ros-base`  

`echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc`  

`sudo apt install python3-rosdep`  

`sudo rosdep init`  

`rosdep update`

Add additional packages using  
`sudo apt install ros-noetic-PACKAGE`  
`sudo apt install ros-noetic-robot-state-publisher`


## Setup catkin workspace
* `sudo apt install g++`  
* `mkdir -p ~/catkin_ws/src`  
* `cd ~/catkin_ws/`  
* `catkin_make`  
* `echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`  
* `source ~/.bashrc`  



## Setup sketch on PC 
https://www.pololu.com/blog/663/building-a-raspberry-pi-robot-with-the-romi-chassis

1. Install Arduino IDE  
Install Pololu RPi Slave libarary and Romi32U4 library in Arduino IDE

2. Add boards package url `https://files.pololu.com/arduino/package_pololu_index.json`

3. Install Pololu A-Star Boards in boards manager

4. Restart IDE

5. Install windows drivers https://www.pololu.com/docs/0J69/5.1