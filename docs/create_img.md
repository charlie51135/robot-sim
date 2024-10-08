# Create the Raspberry Pi image

A few modifications had to be done since ROS Neotic uses Ubuntu 20.04 and Ubuntu did not support USB boot until 21.04. Instead of having to do these manual changes and install all of the required packages, an image can be created with all of the software and configurations preloaded. 

A byte-to-byte copy must be made to create an image of the full OS. This was originally an issue as the USB drive used was 128GB. A file this large is impractical to both share and upload. To reduce the filesystem and partitions to the minimum required size, [PiShrink](https://github.com/Drewsif/PiShrink) was used.


## Shrink and compress the image

* Ensure sufficient space to write location: `df -h`  
* Check for drive name after plugging in: `dmesg`  

1. Copy the drive using dd  
    `sudo dd if=/dev/sda conv=sync,noerror status=progress bs=64K | gzip -c  > /home/charlie/robot_os/backup_image.img.gz`

2. Unzip the file for PiShrink  
    `gunzip backup_image.img.gz`

3. Shrink the image using PiShrink  
    `sudo pishrink.sh -va backup_image.img`

4. Zip the file for uploading  
    `gzip -kv backup_image.img`

These steps were able to get a 3.6GB .img.gz file from a 128GB OS.


## Upload the image to a USB drive

1. Install [Raspberry Pi Imager](https://www.raspberrypi.com/software/) for Windows or `sudo snap install rpi-imager` for Linux

2. Download the [image file](https://drive.google.com/file/d/1gfvOr653rBKGsRue2pg0K5v4wNSrjCS1/view?usp=sharing)

3. Make the following selections:
   ```
    > Choose Device > Pi 4
    > Choose OS > Use Custom > rpi_romi_os.img.gz
    > Choose Storage > Select USB drive
    > Next

   Would you like to apply OS customization settings? > No  
   ```

4. Write to the USB

5. WiFi configuration can be done on a Linux machine.  
   * Plug in USB  
   * `sudo nano /media/*user*/writable/etc/netplan/50-cloud-init.yaml`  
   * Edit the WiFi SSID and Password  
   * Save and exit: *Ctrl+s, Ctrl+x*  
   
6. Raspberry Pi default login:
   ```
   User: romi
   Password: romi32u4
   ```
   Change password with `passwd`  