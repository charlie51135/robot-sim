#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include "romi_base/romi_hw_interface.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "romi_hw_interface");
    ros::NodeHandle nh;
    ROS_INFO("Romi hardware interface node started.");

    RomiHWInterface romi;
    controller_manager::ControllerManager cm(&romi);

    // Set yellow LED to indicate node is active
    romi.getAStar().setYellowLed(255);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(50);

    while (ros::ok()) {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time - prev_time;

        romi.read(time, period);
        cm.update(time, period);
        romi.write(time, period);
        rate.sleep();
        prev_time = time;
    }
    
    romi.getAStar().setYellowLed(0);
    return 0;
}