#ifndef ROMI_HW_INTERFACE_H
#define ROMI_HW_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include "romi_base/a_star.h"

class RomiHWInterface : public hardware_interface::RobotHW {
public:
    RomiHWInterface();
    AStar& getAStar() { return a_star; }

    void read(const ros::Time &time, const ros::Duration &period);
    void write(const ros::Time &time, const ros::Duration &period);

private:
    double calculateDelta(const double &curr, const double &prev) const;
    double encoderCountToAngle(const int &count) const;
    int velocityToMotorSpeed(const double &angularVelocity) const;
    double angularToLinear(const double &angle) const;
    // double linearToAngular(const double &distance) const;

    ros::NodeHandle nh_;
    AStar a_star;

    static const int LEFT_JOINT = 0;
    static const int RIGHT_JOINT = 1;
    static const int NUM_JOINTS = 2;
    static constexpr double WHEEL_DIAMETER = 0.07; // 70mm
    const int MAX_SPEED = 100; // Actually 300, but that travels too quickly 

    int prev_encoder_counts[2]; // Store previous encoder counts

    double cmd[2];  // Commanded wheel velocities in rad/s
    double pos[2];  // Wheel positions in rad
    double vel[2];  // Wheel velocities in rad/s
    double eff[2];  // Not used with diff_drive_controller

    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::VelocityJointInterface joint_velocity_interface;
};

#endif  // ROMI_HW_INTERFACE_H