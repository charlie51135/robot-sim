#include "romi_base/romi_hw_interface.h"
#include <iostream>

RomiHWInterface::RomiHWInterface() : nh_("~"), a_star() {
    // Register joint state interfaces
    hardware_interface::JointStateHandle left_joint_state_handle("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
    joint_state_interface.registerHandle(left_joint_state_handle);
    hardware_interface::JointStateHandle right_joint_state_handle("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
    joint_state_interface.registerHandle(right_joint_state_handle);

    registerInterface(&joint_state_interface);

    // Register joint velocity interfaces
    hardware_interface::JointHandle left_joint_handle(joint_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
    joint_velocity_interface.registerHandle(left_joint_handle);
    hardware_interface::JointHandle right_joint_handle(joint_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
    joint_velocity_interface.registerHandle(right_joint_handle);

    registerInterface(&joint_velocity_interface);
}

// Read state signals from the Romi32U4 board
void RomiHWInterface::read(const ros::Time &time, const ros::Duration &period) {
    ros::Duration elapsed_time = period;

    bool buttonA, buttonB, buttonC;
    std::tie(buttonA, buttonB, buttonC)= a_star.readButtons();

    double encoder_counts[NUM_JOINTS];
    double wheel_angle_deltas[NUM_JOINTS];

    std::tie(encoder_counts[LEFT_JOINT], encoder_counts[RIGHT_JOINT]) = a_star.readEncoders();

    // // Update wheel states for all joints
    for (int joint = 0; joint < NUM_JOINTS; ++joint) {

        // Calculate change in encoder counts since last measured
        double delta_count = calculateDelta(encoder_counts[joint], prev_encoder_counts[joint]);

        // Convert count to angular change
        wheel_angle_deltas[joint] = encoderCountToAngle(delta_count);

        // Update position, velocity, and effort
        pos[joint] += wheel_angle_deltas[joint];
        vel[joint] = wheel_angle_deltas[joint] / period.toSec();
        eff[joint] = 0.0;   // Not used for diff_drive_controller

        // Store current encoder counts for next iteration
        prev_encoder_counts[joint] = encoder_counts[joint];
    }
}

// Write control signals to the Romi32U4 board
void RomiHWInterface::write(const ros::Time &time, const ros::Duration &period) {
    ros::Duration elapsed_time = period;

    // Convert angular speed in rad/s to motor speed
    int left_motor_speed = velocityToMotorSpeed(cmd[LEFT_JOINT]);
    int right_motor_speed = velocityToMotorSpeed(cmd[RIGHT_JOINT])+2;

    // std::cout << "Left angular velocity: " << cmd[LEFT_JOINT] << std::endl;
    // std::cout << "Left motor speed: " << left_motor_speed << std::endl;
    // std::cout << "Right angular velocity: " << cmd[RIGHT_JOINT] << std::endl;
    // std::cout << "Right motor speed: " << left_motor_speed << std::endl;

    // Set motor speeds
    a_star.setMotors(left_motor_speed, right_motor_speed);

    // Set Leds
    uint8_t green = 255;
    uint8_t red = 0;
    // a_star.setYellowLed(yellow); // reserved for romi_base node
    // a_star.setGreenLed(green);
    // a_star.setRedLed(red);
}

// Calculate the change in encoder counts accounting for overflow and underflow
double RomiHWInterface::calculateDelta(const double &curr, const double &prev) const {
    double max_count = 65535;   // Max encoder count reported by Romi32U4 board, uint16->int16
    double delta = curr - prev;
    if (delta > max_count / 2) {            // Underflow
        delta -= max_count + 1;
    } else if (delta < -max_count / 2) {    // Overflow
        delta += max_count + 1;
    }
    return delta;
}

// Convert number of encoder counts to angle in radians
double RomiHWInterface::encoderCountToAngle(const int &count) const {
    // Romi has 1440 encoder counts per wheel revolution
    return static_cast<double>(count) * (2.0 * M_PI / 1440.0);
}

// Convert command velocity (in rad/s) to motor speed. Range: [-MAX_SPEED, MAX_SPEED]
int RomiHWInterface::velocityToMotorSpeed(const double &angularVelocity) const {
    const double max_motor_angular_velocity = 14.0; // in rad/s as published by joy node
    double max_motor_linear_velocity = angularToLinear(max_motor_angular_velocity);
    // std::cout << "ang: " << angularVelocity << std::endl;

    int motor_speed = static_cast<int>(std::round((angularToLinear(angularVelocity) / max_motor_linear_velocity) * MAX_SPEED));
    return std::max(std::min(motor_speed, MAX_SPEED), -MAX_SPEED);
}

// Convert angular velocity (rad/s) into linear velocity (m/s)
double RomiHWInterface::angularToLinear(const double &angle) const {
    return angle * WHEEL_DIAMETER / 2.0;
}

// UNUSED
// Convert linear velocity (m/s) into angular velocity (rad/s)
// double RomiHWInterface::linearToAngular(const double &distance) const {
//     return distance / WHEEL_DIAMETER * 2.0;
// }
