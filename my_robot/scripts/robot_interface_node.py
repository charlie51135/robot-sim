#!/usr/bin/env python3

# Publishes:    /robot/battery_state
# Subscribes:   /robot/cmd_vel

# Sends cmd_vel to AStar board, updates all robot data

import rospy
from a_star import AStar
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState

class RobotInterface:
    def __init__(self):
        self.a_star = AStar()

        rospy.init_node('robot_interface')
        rospy.loginfo("Robot interface node started")

        # Setup topics
        self.battery_state_publisher = rospy.Publisher('/robot/battery_state', BatteryState, queue_size=10)
        rospy.Subscriber('/robot/diff_drive_controller/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocity components from twist message
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate motor speeds
        # Scale the the range [-100, 100], max speeds are [-300, 300]
        left_speed = int(max(min((linear_vel*2 - angular_vel * 0.25 * 14 / 2) * 100, 100), -100))
        right_speed = int(max(min((linear_vel*2 + angular_vel * 0.25 * 14 / 2) * 100, 100), -100))

        rospy.loginfo(f"Set speeds: Left= {left_speed}, Right= {right_speed}")

        try:
            # Send motor speeds to AStar board
            self.a_star.motors(left_speed, right_speed)
        except Exception as e:
            rospy.logerr(f"Failed to send motor speeds: {e}")

    def read_robot_state(self):
        try:
            self.encoders = self.a_star.read_encoders()
            rospy.loginfo(f"Encoder values: Left= {self.encoders[0]}, Right= {self.encoders[1]}")  # Print encoder values
            self.battery_voltage = self.a_star.read_battery_millivolts() / 1000
            self.buttons = self.a_star.read_buttons()
        except Exception as e:
            rospy.logerr(f"Failed to read robot state: {e}")

    def publish_battery_state(self):
        # Calculate battery percentage based on voltage
        min_v = 1.1*6  # Empty batteries
        max_v = 1.4*6  # Fully charged batteries
        battery_percentage = ((self.battery_voltage - min_v) / (max_v - min_v)) * 100.0
        battery_percentage = max(min(battery_percentage, 100.0), 0.0)  # Ensure percentage is within [0, 100]

        # Publish battery state
        battery_state_msg = BatteryState()
        battery_state_msg.header.stamp = rospy.get_rostime()
        battery_state_msg.voltage = self.battery_voltage
        battery_state_msg.percentage = battery_percentage

        self.battery_state_publisher.publish(battery_state_msg)

    def run(self):
        self.rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.read_robot_state()
            self.publish_battery_state()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        romi = RobotInterface()
        romi.run()
    except rospy.ROSInterruptException:
        pass
