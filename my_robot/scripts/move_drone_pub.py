#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point
import tf.transformations as tft

desired_horizontal_offset = 3.0
desired_vertical_offset = 5.0

def model_states_callback(msg, drone_publisher):
    try:
        # Find the index of the "robot" in the model_states message
        robot_index = msg.name.index("robot")

        # Extract the pose of the "robot"
        robot_pose = msg.pose[robot_index]

        # Calculate the pose of the "drone" with the desired offsets
        drone_pose = calculate_drone_pose(robot_pose)

        # Publish the updated drone pose
        drone_publisher.publish(drone_pose)
    except ValueError:
        # Robot not found in the model_states message
        pass

def calculate_drone_pose(robot_pose):
    # Extract position and orientation from the robot's pose
    robot_position = robot_pose.position
    robot_orientation = robot_pose.orientation

    # Calculate the horizontal offset vector based on the desired horizontal offset
    horizontal_offset_vector = Point(-desired_horizontal_offset, 0.0, 0.0)

    # Rotate the horizontal offset vector based on the robot's orientation
    quaternion = [robot_orientation.x, robot_orientation.y, robot_orientation.z, robot_orientation.w]
    rotation_matrix = tft.quaternion_matrix(quaternion)[:3, :3]
    rotated_horizontal_offset = rotation_matrix.dot([horizontal_offset_vector.x, horizontal_offset_vector.y, horizontal_offset_vector.z])

    # Calculate the vertical offset vector based on the desired vertical offset
    vertical_offset_vector = Point(0.0, 0.0, desired_vertical_offset)

    # Calculate the drone's position by adding the rotated horizontal offset and vertical offset to the robot's position
    drone_position = Point(robot_position.x + rotated_horizontal_offset[0], robot_position.y + rotated_horizontal_offset[1], robot_position.z + rotated_horizontal_offset[2] + vertical_offset_vector.z)

    # Use the same orientation as the robot
    drone_orientation = robot_orientation

    # Create the drone's pose
    drone_pose = Pose(drone_position, drone_orientation)

    return drone_pose

def main():
    rospy.init_node('robot_drone_sync_node')

    drone_publisher = rospy.Publisher('/gazebo/drone/pose', Pose, queue_size=10)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_states_callback, drone_publisher)

    rate = rospy.Rate(100) 

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
