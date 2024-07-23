# Getting multiple robots to properly display in Rviz and Gazebo

1. Use the group tag to create two robot namespaces.  
    ```
    <group ns="robot1">
    </group>
    ```

2. Each has a different TF prefix which is entered into RViz.
    ```
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
        <param name="tf_prefix" value="robot" />
    </node>
    ```

3. The gazebo_control.xacro RobotBaseFrame was updated to the new namespace.

4. The `/cmd_vel` topic can be remapped to either `/robot1/cmd_vel` or `/robot2/cmd_vel`.

## Resolved issues

Gazebo issue: PublishWheelTf was set to false as it was publishing right_wheel and left_wheel transforms from base_link when the new frames were `robot1/base_link` and `robot2/base_link`.
