---
title: 'Chapter 4: Simulating Robots with Gazebo'
description: 'Creating and controlling robots in a simulated environment using Gazebo.'
sidebar_position: 5
---

# Chapter 4: Simulating Robots with Gazebo

[TOC]

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the role of simulation in robotics.
- Create a simple robot model using the Unified Robot Description Format (URDF).
- Launch a robot model in the Gazebo simulator.
- Control a simulated robot using ROS 2 messages.
- Use RViz to visualize the simulated robot and its sensor data.

## 4.1 Introduction to Gazebo

**Gazebo** is a powerful 3D robotics simulator. It allows you to simulate robots in complex indoor and outdoor environments, with a high degree of physical fidelity. With Gazebo, you can:
- **Model a wide variety of robots:** From simple wheeled robots to complex humanoids.
- **Simulate realistic physics:** Including gravity, friction, and collisions.
- **Simulate sensors:** Gazebo can generate realistic data from cameras, LiDAR, IMUs, and more.
- **Integrate with ROS 2:** Gazebo is tightly integrated with ROS 2, allowing you to use the same code for both your simulated and physical robots.

## 4.2 Creating a Robot Model with URDF

To simulate a robot, we first need to describe it. In ROS, the standard format for this is the **Unified Robot Description Format (URDF)**. A URDF file is an XML file that describes the physical properties of a robot.

A URDF model is composed of **links** and **joints**:
- **Links:** The rigid parts of the robot (e.g., the body, wheels, arms). Each link has visual, collision, and inertial properties.
- **Joints:** The connections between links. Joints define how links can move relative to each other (e.g., revolute for a wheel, continuous for a rotating sensor).

### 4.2.1 A Simple Wheeled Robot URDF

Let's create a URDF for a simple two-wheeled robot. Create a new package for this chapter's work (`ros2 pkg create --build-type ament_python gazebo_robot`). Inside this package, create a `urdf` directory, and in it, a file named `simple_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
  </link>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_to_right_wheel" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 0" rpy="1.5707 0 0"/>
  </joint>

  <!-- Add a left wheel similarly -->

</robot>
```
> **Note:** This is a simplified example. A complete URDF would also include the left wheel and inertial properties for each link.

## 4.3 Launching Gazebo with a Robot

To launch our robot in Gazebo, we'll use a launch file. The `ros_gz_sim` package provides a bridge between ROS 2 and Gazebo.

### 4.3.1 Gazebo Launch File

In the `launch` directory of your new package, create `spawn_robot.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    pkg_gazebo_robot = get_package_share_directory('gazebo_robot')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items()
    )

    # Spawn robot
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-file', os.path.join(pkg_gazebo_robot, 'urdf', 'simple_robot.urdf'),
                                   '-name', 'simple_robot'],
                        output='screen')

    return LaunchDescription([
        gazebo,
        spawn_entity,
    ])
```

Now, when you build your workspace and run this launch file, Gazebo will open with your robot model spawned in an empty world.

## 4.4 Controlling the Robot

To make the robot move, we need to add a **Gazebo plugin** to our URDF. This plugin will expose a ROS 2 interface for controlling the robot's joints.

### 4.4.1 Adding the Diff Drive Plugin

We'll use the `diff_drive` plugin, which allows us to control a two-wheeled robot by publishing velocity commands. Add this to your `simple_robot.urdf` file:

```xml
<gazebo>
  <plugin name="gazebo_ros_diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>/</namespace>
      <remapping>cmd_vel:=cmd_vel_demo</remapping>
      <remapping>odom:=odom_demo</remapping>
    </ros>
    <left_joint>base_to_left_wheel</left_joint>
    <right_joint>base_to_right_wheel</right_joint>
    <wheel_separation>0.35</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <!-- Other parameters -->
  </plugin>
</gazebo>
```

### 4.4.2 Sending Velocity Commands

With this plugin, Gazebo will now subscribe to the `/cmd_vel_demo` topic for velocity commands. The message type is `geometry_msgs/msg/Twist`.

We can send a command from the command line to make the robot move forward:

```bash
ros2 topic pub /cmd_vel_demo geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

You should see your robot start to move in Gazebo!

## 4.5 Visualization with RViz2

While Gazebo shows you the "ground truth" of the simulation, RViz2 is essential for visualizing what your robot is "thinking." It shows you the data from the robot's perspective.

To use RViz2 with our simulated robot, we need a node to publish the robot's state and TF2 transformations. The `robot_state_publisher` node does exactly this.

Update your launch file to include `robot_state_publisher`, and then run `rviz2`. In RViz, you can add displays for:
- **RobotModel:** To see the robot's URDF.
- **TF:** To visualize the coordinate frames.
- **LaserScan** or **Image:** To see sensor data (once you add sensors to your robot).

## Summary

In this chapter, you took a major step into the world of robotics by simulating your first robot. You learned how to describe a robot using URDF, how to launch and control it in Gazebo, and how to visualize its state in RViz2. This "simulation-first" approach is fundamental to modern robotics development.

## Exercises

1.  **Complete the URDF:** Add the left wheel to the `simple_robot.urdf` to make it a complete two-wheeled robot.
2.  **Create a Control Node:** Write a Python node that publishes to `/cmd_vel_demo` to make the robot drive in a circle.
3.  **Add a Sensor:** Add a LiDAR sensor to your robot's URDF using a Gazebo sensor plugin. Use RViz to visualize the laser scan data.

## Further Reading

-   Gazebo Documentation: [https://gazebosim.org/](https://gazebosim.org/)
-   URDF Tutorials: [http://wiki.ros.org/urdf/Tutorials](http://wiki.ros.org/urdf/Tutorials)
-   Gazebo ROS 2 Plugins: [https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos](https://github.com/gazebosim/ros_gz/tree/ros2/ros_gz_sim_demos)
