---
title: 'Chapter 6: Building and Simulating Robots in Isaac Sim'
description: 'Detailed tutorials on creating and manipulating robots in NVIDIA''s Omniverse environment.'
sidebar_position: 7
---

# Chapter 6: Building and Simulating Robots in Isaac Sim

[TOC]

## Learning Objectives

By the end of this chapter, you will be able to:
- Import a URDF robot model into Isaac Sim.
- Configure the physics properties and joints of a robot.
- Use the ROS 2 bridge to connect Isaac Sim with your ROS 2 environment.
- Control a simulated robot in Isaac Sim using Python scripting and ROS 2 messages.
- Add sensors to your robot and visualize their data.

## 6.1 From URDF to USD

While Isaac Sim uses the USD format natively, it provides tools to import existing robot models from URDF. This is a crucial feature, as it allows us to leverage the vast ecosystem of open-source robot models.

### 6.1.1 Importing a URDF

Isaac Sim has a built-in URDF Importer tool.
1.  Go to `Window > Extensions` and make sure the `URDF Importer` extension is enabled.
2.  Go to `Isaac Utils > URDF Importer`.
3.  Select your URDF file (you can use the one from Chapter 4 or download a more complex one like the Franka Emika Panda arm).
4.  The importer will create a USD version of your robot, which you can then save and add to your scenes.

> **Note:** The importer will automatically convert the URDF's links and joints into USD prims with physics properties. You may need to fine-tune these properties for accurate simulation.

## 6.2 Configuring Physics and Joints

Once your robot is imported, you'll need to configure its physical properties.

### 6.2.1 Physics Properties

Select a link of your robot in the Stage. In the Property panel, you can add and edit its physics properties, such as:
- **Mass:** The mass of the link.
- **Collision:** Define the shape of the collision mesh. It's often better to use a simpler shape (like a box or sphere) than the detailed visual mesh for faster collision checking.
- **Material:** Define properties like friction and restitution (bounciness).

### 6.2.2 Joint Drives

To control the robot's joints, you need to configure their **drives**. A drive specifies how a joint can be moved.
1.  Select a joint in the Stage.
2.  In the Property panel, you'll find the drive settings.
3.  For a revolute joint, you can set a **position drive** or a **velocity drive**.
    - A position drive will try to move the joint to a target angle.
    - A velocity drive will try to move the joint at a target speed.
4.  You can also set the **stiffness** and **damping** of the drive, which affect how "springy" or "damped" the joint's movement is.

## 6.3 The ROS 2 Bridge

The **ROS 2 Bridge** is a key feature of Isaac Sim that allows it to communicate with the broader ROS 2 ecosystem. This means you can use the ROS 2 nodes, tools, and libraries that you've already learned about to control your robot in Isaac Sim.

To enable the ROS 2 bridge:
1.  Go to `Window > Extensions` and enable the `omni.isaac.ros2_bridge` extension.
2.  In the Isaac Sim interface, you can then add ROS 2 bridge components to your robot to publish or subscribe to specific topics.

### 6.3.1 Publishing Joint States

You can add a `ROS2 Publish Joint State` component to your robot. This will publish the current state of all the robot's joints to the `/joint_states` topic, which can then be used by tools like `robot_state_publisher` and RViz.

### 6.3.2 Subscribing to Joint Commands

To control the robot, you can add a `ROS2 Subscribe Joint State` component. This will subscribe to a topic (e.g., `/joint_command`) and use the received messages to set the target for the joint drives you configured earlier.

## 6.4 Controlling Robots with Python

Isaac Sim has a powerful Python scripting interface that gives you direct access to every aspect of the simulation. This is often used for high-level control logic and for integrating with AI frameworks.

### 6.4.1 The Standalone Python API

You can write a Python script that imports the Isaac Sim libraries, loads a scene, and controls the simulation. This is great for running experiments and training AI models.

Here's a snippet of what a standalone Python script might look like:

```python
from omni.isaac.kit import SimulationApp

# Start the simulation
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from my_robot_controller import RobotController

# Create a world and a robot controller
world = World()
robot_controller = RobotController()

# Add the robot to the world
world.scene.add(robot_controller.robot)

# Run the simulation loop
while simulation_app.is_running():
    world.step(render=True)
    robot_controller.do_something()

simulation_app.close()
```

### 6.4.2 Sending Commands via ROS 2

Alternatively, you can use the ROS 2 bridge and write a standard ROS 2 Python node to control the robot, just like you did for the Gazebo simulation. This is a great way to keep your control code separate from the simulation environment.

## 6.5 Adding and Visualizing Sensors

Isaac Sim supports a wide range of simulated sensors. You can add them to your robot model just like any other component.

1.  Find the sensor you want in the Create menu (e.g., `Isaac > Sensors > Camera`).
2.  Attach it to a link on your robot.
3.  Position and orient it as needed.

The ROS 2 bridge has components to publish data from these sensors to ROS 2 topics (e.g., `/image_raw` for a camera, `/scan` for a LiDAR). You can then visualize this data in RViz, just as you would with a real robot.

## Summary

In this chapter, you learned the practical steps for working with robots in Isaac Sim. You now know how to import a robot, configure its physics, and control it using both the ROS 2 bridge and the native Python scripting interface. This powerful combination allows you to build sophisticated robotics applications in a photorealistic, physically accurate environment.

## Exercises

1.  **Import and Configure:** Import the `simple_robot.urdf` from Chapter 4 into Isaac Sim. Configure its physics properties and joint drives.
2.  **ROS 2 Control:** Use the ROS 2 bridge to control the wheeled robot from Exercise 1. Write a ROS 2 node that sends `Twist` messages to make it drive in a square.
3.  **Add a Camera:** Add a camera to the robot and publish its images to a ROS 2 topic. Use RViz to view the camera feed.

## Further Reading

-   Isaac Sim ROS 2 Bridge Documentation: [https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros2_bridge.html](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros2_bridge.html)
-   Isaac Sim Python Scripting Tutorials: [https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/python_tutorials.html](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/python_tutorials.html)
-   Franka Emika Panda URDF (a good example of a more complex model): [https://github.com/frankaemika/franka_ros](https://github.com/frankaemika/franka_ros)
