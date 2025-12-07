---
title: 'Chapter 2: ROS 2 Fundamentals'
description: 'An introduction to the Robot Operating System (ROS 2), including nodes, topics, services, and actions.'
sidebar_position: 3
---

# Chapter 2: ROS 2 Fundamentals

[TOC]

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the purpose and architecture of ROS 2.
- Understand the concepts of ROS 2 nodes, topics, services, and actions.
- Write a simple ROS 2 node in Python.
- Use ROS 2 command-line tools to inspect and debug a ROS 2 system.

## 2.1 What is ROS 2?

The **Robot Operating System (ROS)** is a set of software libraries and tools that help you build robot applications. Despite its name, ROS is not a traditional operating system but a flexible framework for writing robot software. It provides:
- **Hardware Abstraction:** A way to interact with a wide range of robot hardware without writing low-level code.
- **Inter-process Communication:** A messaging system that allows different parts of your robot's software to communicate with each other.
- **A Rich Ecosystem of Tools:** Tools for visualization, simulation, debugging, and more.

**ROS 2** is the second generation of ROS, redesigned from the ground up to support new use cases, including multi-robot systems, real-time control, and production environments.

> **Note:** We are using ROS 2 in this course because it is the current standard and offers significant improvements over ROS 1.

## 2.2 Core ROS 2 Concepts

A ROS 2 system is a distributed network of processes called **nodes**. These nodes communicate with each other using a few fundamental concepts.

### 2.2.1 Nodes

A **node** is the smallest unit of computation in ROS 2. You can think of a node as a single program that performs a specific task. For example, you might have one node for controlling the robot's wheels, another for processing camera data, and a third for planning a path.

### 2.2.2 Topics

**Topics** are named buses over which nodes exchange messages. Topics use a publish-subscribe communication pattern:
- A **publisher** is a node that sends messages to a topic.
- A **subscriber** is a node that receives messages from a topic.

Many nodes can publish or subscribe to the same topic, making it a powerful way to decouple different parts of your system. For example, a camera node might publish images to an `/image_raw` topic, and a computer vision node could subscribe to this topic to process the images.

*Diagram: A diagram showing three nodes. A "Camera Driver" node publishes to an "/image_raw" topic. A "Computer Vision" node and a "Logger" node both subscribe to the "/image_raw" topic.*

### 2.2.3 Services

**Services** are used for request-response communication. One node (the **server**) provides a service, and another node (the **client**) can call that service. Unlike topics, services are synchronous: the client sends a request and waits for a response from the server.

Services are useful for tasks that have a clear beginning and end, like "capture a single image" or "calculate the distance to an object."

### 2.2.4 Actions

**Actions** are similar to services but are used for long-running tasks that provide feedback. An action has three parts:
- A **goal:** The request sent by the client to the server.
- **Feedback:** Messages sent by the server to the client to provide updates on the task's progress.
- A **result:** A message sent by the server when the task is complete.

Actions are ideal for tasks like "navigate to a waypoint," where you want to know how the task is progressing and when it's finished.

## 2.3 Setting up Your ROS 2 Environment

Before you can write your first ROS 2 node, you need to install ROS 2 and set up your environment.

### 2.3.1 Installation

We will be using the **ROS 2 Humble Hawksbill** release, which is the latest long-term support (LTS) release. Detailed installation instructions can be found on the official ROS 2 documentation website.

> **Installation Guide:** Follow the official installation guide for your operating system (Ubuntu 22.04 is recommended): [https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

### 2.3.2 Sourcing the Setup File

Every time you open a new terminal to work with ROS 2, you need to "source" the setup file. This command sets up the environment variables needed for ROS 2 to work correctly.

```bash
source /opt/ros/humble/setup.bash
```

You can add this line to your `~/.bashrc` file to have it run automatically every time you open a new terminal.

## 2.4 Your First ROS 2 Node in Python

Let's create a simple "hello world" node in Python. This node will publish a message to a topic, and we'll use a command-line tool to see the message.

### 2.4.1 Creating a Workspace and a Package

First, we need to create a **workspace** to hold our ROS 2 packages, and then a **package** to hold our code.

```bash
# Create a workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create a new package
ros2 pkg create --build-type ament_python my_first_package
```

### 2.4.2 Writing the Publisher Node

Now, let's write the Python code for our publisher node. Create a file named `hello_publisher.py` inside `my_first_package/my_first_package/`.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    hello_publisher = HelloPublisher()
    rclpy.spin(hello_publisher)
    hello_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.4.3 Building and Running the Node

To run this node, we first need to build our package. Navigate to the root of your workspace (`~/ros2_ws`) and run:

```bash
colcon build
```

After the build is finished, source the new setup file in the `install` directory:

```bash
source ~/ros2_ws/install/setup.bash
```

Now, you can run your node:

```bash
ros2 run my_first_package hello_publisher
```

You should see the "Publishing: 'Hello, ROS 2!'" message printed in your terminal.

## 2.5 ROS 2 Command-Line Tools

ROS 2 provides a powerful set of command-line tools for inspecting and interacting with your system.

- `ros2 node list`: Lists all running nodes.
- `ros2 topic list`: Lists all active topics.
- `ros2 topic echo <topic_name>`: Prints messages from a topic to the console.

Open a new terminal (and source the setup files), and try running `ros2 topic echo /hello_topic`. You should see the messages being published by your node.

## Summary

In this chapter, you learned the fundamental concepts of ROS 2, including nodes, topics, services, and actions. You set up a ROS 2 workspace, wrote your first Python node, and used command-line tools to interact with it. This forms the foundation for all the robotics software we will build in this course.

## Exercises

1.  **Create a Subscriber:** Write a new Python node called `hello_subscriber.py` that subscribes to the `/hello_topic` and prints the received messages to the console.
2.  **Change the Message:** Modify the `hello_publisher.py` to publish a different message and change the publishing frequency.
3.  **Explore:** Use the `ros2` command-line tools to find out the message type of the `/hello_topic`.

## Further Reading

-   ROS 2 Documentation: [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)
-   *ROS 2 in 5 Days* - A comprehensive online course from The Construct.
