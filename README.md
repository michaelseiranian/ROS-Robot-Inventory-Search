# TurtleBot3 Inventory Search - ROS Action Server & State Machine

## Project Overview

This project focuses on developing a **ROS-based system** for a **TurtleBot3** robot to autonomously conduct an inventory search of items in a store. The robot navigates through predefined rooms, detecting various objects, with special emphasis on locating a **cake**. The system leverages ROS services, actions, and a state machine to manage the robot’s behavior effectively.

## Features

- **Room Coordinate Service**  
  A ROS service named `GetRoomCoord` returns sequential coordinates within a given room. The robot navigates the room by following these coordinates in a systematic order.
  
- **Search Action Server**  
  The action server, implemented in the `main_node`, allows the robot to perform inventory searches. It provides **real-time feedback** on the objects detected and stops the search upon detecting a cake.

- **State Machine Behavior**  
  The robot's movement and object recognition are managed using a **state machine** implemented with **SMACH**. The robot processes its camera feed using the **YOLO** object detection algorithm while moving, ensuring it detects objects concurrently with navigation.

- **Feedback and Results**  
  The robot sends feedback at a rate of **5 Hz**, reporting the list of detected objects and their counts. When the robot detects a cake, it stops and returns the final list of objects, their quantities, and a timestamp indicating when the cake was detected.

## Technologies Used

- **ROS (Robot Operating System)**  
  - Custom **services** and **actions** for room navigation and object detection.
  - State machine implementation using **SMACH** to manage robot behavior.
  - **YOLO** object detection for inventory tracking based on camera feed analysis.

- **Python (rospy)**  
  - ROS nodes and service implementations.
  - State machine and concurrency management using SMACH.
  
- **ROS Bags**  
  - Provided video files in the form of ROS bags to simulate camera input for testing without the need for physical hardware.
