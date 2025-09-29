# ROS2 Turtle Shapes Project
## 📌 Overview

This project controls the Turtlesim in ROS2 to draw different shapes:

🟥 Pentagon

💖 Heart

♾️ Infinity

The project uses a custom ROS2 package (my_controller) with two main nodes:

shapeNode → lets the user choose which shape to draw (or stop).

turtleCommander → receives the selected shape and commands the turtle to move.

## 🛠️ Requirements

ROS2 Humble (or compatible version)

Python 3

Turtlesim package

Install Turtlesim if not already installed:

  sudo apt install ros-humble-turtlesim

# 🚀 How to Run

## 1. Clone this repository:

git clone https://github.com/AbdelrahmanSallam04/ros2-turtle-shapes.git
cd ros2_ws
colcon build
source install/setup.bash


## 2. Open Terminal 1 and run the launch file:

ros2 launch my_controller shapes_launch.py


👉 This will start:

turtlesim_node

shapeNode

turtleCommander

## 3. Open Terminal 2 and run the shapeNode directly to choose a shape:

ros2 run my_controller shapeNode


## 4. From the terminal, enter one of the following:

pentagon → 🟥 Draws a pentagon

heart → 💖 Draws a heart

infinity → ♾️ Draws an infinity shape

stop → ⏹ Stops the turtle
