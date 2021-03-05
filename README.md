# Autonomous Highway Simulation
![TwoBots](/assets/qmindlogo.png)

## QMIND DAIR Highway Simulation Team
### By Andrew Simonds, Dylan Moss, Brock MacDonald, Azeem Quadri

## Inspiration
As major automotive companies continue to incorporate new forms of automation to their fleet of vehicles, one of the major tasks that remains is perfecting self-driving capabilities on highways.  The goal of this project is to use a Turtlebot and the sensors that are provided on the robot to simulate various aspects of an autonomous highway. 

## What it does
The current state of the project allows for lane detection, automated steering, detection of other robots on the highway, and speed corrections dependant on distance of other robots in the vicinity.  
![TwoBots](/assets/laneDetection.png)

## How we built it
### Hardware
Due to the global pandemic, the team used a simulated version of the Turtlebot Waffle-Pi to allow for collaboration.  The robot is equipped with a variery of sensors such as a camera, LiDAR, and an IMU.
### Software
Robot Operation System (ROS) was used for publishing and subscribing to all of the topics required for turtlebot operations.  We used ROS2 Dashing as it is one of the more recent versions of the software.  The team also used a multitude of Python scripts to facilitate the publishers and subscribers required for ROS, and to incorporate a variety of machine learning techniques.
### Simulation
Gazebo was used to simulate the Turtlebots and was also perfect for loading in the custom road world used for the testing of features.
![TwoBots](/assets/twoBots.jpg)

## What's next for the project
The team will link up with the other Highway Simulation group to incorporate some of the features they have been working on into one greater simulation.  These features include multi-lane detection, lane changing, and passing another turtlebot.
