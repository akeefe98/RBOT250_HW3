# RBOT250_HW3
Homework 3 for RBOT 250

The first assignment was to write an rclpp node that reads a homogeneous TransformStamped.msg and then broadcasts it on a new topic called 'homeworks/hw1/tf/' and the parameters for the tranform are provided:
header frame id: base
child frame id: elbow
vector translation: 1.57, 3.142, -2*pi/3
q rotation: 0.123, 1.57, 5*pi/6, 1

This node is created by sourcing the setup script
. install/setup.bash
And then running the cpp program
ros2 run rbot250 homework1

It's called homeowork1 because there are two assignments for week 3, and they are labeled homework1 and homework2.

A subscriber python node was also requested, that reads the 'homeworks/hw1/tf' topic at a rate of 30Hz.

This node is created by sourcing the setup script
. install/setup.bash
And then running the python script
ros2 run rbot250 subscriber_member_function.py

The name comes from the tutorial I based the code on.
