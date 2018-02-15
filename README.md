# Mastering-ROS-for-Robotics-Programming-Second-Edition

This is the code repository for [Mastering ROS for Robotics Programming - Second Edition]https://www.packtpub.com/hardware-and-creative/mastering-ros-robotics-programming-second-edition?utm_source=github&utm_medium=repository&utm_content=9781788478953). It contains all the supporting project files necessary to work through the book from start to finish.

## About the Book
In today’s era, robotics has been gaining a lot of traction in various industries where consistency and perfection matters most. Automation plays a major role in our world, and most of this is achieved via robotic applications and various platforms that support robotics. The Robot Operating System (ROS) is a modular software platform to develop generic robotic applications. This book focuses on the most stable release of ROS (Kinetic Kame), discusses the advanced concepts in robotics, and shows you how to program using ROS.
This book begins with a deep overview of the ROS framework, which will give you a clear idea of how ROS really works. During the course of the book, you’ll learn how to build models of complex robots, and simulate and interface the robot using the ROS MoveIt! motion planning library and ROS navigation stacks. We’ll also teach you to leverage several ROS packages to embrace your robot models.

At the end of this book, you’ll discover the best practices to follow when programming using ROS.

## Instructions and Navigation

All of the code is organized into folders. For example, Chapter04.


The code will look like the following:
```
<launch> 
 <group ns="/"> 
  <param name="rosversion" command="rosversion roslaunch" /> 
  <param name="rosdistro" command="rosversion -d" /> 
  <node pkg="rosout" type="rosout" name="rosout" respawn="true"/> 
 </group> 
</launch>
```


## Related Embedded Linux Programming Products


