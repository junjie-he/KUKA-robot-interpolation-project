# KUKA-robot-interpolation-project
1. Introduction

This project is one of the WAAM projects which were conducted in big robot hall of the RWTH Aachen university in 2022, this project is designed to explore the possibility and feasibility of the implementation of interpolation method in the welding process in order to improve the welding quality. The welding robot for this project is KUKA KR30 and the programming framework ROS2 and the simulation environment Rviz2 will be used to complete the tasks of this project.

 

1.1 Scope

The primary goal of this project is to visualize both of the interpolated path and the original path of the TCP (Tool Center Point) of the KUKA KR30 robot during welding process as well as to visualize the TCP translational speed in the operational space in its digital twin and the deviation between the original path and interpolated path is finally to be measured.

 

1.2 Milestones

This interpolation project can be divided into four major tasks and each task shall be checked and delivered according to our project schedule. The four stages of this project include:

• Create the URDF file for the robot and the dynamic model in Rviz
• Write a python script as ROS2 node to subscribe MQTT messages
• Parse JSON messages and synchronize the state of digital twin in Rviz
• Visualize the interpolated path and original path and measure the deviation
 

2. Project concept

2.1 Background

The current situation in the welding robot industry is that the welding quality is affected by the inconsistent welder moving speed. When the welder moving relatively slow, the width of face and root of weld will increase because bead that has too much weld deposit will be created in the welding area. Oppositely, when the welding speed increases, the higher moving speed of welder will cause bead width reduced and too little weld deposit will be applied into the welding area, which may cause poor welding quality and problems such as insufficient welding and porosity of the weld. [1] How the bead width and weld quality are influenced by welding speed is illustrated in the figure 1 below.



Figure 1. Weld quality of different welding speed [1]

 

2.2 Interpolation method

To prevent the problems caused by inconsistent welder moving speed, the interpolation method can be implemented for the trajectory planning of the TCP during the welding process.



Figure 2. Speed change for original path (left) and interpolated path (right)

 

3. Deviation

how to calculate the deviation between the real path and interpolated path:  

To measure the deviation between two paths, original and interpolated, in ROS2 with waypoints involves calculating the Euclidean distance between each waypoint in the original path and the corresponding waypoint in the interpolated path. To do this, you should extract the waypoints from the original and interpolated path. After that we have to calculate the Euclidean distance between the two waypoints.

The Euclidean distance between two points in a multi-dimensional space can be calculated using the formula:

Euclidean distance = √((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2 + …)

Average the distances calculated in last step to get the average deviation between the original and interpolated paths.

 

4. Simulation

The movements and welding trajectory is to be simulated and visualized in the Rviz simulation environment, in order to visualize the digital twin in Rviz, the URDF



 

5. Results

Upon completion of this project, the feasibility of implementation of interpolation method in the trajectory planning during the welding process has been tested.

 

6. Outlook

This project is only preliminary project for implementing interpolation method in the trajectory planning of welding robot. Further research can be focusing on the algorithm part of trajectory planning implementation,
