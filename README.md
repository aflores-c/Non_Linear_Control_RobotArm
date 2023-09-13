# Non_Linear_Control_RobotArm
A non linear control for a robot arm. The design and simulation of the robot arm was implemented.

https://www.youtube.com/watch?v=o141rkJ0HUA&ab_channel=robotics_af
![alt text][sim]

## Requierements

1. Coppelia Simulator, edu version is enough : https://www.coppeliarobotics.com/ 
2. Matlab 2021a - current Matlab versions also work

[non linear control]: ./images/kinematic_control.png
[forward_kinematics]: ./images/robot_structure.png
[ik1]: ./images/inverse_kinematics_1.png
[ik2]: ./images/inverse_kinematics_2.png
[ik3]: ./images/inverse_kinematics_3.png
[ik4]: ./images/inverse_kinematics_4.png
[sim]: ./images/VREP_Simulation.png


## Description 

A full description can be found in the pdf file.
The idea is to implement a non linear control called kinematic control for robot arm, which is based in the kinematic model and 
the use of Jacobians. For that, we need the kinematic robot model and and the inverse kinematic equations.

![alt text][non linear control]

## Forward Kinematics

![alt text][forward_kinematics]

## Inverse Kinematics 

![alt text][ik1]
![alt text][ik2]
![alt text][ik3]
![alt text][ik4]

## Simulation 

1. Run the RobotFinal.ttt to open the Copelia Simulator
2. Run the control_inv_robot.m file. 

![alt text][sim]


