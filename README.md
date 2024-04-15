# ROS_Foosball
<h1 align="center">The ROS Foosball Table project</h1>
<a href="https://www.ros.org"><img src="https://github.com/Cinpean/ROS_Foosball/assets/35309003/6677bf4b-b719-4ae4-bd6b-6f2c8b030856" align="center" width="640" height="171"></a>
<a href="https://www.ros.org"><img src="https://github.com/Cinpean/ROS_Foosball/assets/35309003/29f570fe-21ea-44de-bd70-340f7aa1fd6c" align="center" alt="Size Limit logo" width="136" height="170"></a><br />
<p align="center">
	<br>
	<br>
	<a href=https://github.com/Cinpean/ROS_Foosball/blob/main/LICENSE><img src="https://img.shields.io/badge/Licence-MIT-blue"></a>
	|
	<a href=""><img src="https://img.shields.io/badge/Current_Stage-One-red"></a>
	|
	<a href="https://github.com/Cinpean/ROS_Foosball/issues"><img src="https://img.shields.io/badge/Stauts-Private-cyan"></a>
	<br><br>
</p>

----
### Disclaimer: Robot is in development.
The ROS Robot project.
<br>
<br>
<br>
<img src="https://raw.githubusercontent.com/Cinpean/ROS_Foosball/main/Assets/Renders_img/Foosball_V5.JPG?token=GHSAT0AAAAAACNMGPMOJJMQLYWU4QOWKIRUZQ44GHA">
This is a 3D render of the CAD design.
<br>
<br>

## Overview

### Stages
***This project will be completed in stages.***<br>
*The project is currently at stage 4.*<br>
<br>
<br>

#### Stage 1
The robot will follow an object with image processing and a Raspberry Pi camera.<br>
<br>

#### Stage 2 
Steppers & servos will be added.<br>
The robot will be able to move each row of players. <br>
<br>

#### Stage 3 
Code will be added so that the players intercept the ball.<br>
<br>

#### Stage 4 
After stage 3 is completed, I plan to re-design some parts of the robot chassis and PCB.<br>
This will improve upon the current design and allow for future stages and expansions.<br>
Most of the wiring and general design will be kept the same.<br>
This will only improve upon the current design.<br>
<br>

#### Future stages
More stages may be added as the project progresses.

<br>

### System Architecture and Mechanical Design Overview
#### System Architecture
The general architecture of the robot's electronics system consists of a main computer *(the Raspberry Pi 4 Model B)* .
The Pi 4 handles image processing & sending information to stepper drivers<br>
<br>
ROS is used to handle communications between multiple nodes either on the same machine or between external nodes. <br>
<br>
More details regarding the electronics design <a href="https://github.com/Cinpean/ROS_Foosball/tree/main/Circuit_diagam">here</a>.<br>
<br>

#### Mechanical Design Overview
The mechanical design of the robot is quite simple. 
The chassis is 3D printed using PLA filament (more details regarding 3D printing <a href="https://github.com/Cinpean/ROS_Foosball/tree/main/CAD_Files">here</a>) and the robot uses 3 stepper motors and 3 servos<be>

<br>

## Contact
You can contact me via e-mail.<br>
E-mail: @gmail.com<br>

<br>

## Credits
| Role           | Name                                                             |
| -------------- | ---------------------------------------------------------------- |
| Lead Developer | <a href="https://github.com/cinpean">Cinpean Sebastian</a> |
| CAD Design     | <a href="https://github.com/cinpean">Cinpean Sebastian</a> |
| PCB Design     | <a href="https://github.com/cinpean">Cinpean Sebastian</a> |

<br>
<br>

Copyright © 2023-2024 Cinpean Sebastian.
