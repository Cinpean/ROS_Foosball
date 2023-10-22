# ROS_Foosball
<h1 align="center">The ROS Foosball Table project</h1>

<p align="center">
	<br>
	<a href="https://www.ros.org"><img src="https://github.com/Cinpean/ROS_Foosball/assets/35309003/6677bf4b-b719-4ae4-bd6b-6f2c8b030856" align="left" width="1604" height="427"></a>
	<a href="https://www.ros.org"><img src="https://github.com/Cinpean/ROS_Foosball/assets/35309003/29f570fe-21ea-44de-bd70-340f7aa1fd6c" align="right" alt="Size Limit logo" width="120" height="178"></a><br>
	<br>
	<br>
	<a href=https://github.com/Cinpean/ROS_Foosball/blob/main/LICENSE"><img src="https://img.shields.io/github/license/samyarsadat/ROS-Robot?color=blue"></a>
	|
	<a href=""><img src="https://img.shields.io/badge/Current_Stage-One-red"></a>
	|
	<a href="https://github.com/samyarsadat/ROS-Robot/issues"><img src="https://img.shields.io/github/issues/samyarsadat/ROS-Robot"></a>
	<br><br>
</p>

----
### Disclaimer: Robot is in development.
The ROS Robot project.
<br>
<br>
<br>
<img src="https://github.com/Cinpean/ROS_Foosball/assets/35309003/6adeacbd-e629-4fe4-9997-c4f44ffdd765">
This is a 3D render of the CAD designs.
<br>
<br>

## Overview

### Stages
***This project will be completed in stages.***<br>
*The project is currently at stage 1.*<br>
<br>
<br>

#### Stage 1 (Humble Beginnings / Follow Me)
The robot will follow an object with image processing and a Raspberry Pi camera.<br>
<br>

#### Stage 2 (S.L.A.M. It Shut)
A LIDAR sensor will be added.<br>
The robot will be able to map and navigate its environment.<br>
<br>

#### Stage 3 (Samyarm 1)
A robotic arm will be added for object manipulation.<br>
<br>

#### Stage 4 (Back To The Shipyard)
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
The general architecture of the robot's electronics system consists of a main computer *(the Raspberry Pi 4B)* .
The Pi 4 handles image processing & sending information to stepper drivers<br>
<br>
ROS is used to handle communications between multiple nodes either on the same machine (i.e. a mapping node and a navigation node running on the Pi 4) or between external nodes (i.e. the two Raspberry Pi Picos running microROS). 
Both Raspberry Pi Picos are connected to the Pi 4 via USB cables.<br>
<br>
More details regarding the electronics design <a href="https://github.com/samyarsadat/ROS-Robot/tree/stage-1/Circuit%20diagrams%20%26%20PCB%20files">here</a>.<br>
<br>

#### Mechanical Design Overview
The mechanical design of the robot is quite simple. 
The chassis is 3D printed using PLA filament (more details regarding 3D printing <a href="https://github.com/samyarsadat/ROS-Robot/tree/stage-1/CAD%20files/STL%20files">here</a>) and the robot uses 3 stepper motors and 3 servos<be>

<br>

## Contact
You can contact me via e-mail.<br>
E-mail: @gmail.com<br>

## Contributing
Please take a look at <a href="https://github.com/samyarsadat/ROS-Robot/blob/dev/CONTRIBUTING.md">CONTRIBUTING.md</a> for contributing.

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
