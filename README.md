# nav200_laser
ROS package for SICK NAV200 laser

## 1-Installation of SICK configuration tool

[Download SICK NAV200-Setup](https://www.sick.com/de/en/nav200-setup/p/p197041)

## 2- Overview

This package contains launch file for  setting up serial connection with Sick NAV200 laser positioning system and publishing  the position and the orientation data according to map frame(world fixed frame).

## 3-Quick Start

Launch the NAV200 driver.

>roslaunch nav200_laser robotnik_nav200.launch

You can see the published data by listening /node_position/pose topic.

>rostopic echo /node_position/pose

## 4-Launch File
### 4.1-Parameters

You can change the parameters from config.yaml file in the package.

* port(string, default: /dev/ttyUSB0)
This is the serial port file name. This file is created by the system in /dev file when serial port attached to the computer.
* mode(string, default:automatic_speed, other option:feeding_speed)
In the default mode, NAV200 calculates its own speed.
The other mode is feeding speed, in this mode NAV200 takes the speed data fron the /odom topic.

### 4.2-Published Topics
* /node_position/pose
###4.3-Subscribed Topics
* /odom
/odom topic is subscribed because, NAV200 node broadcasts /map to /odom transformation and also uses speed data in the feeding speed mode.
