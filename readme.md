
# ROS Beginner Tutorials

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview

This repository contains a simple publisher and subscriber node.

The tutorials were followed from:[website](http://wiki.ros.org/ROS/Tutorials).

A launch file ```beginner_tutorials_10.launch``` can be used to launch talker and listener nodes concurrently.


A service message named ```TalkerService.srv``` is used to change the output string upon the request from the client


<p align="center">
<a target="_blank"><img src="img/node_graph.png"
alt="NMPC" width="480" height="260" border="10" />
</a>
</p>


## Purpose

###### Main Features

The beginner_tutorials project builds a simple publisher and subscriber node.
talker is the publisher node which publishes the msg of data type std_msgs::string.
listener is subscriber node, which subscribe to the topic chatter.

## License

BSD 3-Clause License

Copyright (c) 2018, Saurav
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Assumptions
- ROS Kinetic is installed properly.
- ROS environment is properly configured.
- Dependencies are met


## Dependencies
- ROS Kinetic
- catkin
- roscpp package
- rospy pacakges
- std_msgs package
- genmsg package

## Standard build via command-line
```
cd <path to workspace>
mkdir src
cd src
git clone -b Week10_HW --single-branch https://github.com/sauravkdeo/beginner_tutorials.git
cd ..
catkin_make
```
## Steps to run the package

### Sourcing to .bashrc
- open the .bashrc file located in the home folder using your favorite editor.Add the undermentioned lines at the end of the file and then save it.
- This step is done to avoid sourcing  ~/```path to workspace```/devel/setup.bash every time.
```
source ~/<path to workspace>/devel/setup.bash
ex:
source ~/catkin_ws/devel/setup.bash
```


### Using rosrun

######  Open three terminals concurrently :

- Run following commands in Terminal 1 :
```
roscore
```

- Run following commands to execute talker node in Terminal 2 :
```
rosrun beginner_tutorials talker
```

- Run following commands to execute listener node in Terminal 3 :
```
rosrun beginner_tutorials listener
```

### Using roslaunch

- To use the launch file type the undermentioned command in the terminal :
```
roslaunch beginner_tutorials_10.launch
```
User can also change the frequency at which the loop operates using undermentioned command :
```
roslaunch beginner_tutorials_10.launch frequency:=<desired frequency>
ex:
roslaunch beginner_tutorials_10.launch frequency:=10
```



### Service

- User can also change the output string message by typing the following command in a new terminal
```
rosservice call /TextService "<text to be entered by the user>"
ex:
rosservice call /TextService "Hello ROS!!!"
```
