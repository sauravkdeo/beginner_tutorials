# beginner_tutorials
---

## Overview

The beginner_tutorials project builds a simple publisher and subscriber node.

The tutorials were followed from: http://wiki.ros.org/ROS/Tutorials.


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
git clone --recursive https://github.com/sauravkdeo/beginner_tutorials.git
cd ..
catkin_make
```

## Steps to run the package

######  Open three terminals concurrently :

- Run following commands in Terminal 1 :
```
roscore
```

- Run following commands to execute talker node in Terminal 2 :
```
cd <path to workspace>
source devel/setup.bash
rosrun beginner_tutorials talker
```

- Run following commands to execute listener node in Terminal 3 :
```
cd <path to workspace>
source devel/setup.bash
rosrun beginner_tutorials listener
```
