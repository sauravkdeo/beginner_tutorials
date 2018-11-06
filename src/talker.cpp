/**
 * @file      talker.cpp
 * @brief     Publishes  msg to topic chatter
 * @author    Saurav Kumar
 * @copyright 2018
 *
 **BSD 3-Clause License
 *
 *Copyright (c) 2018, Saurav Kumar
 *All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived from
 *  this software without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "../include/talker.h"
#include "beginner_tutorials/TalkerService.h"

Talker::Talker()
    : textvalue("Hello ROS!!"), count(1), frequency(5) {
}
bool Talker::callbackfunction
(beginner_tutorials::TalkerService::Request &req,
  beginner_tutorials::TalkerService::Response &resp) {
  if (req.request_string == "") {
    ROS_WARN_STREAM("No Input !! Considering defalut value");
    textvalue = "Hello ROS!!";
  } else {
  ROS_INFO("Request : %s", req.request_string.c_str());
  textvalue = req.request_string;
  count = 1;
  resp.response_string = textvalue;
  ROS_INFO("Respond : %s", resp.response_string.c_str());
}
  return true;
}

std::string Talker::getText(void) {
    return textvalue;
  }
int Talker::getCount(void) {
    return count;
  }
void Talker::setCount(int countvalue) {
    count = countvalue;
  }

void Talker::setFrequency(double freq) {
  frequency = freq;
  if (freq > 0) {
      ROS_DEBUG_STREAM("loop operating at frequency of: "<< frequency);
    } else if (freq < 0) {
      ROS_ERROR_STREAM("The input frequency cannot be negative");

      ROS_WARN_STREAM("Setting to default frequency of 5Hz");

      // setting frequency to 5 hz
      frequency = 5;
    } else if (freq == 0) {
      ROS_FATAL_STREAM("Input frequency cannot be 0Hz");

      ROS_WARN_STREAM("Setting to default frequency of 5Hz");

      // setting frequency back to 5Hz
      frequency = 5;
  }
}
double Talker::getFrequency() {
  return frequency;
}
