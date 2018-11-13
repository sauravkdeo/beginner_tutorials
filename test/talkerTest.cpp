/**
 * @file      test/talkerTest.cpp
 * @brief     It tests the talkernode
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
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include "beginner_tutorials/TalkerService.h"

/**
 * @brief      Tests whether the service exists
 * @param      testTalkerNode         gtest framework
 * @param      serviceExsistanceTest  Name of the test
 */

TEST(TESTSuite, testServiceExists) {
  // Create node handle
  ros::NodeHandle n;
  // Register the client to the service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::TalkerService>("TextService");
  // Check if the service exist
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}

/**
 * @brief  Tests whether the service changes the text
 *
 * @param  TESTSuite              gtest framework
 * @param  testChangeTestService  Name of the test
 */
TEST(TESTSuite, testChangeTestService) {
  // Create a node handle
  ros::NodeHandle n;
  // Register the client to the service
  ros::ServiceClient client =
      n.serviceClient<beginner_tutorials::TalkerService>("TextService");
  beginner_tutorials::TalkerService srv;
  // Set the input text
  srv.request.request_string = "input";
  // Call the service
  client.call(srv);
  // Check the response
  EXPECT_STREQ("input", srv.response.response_string.c_str());
}

