/**
 * @file      talker.h
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
#ifndef INCLUDE_TALKER_H_
#define INCLUDE_TALKER_H_
#include <sstream>
#include <string>
#include "beginner_tutorials/TalkerService.h"

  /**
  * @brief   class Talker is declared with its three private data members textvalue, count and frequency
  *          and six methods : callbackfunction
  *                             getText
  *                             getCount
  *                             setCount
  *                             getFrequency
  *                             setFrequency
  */
class Talker {
 private:
  /**
   * @brief Container for the text to send over the chatter topic
   */
  std::string textvalue;
  int count;
  double frequency;

 public:
  /**
   * @brief Talker constructor
   */
  Talker();

  /**
   * @brief method to handle changing the text to be sent on the chatter topic
   * @param beginner_tutorials::TalkerService::Request &
   * @param beginner_tutorials::TalkerService::Response &
   * @return bool
   */
  bool callbackfunction(beginner_tutorials::TalkerService::Request &,
                   beginner_tutorials::TalkerService::Response &);

  /**
   * @brief Get the current text that is being sent over the chatter topic
   * @param void
   * @return string
   */
  std::string getText();

  /**
   * @brief get the value for the number of times message is published
   * @param void
   * @return int
   */
  int getCount();

  /**
   * @brief set the value for the number of times message is published
   * @param int
   * @return void
   */
  void setCount(int);

  /**
     * @brief get the frequncy of the message to be published
     * @param void
     * @return double
     */
  double getFrequency();
  /**
     * @brief set the frequncy of the message to be published
     * @param double
     * @return void
     */
  void setFrequency(double);
};
#endif  // INCLUDE_TALKER_H_
