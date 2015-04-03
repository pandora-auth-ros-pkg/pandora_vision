/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#include "pandora_vision_qrcode/qrcode_handler.h"

namespace pandora_vision
{
  QrCodeHandler::QrCodeHandler(const std::string& ns) : sensor_processor::Handler<sensor_msgs::Image, CVMatStamped, 
    POIsStamped, pandora_vision_msgs::QRAlertsVectorMsg>(ns)
  {
    boost::shared_ptr<QrCodePostProcessor> postProcPtr(
        boost::dynamic_pointer_cast<QrCodePostProcessor>(postProcPtr_));
    boost::shared_ptr<QrCodeDetector> processorPtr(
        boost::dynamic_pointer_cast<QrCodeDetector>(processorPtr_));
    postProcPtr->setDebugFrame(processorPtr->get_debug_frame());
  }
  
  void QrCodeHandler::startTransition(int newState)
  {
    currentState_ = newState;
    
    bool previouslyOff = (previousState_ == state_manager_msgs::RobotModeMsg::MODE_OFF);
    bool currentlyOn = (currentState_ != state_manager_msgs::RobotModeMsg::MODE_OFF);
    
    if (previouslyOff && currentlyOn)
    {
      preProcPtr_.reset(new QrCodePreProcessor("~/preprocessor", this));
      processorPtr_.reset(new QrCodeDetector("~/processor", this));
      postProcPtr_.reset(new QrCodePostProcessor("~/postprocessor", this));
    }
    else if (!previouslyOff && !currentlyOn)
    {
      preProcPtr_.reset();
      processorPtr_.reset();
      postProcPtr_.reset();
    }
    
    if (currentState_ == state_manager_msgs::RobotModeMsg::MODE_TERMINATING)
    {
      preProcPtr_.reset();
      processorPtr_.reset();
      postProcPtr_.reset();
      
      ros::shutdown();
      return;
    }

    previousState_ = currentState_;
    transitionComplete(currentState_);
  }
  
  void QrCodeHandler::completeTransition()
  {
  }
}  // namespace pandora_vision
