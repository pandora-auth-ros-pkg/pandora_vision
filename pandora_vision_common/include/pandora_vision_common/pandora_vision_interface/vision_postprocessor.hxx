/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors:
* Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
*********************************************************************/

#include <opencv2/opencv.hpp>
#include "pandora_common_msgs/GeneralAlertMsg.h"
#include "sensor_processor/vision_postprocessor.h"

namespace sensor_processor
{
  template <class ProcOutput, class PubType>
  VisionPostProcessor<ProcOutput, PubType>::VisionPostProcessor(const NodeHandlePtr& nhPtr): PostProcessor(nhPtr)
  {
    // .......
  }
  
  template <class ProcOutput, class PubType>
  VisionPostProcessor<ProcOutput, PubType>::~VisionPostProcessor()
  {
  }
  
  template <class ProcOutput, class PubType>
  template<class Type> void VisionPostProcessor<ProcOutput, PubType>::getParameter(const std::string& name, 
    const Type& param)
  {
    if (nhPtr_->getParam(name, param))
    {
      ROS_DEBUG_STREAM(name << " : " << param);
    }
    else
    {
      ROS_FATAL("[Node] : Parameter " << name << " not found. Using Default");
      ROS_BREAK();
    }
  }
  
  template <class ProcOutput, class PubType>
  bool VisionPostProcessor<ProcOutput, PubType>::getParentFrameId()
  {
    const std::string modelParamName = "/robot_description";
    bool res = nhPtr_->hasParam(modelParamName);
    std::string robotDescription;
    
    if (!res || !nhPtr_->getParam(modelParamName, robotDescription))
    {
      ROS_ERROR("Robot description couldn't be retrieved from the parameter server.");
      return false;
    }
    
    boost::shared_ptr<urdf::ModelInterface> model(urdf::parseURDF(robotDescription));
    boost::shared_ptr<const urdf::Link> currentLink = model->getLink(frameId_);
    
    if (currentLink)
    {
      boost::shared_ptr<const urdf::Link> parentLink = currentLink->getParent();
      parentFrameIdMap_[frameId_] = parentLink->name;
      return true;
    }
    else
    {
      parentFrameIdMap_[frameId_] = frameId_;
    }
    return false;
  }
  
  template <class ProcOutput, class PubType>
  void VisionPostProcessor<ProcOutput, PubType>::getAllParameters()
  {
    bool result = getParentFrameId(frameId_);
    getParameter<double>("hfov", hfovMap_[frameId_]);
    getParameter<double>("vfov", vfovMap_[frameId_]);
  }
  
  template <class ProcOutput, class PubType>
  void VisionPostProcessor<ProcOutput, PubType>::setFrameInfo(const ImagePtr& frame)
  {
    frameId_ = frame->header.frame_id;
    frameHeight_ = frame->height;
    frameWidth_ = frame->width;
    
    //~ if (frameId_[0] == '/')  // ??????????
    //~ {
      //~ frameId_ = frameId_.substr(1);
    //~ }
    
    if (parentFrameIdMap_.find(frameId_) == parentFrameIdMap_.end())
    {
      getAllParameters(frameId_);
    }
  }
  
  template <class ProcOutput, class PubType>
  void VisionPostProcessor<ProcOutput, PubType>::findAnglesOfRotation()
  {
    for (int ii = 0; ii < imagePoints_.size(); ii++)
    {
      float x = imagePoints_[ii].x - static_cast<float>(frameWidth_) / 2;
      float y = static_cast<float>(frameHeight_) / 2 - imagePoints_[ii].y;
      
      anglesOfRotation_[ii].yaw = atan(2 * x / frameWidth_ * tan(hfovMap_[frameId_] / 2));
      anglesOfRotation_[ii].pitch = atan(2 * y / frameHeight_ * tan(vfovMap_[frameId_] / 2));
    }
  }
}  // namespace sensor_processor
