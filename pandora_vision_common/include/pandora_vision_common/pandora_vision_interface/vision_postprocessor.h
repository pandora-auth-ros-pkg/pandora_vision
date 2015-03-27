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

#ifndef SENSOR_PROCESSOR_VISION_POSTPROCESSOR_H
#define SENSOR_PROCESSOR_VISION_POSTPROCESSOR_H

#include <urdf_parser/urdf_parser.h>
#include <sensor_msgs/Image.h>  // ................
#include "pandora_common_msgs/GeneralAlertMsg.h"
#include "sensor_processor/postprocessor.h"

namespace sensor_processor
{
  template <class ProcOutput, class PubType>
  class VisionPostProcessor: public PostProcessor<ProcOutput, PubType>
  {
    public:
      typedef boost::shared_ptr<sensor_msgs::Image> ImagePtr; 
      
      explicit VisionPostProcessor(const NodeHandlePtr& nhPtr);
      virtual ~VisionPostProcessor();
      
      virtual void process();
      
    protected:
      int frameWidth_;
      int frameHeight_;
      std::string frameId_;
      
      std::map<std::string, std::string> parentFrameIdMap_;
      std::map<std::string, double> hfovMap_;
      std::map<std::string, double> vfovMap_;
      
      std::vector<cv::Point> imagePoints_;
      std::vector<pandora_common_msgs::GeneralAlertMsg> anglesOfRotation_;
      
      void setFrameInfo(const ImagePtr& frame);  //
      void findAnglesOfRotation();
      bool getParentFrameId();
      void getAllParameters();
      template<class Type> void getParameter(const std::string& name, const Type& param);
  };
}  // namespace sensor_processor

#include "sensor_processor/vision_postprocessor.hxx"

#endif  // SENSOR_PROCESSOR_VISION_POSTPROCESSOR_H
