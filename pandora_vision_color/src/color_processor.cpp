/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
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
* Author:  Marios Protopapas, <marios_protopapas@hotmail.com>
*********************************************************************/

#include <vector>
#include "pandora_vision_color/color_processor.h"

namespace pandora_vision
{
  /**
    @brief Class Constructor
    Initializes all variables for thresholding
  */
  ColorProcessor::ColorProcessor(const std::string& ns, sensor_processor::Handler* handler) :
    VisionProcessor(ns, handler)
  {
    ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
      this->accessProcessorNh()->getNamespace());

    //!< The dynamic reconfigure parameter's callback
    server.setCallback(boost::bind(&ColorProcessor::parametersCallback, this, _1, _2));
  }

  ColorProcessor::ColorProcessor(void) : VisionProcessor()
  {
  }

  /**
    @brief Class Destructor
    Deallocates memory used for storing images
  */
  ColorProcessor::~ColorProcessor()
  {
    ROS_INFO("Destroying ColorProcessor instance");
  }
  
  void ColorProcessor::parametersCallback(
    const pandora_vision_color::color_cfgConfig& config,
    const uint32_t& level)
  {
  }

   /**
   * @brief
   **/
  bool ColorProcessor::process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    colorDetector.detectColor(input->getImage());
    bounding_box_ = colorDetector.getColorPosition();
    output->frameWidth = input->getImage().cols;
    output->frameHeight = input->getImage().rows;
    if (bounding_box_->getProbability() > 0.1)
    {
      output->pois.push_back(bounding_box_);
      return true;
    }
    return false;
  }
}  // namespace pandora_vision
