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

#include "pandora_vision_victim/victim_vj_processor.h"

namespace pandora_vision
{
  VictimVJProcessor::VictimVJProcessor(const std::string& ns, 
    sensor_processor::Handler* handler) : VisionProcessor(ns, handler)
  {
    params_.configVictim(*this->accessPublicNh());

    /// Initialize the face detector classifiers
    _rgbViolaJonesDetector = VictimVJDetector(params_.cascade_path,
      params_.model_path);
    
    ROS_INFO("[victim_node] : Created Victim VJ Processor instance");
  }
  
  VictimVJProcessor::VictimVJProcessor() : VisionProcessor() {}
  
  VictimVJProcessor::~VictimVJProcessor()
  {
    ROS_DEBUG("[victim_node] : Destroying Victim VJ Processor instance");
  }
  
  
  // .........
  
  
  
  /**
   * @brief
   **/
  bool VictimVJProcessor::process(const ImagesStampedConstPtr& input, 
    const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    output->frameWidth = input->getRgbImage().cols;
    output->frameHeight = input->getRgbImage().rows;
    
    std::vector<VictimPOIPtr> victims = detectVictims(input);
    
    for (int ii = 0; ii < victims.size(); ii++)
    {
      if (victims[ii]->getProbability() > 0.0001)
      {
        output->pois[ii] = victims[ii];
      }
    }
    
    if (output->pois.empty())
    {
      return false;
    }
    return true;
  }
}  // namespace pandora_vision
