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
 * Authors:
 *   Tsirigotis Christos <tsirif@gmail.com>
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_COMMON_VISION_PREPROCESSOR_H
#define PANDORA_VISION_COMMON_VISION_PREPROCESSOR_H

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include "sensor_processor/abstract_handler.h"
#include "sensor_processor/preprocessor.h"

#include "pandora_vision_common/cv_mat_stamped.h"

namespace pandora_vision
{
  class VisionPreProcessor: public sensor_processor::PreProcessor<sensor_msgs::Image, CVMatStamped>
  {
    protected:
      typedef boost::shared_ptr<sensor_msgs::Image> ImagePtr;
      typedef boost::shared_ptr<sensor_msgs::Image const> ImageConstPtr;

    public:
      VisionPreProcessor(const std::string& ns, sensor_processor::AbstractHandler* handler);
      virtual
        ~VisionPreProcessor() {}

      virtual bool
        preProcess(const ImageConstPtr& input, const CVMatStampedPtr& output);
  };

  VisionPreProcessor::
  VisionPreProcessor(const std::string& ns, sensor_processor::AbstractHandler* handler) :
    PreProcessor<sensor_msgs::Image, cv::Mat>(ns, handler)
  {
  }

  bool
  VisionPreProcessor::
  preProcess(const ImageConstPtr& input, const CVMatStampedPtr& output)
  {
    cv_bridge::CvImagePtr inMsg;
    inMsg = cv_bridge::toCvCopy(*input, sensor_msgs::image_encodings::BGR8);
    output->image = inMsg->image.clone();

    output->header = input->header;

    if (output->image.empty())
    {
      ROS_ERROR("[Node] No more Frames or something went wrong with bag file");
      // ros::shutdown();
      return false;
    }
    return true;
  }
}  // namespace pandora_vision

#endif  // PANDORA_VISION_COMMON_VISION_PREPROCESSOR_H
