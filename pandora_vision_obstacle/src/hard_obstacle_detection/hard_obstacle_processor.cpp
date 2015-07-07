/*********************************************************************
 *
 * Hardware License Agreement (BSD License)
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
 *     from this hardware without specific prior written permission.
 *
 *  THIS HARDWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS HARDWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *  Choutas Vassilis <vasilis4ch@gmail.com>
 *  Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#include <string>
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_processor.h"

namespace pandora_vision
{
  HardObstacleProcessor::HardObstacleProcessor(const std::string& ns,
      sensor_processor::Handler* handler) :
    sensor_processor::Processor<CVMatStamped, CVMatStamped>(ns, handler)
  {
    ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
    this->accessProcessorNh()->getNamespace());

    detector_.reset(new HardObstacleDetector(this->getName(),
          *this->accessPublicNh()));

    server.setCallback(boost::bind(&HardObstacleProcessor::parametersCallback,
        this, _1, _2));
  }

  HardObstacleProcessor::HardObstacleProcessor() :
    sensor_processor::Processor<CVMatStamped, CVMatStamped>()
  {
  }

  void HardObstacleProcessor::parametersCallback(
    const pandora_vision_obstacle::hard_obstacle_cfgConfig& config,
    const uint32_t& level)
  {
    // Debug show parameters
    detector_->setShowInputImage(config.show_input_image);
    detector_->setShowEdgesImage(config.show_edges_image);
    detector_->setShowEdgesThresholdedImage(config.show_edges_thresholded_image);
    detector_->setShowEdgesUnkownImage(config.show_edges_and_unkown_image);
    detector_->setShowNewMapImage(config.show_new_map_image);

    // Edge detection parameters
    detector_->setEdgeMethod(config.edge_detection_method);

    // Canny parameters
    detector_->setCannyKernelSize(config.canny_kernel_size);
    detector_->setCannyLowThreshold(config.canny_low_threshold);
    detector_->setCannyBlurNoiseKernelSize(config.canny_blur_noise_kernel_size);

    detector_->setEdgesThreshold(config.edges_threshold);
  }

  bool HardObstacleProcessor::process(const CVMatStampedConstPtr& input,
      const CVMatStampedPtr& output)
  {
    output->header = input->getHeader();
    output->image = detector_->startDetection(input->getImage());

    return true;
  }

}  // namespace pandora_vision
