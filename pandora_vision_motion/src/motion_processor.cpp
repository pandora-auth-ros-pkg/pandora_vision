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
 *   Despoina Paschalidou
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#include <vector>
#include "pandora_vision_motion/motion_processor.h"

namespace pandora_vision
{
  /**
   * @brief Class Constructor
   * Initializes all variables for thresholding
   */
  MotionProcessor::MotionProcessor(const std::string& ns, sensor_processor::Handler* handler) :
    VisionProcessor(ns, handler)
  {
    ROS_INFO_STREAM("[" + this->getName() + "] processor nh processor : " +
      this->accessProcessorNh()->getNamespace());

    /// The dynamic reconfigure parameter's callback
    server.setCallback(boost::bind(&MotionProcessor::parametersCallback, this, _1, _2));
  }

  MotionProcessor::MotionProcessor(void) : VisionProcessor()
  {
  }

  /**
   * @brief Class Destructor
   * Deallocates memory used for storing images
   */
  MotionProcessor::~MotionProcessor()
  {
    ROS_INFO("Destroying MotionProcessor instance");
  }

  void MotionProcessor::parametersCallback(
    const pandora_vision_motion::motion_cfgConfig& config,
    const uint32_t& level)
  {
    motionDetector_.setDiffThreshold(config.diff_threshold);
    motionDetector_.setHighMotionThreshold(config.high_motion_threshold);
    motionDetector_.setLowMotionThreshold(config.low_motion_threshold);
    motionDetector_.setMaxDeviation(config.max_standard_deviation);
    motionDetector_.setEnableDBSCAN(config.dbscan_enable);

    motionDetector_.visualization = config.visualization;
    motionDetector_.show_image = config.show_image;
    motionDetector_.show_background = config.show_background;
    motionDetector_.show_diff_image = config.show_diff_image;
    motionDetector_.show_moving_objects_contours = config.show_moving_objects_contours;
  }

   /**
    * @brief
    */
  bool MotionProcessor::process(const CVMatStampedConstPtr& input, const POIsStampedPtr& output)
  {
    output->header = input->getHeader();
    motionDetector_.detectMotion(input->getImage());
    boundingBoxes_ = motionDetector_.getMotionPosition();
    output->frameWidth = input->getImage().cols;
    output->frameHeight = input->getImage().rows;
    for (int i = 0; i < boundingBoxes_.size(); i++)
    {
      if (boundingBoxes_[i]->getProbability() > 0.1)
      {
        output->pois.push_back(boundingBoxes_[i]);
        ROS_INFO_STREAM("Motion bbox no:" << i <<
                        "with prob=" << boundingBoxes_[i]->getProbability());
      }
    }
    if (output->pois.size() != 0)
      return true;
    else
      return false;
  }
}  // namespace pandora_vision
