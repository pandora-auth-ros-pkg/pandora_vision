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
* Author:  Despoina Pascahlidou
*********************************************************************/

#ifndef PANDORA_VISION_MOTION_MOTION_DETECTOR_H
#define PANDORA_VISION_MOTION_MOTION_DETECTOR_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "pandora_vision_motion/motion_parameters.h"

namespace pandora_vision 
{
  class MotionDetector
  {
    public:
      /**
        @brief Class Constructor
        Initializes all varialbes for thresholding
      */
      MotionDetector();
      
      /**
        @brief Class Destructor
      */
      ~MotionDetector();
      
      /**
        @brief Function that detects motion, according to substraction
        between background image and current frame. According to predifined 
        thresholds motion is detected. According to the type of motion
        the suitable value is returned.
        @param frame [&cv::Mat] current frame to be processed 
        @return [int] Index of evaluation of Motion in current frame.
      */
      int detectMotion(cv::Mat &frame);
      
      /**
        @brief Creates the continuous table of motion in current frame
        @return [std::array<int,4>] table of motion position and size
      */
      int* getMotionPosition();
    
     protected:
      /**
        @brief Function that defines the type of movement 
        according to the number of pixels, that differ from current
        frame and background. In case insignificant motion 0 is detected
        0 is returned. If there is slight motion 1 is returned and last
        bust not least in case extensive motion is detected 2 is returned
        @param thresholdedDifference: [&cv::Mat] frame that represents
          the thresholded difference between current frame and computed 
          background
        @return typeOfMovement [int], where 2 corresponds to moving objects
        with greater probability whereas 0 corresponds to stationary objects
      */ 
      int motionIdentification(const cv::Mat &thresholdedDifference);
      
      /**
        @brief Function used for debug reasons, that shows background
        foreground and contours of motion trajectories in current frame
        @param thresholdedDifference: [&cv::Mat] frame that represents
          the thresholded difference between current frame and computed 
          background.
        @param frame: [&cv::Mat] current frame, captured from camera
        @return void
      */ 
      void debugShow(
        const cv::Mat &thresholdedDifference, 
        const cv::Mat &frame
      );
      
      /**
        @brief Function that calculates motion's postion
        @param thresholdedDifference: [&cv::Mat] frame that represents
        the thresholded difference between current frame and computed 
        background.
        @return void 
      */
      void detectMotionPosition(cv::Mat diff); 
        
    private:
      //!< Current frame to be processed
      cv::Mat frame_;
      //!< Background image
      cv::Mat background_;
      //!< Foreground mask
      cv::Mat foreground_;
      cv::Mat movingObjects_;
      //!< Class instance for Gaussian Mixture-based backgound 
      //!< and foreground segmentation 
      cv::BackgroundSubtractorMOG2 bg_;
      ///!< Erode kernel
      cv::Mat kernel_erode_;
      //!< Maximum deviation for calculation position of moving objects;
      int max_deviation_;
      //!< Bounding box of moving objects.    
      cv::Rect_<int> bounding_box_;
      
      friend class MotionDetectorTest;
  };
}// namespace pandora_vision
#endif  // PANDORA_VISION_MOTION_MOTION_DETECTOR_H
