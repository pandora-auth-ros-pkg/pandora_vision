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
* Author:  Marios Protopapas, <protopapas_marios@hotmail.com>
*********************************************************************/

#include <vector>
#include "pandora_vision_color/color_detector.h"

namespace pandora_vision
{
  /**
    @brief Class Constructor
    Initializes all varialbes for thresholding
  */

  ColorDetector::ColorDetector(void)
  {

  }

  
  /**
    @brief Class Destructor
    Deallocates memory used for storing images
  */
  ColorDetector::~ColorDetector()
  {
    ROS_INFO("Destroying ColorDetector instance");
  }
  
  BBoxPOIPtr ColorDetector::getColorPosition(void)
  {
    return bounding_box_;
  }

   /**
    @brief Function that detects color,
    @param frame [&cv::Mat] current frame to be processed
    @return void.
  */
  void ColorDetector::detectColor(const cv::Mat& frame)
  {
    frame_ = frame.clone();
    /// Check that frame has data and that image has 3 channels
    if (frame_.data && frame_.channels() == 3)
    {
      /// blur the image using GaussianBlur
      GaussianBlur( frame_, frame_, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );

      //convert RGB image into HSV image
      cvtColor(frame, hsvFrame_, CV_BGR2HSV);

      //get binary image
      /*inRange(hsv, Scalar(157, 72, 156), Scalar(180, 169, 255), binary);//pink*/
      inRange(hsvFrame_, cv::Scalar(110,50,50), cv::Scalar(130,255,255), binary_);//blue

    }
  }

  /**
    @brief Function that calculates motion's position
    @param diff: [&cv::Mat] frame that represents
      the thresholded difference between current frame and computed
      background.
    @return void
  */
  void ColorDetector::detectColorPosition(const cv::Mat& diff)
  {
      
  }

  /**
    @brief Function used for debug reasons
    @return void
  */
  void ColorDetector::debugShow( )
  {
    
  }
}  // namespace pandora_vision
