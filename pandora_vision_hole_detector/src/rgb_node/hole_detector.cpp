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
* Author: Despoina Paschalidou
*********************************************************************/
#include "rgb_node/hole_detector.h"

namespace pandora_vision
{
  /**
   @brief Class constructor
  */ 
  HoleDetector::HoleDetector()
  {
    backprojectedFrame = cv::Mat::zeros(RgbParameters::frameHeight, 
        RgbParameters::frameWidth, CV_8UC1);
    
    ROS_INFO("[rgb_node]: HoleDetector instance created");
  }
  
  /**
   @brief Class destructor
  */
  HoleDetector::~HoleDetector()
  {
    ROS_INFO("[rgb_node]: HoleDetector instance destroyed");
  } 
  
  /**
   @brief Function that locates the position of potentional holes
   in current frame.
   @param holeFrame [cv::Mat] current frame to be processed
   @return void
  */ 
  void HoleDetector::findHoles(cv::Mat holeFrame)
  {
    //! Find pixels in current frame where there is the same texture
    //! according to the given histogramm and calculate
    //! backprojection of current frame
    _textureDetector.applyTexture(holeFrame, &backprojectedFrame);
    
    //! Apply in current frame Canny edge detection algorithm
    _edgeDetector.applySobel(backprojectedFrame, &edgesFrame);
   
    debug_show(holeFrame, backprojectedFrame, edgesFrame);
    //~ _blobDetector.detectBlobs(edgesFrame, &detectedkeyPoints);
  }
  
  /**
    @brief Function for debbuging reasons,shows histogramm and 
    current frame after backprojection is applied
    @param holeFrame [cv::Mat] the currrent frame to be processed
    @param backprojectedFrame [cv::Mat] current frame after backprojection,
    this parameter is returned
    @param edgesFrame [cv::Mat] current frame after edge detection algorithm
    is applied 
    @return void
  */
  void HoleDetector::debug_show(cv::Mat holeFrame, 
      cv::Mat backprojectedFrame, cv::Mat edgesFrame)
  {
    ros::Time timeBegin = ros::Time::now();
    while( ros::Time::now()-timeBegin < ros::Duration(1))
    {
       cv::imshow(" Current frame", holeFrame);
       cv::imshow(" Frame after backprojection ", backprojectedFrame);
       cv::imshow(" Frame after edge detection ", edgesFrame);
       cv::waitKey(10);
    }
  }
}// namespace pandora_vision
