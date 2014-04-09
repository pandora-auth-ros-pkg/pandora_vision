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
#ifndef EDGEDETECTION_H
#define EDGEDETECTION_H

#include "ros/ros.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <iostream>

namespace pandora_vision
{
  class EdgeDetector
  {
    
    public:
    //!< Constructor
    EdgeDetector();
    
    //!<Destruuctor
    virtual ~EdgeDetector();
    
    /**
      @brief Applies the Canny edge transform
      @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
      @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
      @return void
    **/ 
    void applyCanny(const cv::Mat inImage, cv::Mat* outImage);
    
    /**
      @brief Applies the Sobel edge transform
      @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
      @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
      @return void
    **/
    void applySobel (const cv::Mat inImage, cv::Mat* outImage);
    
    /**
      @brief Applies the Scharr edge transform
      @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
      @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
      @return void
    **/
    void applyScharr (const cv::Mat inImage, cv::Mat* outImage);
    
    /**
      @brief Applies the Laplacian edge transform
      @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
      @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
      @return void
   **/
    void applyLaplacian (const cv::Mat inImage, cv::Mat* outImage);
  };
}
#endif
