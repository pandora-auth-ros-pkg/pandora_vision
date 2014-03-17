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
#ifndef TEXTUREFILTER_H
#define TEXTUREFILTER_H

#include "ros/ros.h"
#include <ros/package.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <iostream>

namespace pandora_vision
{
  class TextureDetector
  {
    std::string packagePath;
    int frameHeight;
    int frameWidth;
    //!< Relative path to walls
    std::string pathToWalls;
        
    //!< Current frame to be processed
    cv::Mat holeFrame;
    //!< Calculated histogramm according to given images
    cv::MatND histogramm;
    //!< Current frame after backprojection
    cv::Mat backprojectedFrame;
    
    public:
    /**
     @brief Class constructor 
    */ 
    TextureDetector();
    
    /**
     @brief Destructor
    */ 
    virtual ~TextrureDetector();
    
    /**
     @brief Get parameters referring to view and frame characteristics
     from launch file 
     @return void
    */ 
    void getGeneralParams();
    
    /**
      @brief Function for calculating histogramms for texture recognition
      @return void
    */
    void  calculateTexture();
  
    /**
      @brief Function for calculating HS histogramm
      @param vector of images corresponding to walls
      @return histogramm [cv::MatND] 
    */
    cv::MatND get_hist(std::vector<cv::Mat> walls);
    
    /**
      @brief Function for calculating applying backprojection in input image
      @param hist [cv::MatND] calculated histogramm from input images
      @param frame [cv::Mat] current frame to be processed
      @return backprojectedframe [cv::Mat] image after backprojection is
      applied
    */
    cv::Mat applyBackprojection(cv::MatND hist,cv::Mat frame);
  } 
}
#endif
