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
#include "rgb_node/texture_filter.h"

namespace pandora_vision
{
  
  /**
     @brief Class constructor 
  */ 
  TextureDetector::TextureDetector()
  {
    getGeneralParams();
    
    pathToWalls = packagePath+ "/walls/";
    calculateTexture();
    ROS_INFO("[rgb_node]: Textrure detector instance created");
  }
    
  /**
    @brief Destructor
  */ 
  TextureDetector::~TextureDetector()
  {
    ROS_INFO("[rgb_node]: Textrure detector instance deleted");
  }
  
  /**
    @brief Get parameters referring to view and frame characteristics
    from launch file 
    @return void
  */ 
  void TextureDetector::getGeneralParams()
  {
    packagePath = ros::package::getPath("pandora_vision_hole_detector");
    
    //!< Get the Height parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_height"))
    {
      _nh.getParam("/" + cameraName + "/image_height", frameHeight);
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    else
    {
      frameHeight = RgbParameters::frameHeight;
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }

    //!< Get the Width parameter if available;
    if (_nh.hasParam("/" + cameraName + "/image_width"))
    {
      _nh.getParam("/" + cameraName + "/image_width", frameWidth);
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    else
    {
      frameWidth = RgbParameters::frameWidth;
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
  }
    
  /**
    @brief Function for calculating histogramms for texture recognition
    @return void
  */
  void  TextureDetector::calculateTexture()
  {
    std::vector<cv::Mat> walls;
    for(int i = 0; i < 6; i++)
    {
      char temp_name[250];
      std::string temp;
      temp = pathToWalls+"%d.png";
      sprintf(temp_name, temp.c_str(), i);
      walls[i] = cv::imread(temp_name);
    }
    histogramm = get_hist(walls);
  }
  
  /**
    @brief Function for calculating HS histogramm
    @param walls [vector<cv::Mat>] vector of images corresponding to walls
    @return histogramm [cv::MatND] 
  */
  cv::MatND TextureDetector::get_hist(std::vector<cv::Mat> walls)
  {
    cv::Mat* hsv = new cv::Mat[14];
    for(int i = 0; i < 6; i++)
      cvtColor(walls[i], hsv[i], CV_BGR2HSV);
  
    /// Quantize the hue to 30 levels
    /// and the saturation to 32 levels
    int hbins = 30, sbins = 32;
    int histSize[] = {hbins, sbins};
    /// hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    ///saturation varies from 0 (black-gray-white) to
    /// 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    const float* ranges[] = { hranges, sranges };
    cv::MatND hist;
    /// We compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};
    cv::calcHist(hsv, 6, channels, cv::Mat(), hist, 2, 
          histSize, ranges, true, false );
    return hist;
  }
  
  /**
    @brief Function for calculating applying backprojection in input image
    @param hist [cv::MatND] calculated histogramm from input images
    @param frame [cv::Mat] current frame to be processed
    @return backprojectedframe [cv::Mat] image after backprojection is
    applied
  */
  cv::Mat TextureDetector::applyBackprojection(cv::MatND hist,
        cv::Mat holeFrame)
  {
    cv::Mat hsv;
    cvtColor(holeFrame, hsv, CV_BGR2HSV);
    /// Get Backprojection 
    cv::Mat backproj = cv::Mat::zeros(frameHeight, frameWidth, CV_8UC1);
    /// hue varies from 0 to 180
    float hranges[] = { 0, 180 };
    /// saturation varies from 0 to 80
    float sranges[] = { 0, 120 };
   
    const float* ranges[] = { hranges, sranges};
    int channels[] = {0, 1};
    cv::calcBackProject( &hsv, 1, channels , hist, backproj, ranges, 1, true );
    int const max_BINARY_value = 255;
    /// apply backprojected image
    cv::threshold(backproj, backproj, 45, max_BINARY_value, 0);
    cv::Mat kernel = 
          getStructuringElement(cv::MORPH_CROSS , cv::Size(3, 3), 
          cv::Point( -1, -1 ));
    cv::morphologyEx(backproj, backproj, cv::MORPH_CLOSE, 
        kernel, cv::Point(-1, -1), 15);

    return backproj;
  }
  
  /**
   @brief Function that applies backprogected image in current frame
   in order to find out which part of it belong to the given texture
   @return void
  */ 
  void TextureDetector::applyTexture()
  {
    cv::Mat backprojection = cv::Mat::zeros(frameHeight, frameWidth, CV_8UC1);
    backprojection = applyBackprojection(histogramm, holeFrame).clone();
    bitwise_and( holeFrame, backprojection, backprojectedFrame);
  }
}// namespace pandora_vision
