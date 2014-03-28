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
* Author: Marios Protopapas
*********************************************************************/
#ifndef PANDORA_VISION_VICTIM_COLOR_DETECTION_H 
#define PANDORA_VISION_VICTIM_COLOR_DETECTION_H 

#include <opencv2/opencv.hpp>
#include <iostream>
#include "ros/ros.h"

namespace pandora_vision
{
  class ColorDetection 
  {
    
    public:
    
    //!Constructor
    ColorDetection();
    
    //!Destructor
    virtual ~ColorDetection();
    
    /**
     * @brief This is the main function which calls all other for the 
     * computation of the color features.
    */ 
    void findColorFeatures();
    
    /**
     * @brief This function returns the histogram of one color component from 
     * the src image.
     * @param planes [cv::Mat] contains the pixel values of a color component.
     * @param bins [int] num of bins where the histogram will be divided.
     * @param histRange [const float*] the range of the histogram.
     * @return [cv::Mat] the calculated histogram.
    */ 
    cv::Mat computeHist(cv::Mat planes, int bins, const float* histRange);
    
    /**
     * @brief This function computes the average and standard deviation value of 
     * every color component(HSV) and returns a feature vector.
     * @param hsv_planes [std::vector<cv::Mat>] contains the pixel values of the
     *  color components.
     * @return [std::vector<double>] the calculated histogram.
    */ 
    std::vector<double> computeMeanStdHSV(std::vector<cv::Mat> hsv_planes);
    
    /**
     * @brief This function computes the dominant Color and it's density value 
     * in a color component.
     * @param img [cv::Mat] the source image
     * @param hist [cv::Mat] the histogram of one color component of the image.
     * @param histSize [int] the size of the histogram
     * @param value [double&] the dominant color value (to be returned).
     * @param density [double&] the dominant color density (to be returned).
     * @return void
    */ 
    void findDominantColor(cv::Mat img, cv::Mat hist, int histSize, double*
                            value, double* density);
    
    /**
     * @brief This function computes the Dft coefficients . 
     * @param img [cv::Mat] the source image
     * @return [std::vector<double>] the feature vector with the 6 first Dft 
     * coefficients.
    */
    std::vector<double> computeDFT(cv::Mat img);
    
    /**
     * @brief This function computes the color angles of the image and the 
     * normalized intensity std . 
     * @param img [cv::Mat] the source image
     * @return [std::vector<double>] the feature vector with the 3 color angles 
     * and normalizes intensity std.
    */
    std::vector<double> computeColorAngles(cv::Mat img);


  };
  
}// namespace pandora_vision
#endif  // PANDORA_VISION_VICTIM_COLOR_DETECTION_H
