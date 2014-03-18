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
#include "rgb_node/edge_detection.h"

namespace pandora_vision
{
  //!< Constructor
  EdgeDetector::EdgeDetector()
  {
    ROS_INFO("[rgb_node]: Edge detector instance created");
  }
  
  EdgeDetector::~EdgeDetector()
  {
    ROS_INFO("[rgb_node]: Edge detector instance destroyed");
  }  
  
  /**
    @brief Applies the Sobel edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
  **/
  void EdgeDetector::applySobel (const cv::Mat inImage, cv::Mat* outImage)
  {
    //!< appropriate values for scale, delta and ddepth
    int scale = RgbParameters::sobel_scale;
    int delta = RgbParameters::sobel_delta;
    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);
    
    //!< Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    //!< Gradient X
    cv::Sobel(edges, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    //!< Gradient Y
    cv::Sobel(edges, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    //!< Total Gradient 
    cv::Mat grad_g;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad_g);

    *outImage = grad_g;
  }
  
 /**
    @brief Applies the Canny edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
  **/
  void EdgeDetector::applyCanny(const cv::Mat inImage, cv::Mat* outImage)
  {
    cv::Mat detected_edges;
    cv::Mat dst;
    int ratio = RgbParameters::canny_ratio;
    int kernel_size = RgbParameters::canny_kernel_size;
    int lowThreshold = RgbParameters::canny_low_threshold;
    
    cv::Mat edges;
    inImage.copyTo(edges);
    edges.copyTo(*outImage);
    
    //!< Reduce noise with a kernel 3x3
    cv::blur(*outImage, detected_edges, cv::Size(
         RgbParameters::canny_blur_noise_kernel_size,
         RgbParameters::canny_blur_noise_kernel_size));

    //!< Canny detector
    cv::Canny(detected_edges, detected_edges, lowThreshold,
        lowThreshold * ratio, kernel_size);

    //!< Using Canny's output as a mask, we display our result
    dst = cv::Scalar::all(0);

    outImage->copyTo(dst, detected_edges);
    *outImage = dst; 
  }
  
  /**
    @brief Applies the Scharr edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetector::applyScharr(const cv::Mat inImage, cv::Mat* outImage)
  {
    //!< appropriate values for scale, delta and ddepth
    int scale = 1;
    int delta = 0; //!< the value for the non-edges
    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);
    //!< Convert image to grayscale
    cvtColor( edges, edges, CV_RGB2GRAY );
    
    //!< Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    //!< Gradient X
    cv::Scharr(edges, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    //!< Gradient Y
    cv::Scharr(edges, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    //!< Total Gradient 
    cv::Mat grad_g;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad_g);

    *outImage = grad_g;

  }
  
  /**
    @brief Applies the Laplacian edge transform
    @param[in] inImage [const cv::Mat&] Input image in CV_8UC1 format
    @param[out] outImage [cv::Mat*] The processed image in CV_8UC1 format
    @return void
   **/
  void EdgeDetector::applyLaplacian(const cv::Mat inImage, cv::Mat* outImage)
  {
    //!< appropriate values for scale, delta and ddepth
    int scale = 1;
    int delta = 0; //!< the value for the non-edges
    int ddepth = CV_16S;

    cv::Mat edges;
    inImage.copyTo(edges);
    //!< Convert image to grayscale
    cvtColor( edges, edges, CV_RGB2GRAY );
    cv::Laplacian(edges, *outImage, ddepth, 1, scale, delta, cv::BORDER_DEFAULT);
    convertScaleAbs(*outImage, *outImage);
  }
}// namespace pandora_vision
