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
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_detector.h"

namespace pandora_vision
{
  HardObstacleDetector::HardObstacleDetector()
  {
  }

  HardObstacleDetector::HardObstacleDetector(const std::string& name,
      const ros::NodeHandle& nh)
  {
    nodeName = name;
  }

  cv::Mat HardObstacleDetector::startDetection(const cv::Mat& inputImage)
  {
    if (show_input_image)
    {
      // Show the input image
      showImage("The input image", inputImage, 1);
    }

    //detectEdges(inputImage, );

  }

  void HardObstacleDetector::showImage(
    const std::string& title, const cv::Mat& image, int time)
  {
    // Copy the input image to another
    cv::Mat imageCpy;
    image.copyTo(imageCpy);

    if (imageCpy.depth() == CV_8SC1)
    {
      image.convertTo(imageCpy, CV_8UC1);

      // If value is negative (-1), make it 255 for visualization
      for (unsigned int rows = 0; rows < image.rows; rows++)
      {
        for (unsigned int cols = 0; cols < image.cols; cols++)
        {
          if (image.at<signed char>(rows, cols) < 0)
          {
            imageCpy.at<unsigned char>(rows, cols) = 255;
          }
        }
      }
    }
    cv::imshow(title, imageCpy);
    cv::waitKey(time);
  }

  void HardObstacleDetector::applyCanny(
    const cv::Mat& inImage, cv::Mat* outImage)
  {

  }

  void HardObstacleDetector::applyScharr(
    const cv::Mat& inImage, cv::Mat* outImage)
  {

  }

  void HardObstacleDetector::applySobel(
    const cv::Mat& inImage, cv::Mat* outImage)
  {

  }

  void HardObstacleDetector::detectEdges(
    const cv::Mat& inImage, cv::Mat* outImage)
  {
    // Copy the input image to another
    cv::Mat imageCpy;
    inImage.copyTo(imageCpy);

    // Since the input image has CV_8S type, change it to CV_8UC1 in order to
    // further process it.
    // The input image had values in range [-1, 100] Occupancy grid map
    // After conversion its range will be [0, 100]
    imageCpy.convertTo(imageCpy, CV_8UC1);

    switch (edge_method)
    {
      case 0 :
        applyCanny(imageCpy, outImage);
        break;
      case 1 :
        applyScharr(imageCpy, outImage);
        break;
      case 2 :
        applySobel(imageCpy, outImage);
        break;
    }

    if (show_edges_image)
    {
      showImage("The edges image", *outImage, 1);
    }

  }

}  // namespace pandora_vision
