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

#ifndef PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_DETECTOR_H
#define PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_DETECTOR_H

#include <string>
#include <vector>
#include "cv_bridge/cv_bridge.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>

namespace pandora_vision
{
  class HardObstacleDetector
  {
    public:
      HardObstacleDetector();

      HardObstacleDetector(const std::string& name,
          const ros::NodeHandle& nh);

      virtual ~HardObstacleDetector()
      {
      }

      /**
        @brief Start of the hard obstacle detection process.
        @param[in] inputImage [const cv::Mat&] The input image from preprocessor
        @return cv::Mat
       **/
      cv::Mat startDetection(const cv::Mat& inputImage);

    private:
      std::string nodeName;

      // The robots mask
      cv::Mat robotMask_;

      // Configuration parameters
      int edge_method;
      int edges_threshold;
      bool show_input_image;
      bool show_edges_image;
      bool show_edges_thresholded_image;
      bool show_edges_and_unkown_image;
      bool show_new_map_image;

      /**
        @brief Visualization of an image with CV_32FC1 or CV_8UC1 type.
        If there is a negative value in mat aka unkown area set it 255 for
        visualization.
        @param[in] title [const std::string&] The title of image to be shown.
        @param[in] image [const cv::Mat&] The image to be shown.
        @param[in] time [int] The time that imshow function lasts in ms.
        @return void
       **/
      void showImage(
        const std::string& title, const cv::Mat& image, int time);

      /**
        @brief Converts an image of CV_32FC1 type to CV_8UC1. Negative values
        will be replaced with zero values.
        @param[in] inImage [const cv::Mat&] The input image.
        @return cv::Mat
       **/
      cv::Mat scaleFloatImageToInt(const cv::Mat& inImage);

      /**
        @brief Checks if there is any negative value in the input mat, if yes
        fill the peer pixel in outImage with value -1.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[in] outImage [cv::Mat*] The output image with unkown areas.
        @return void
       **/
      void fillUnkownAreas(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Now that we have a complete map, aka dangerous, safe and unkown
        areas we will include the robot's size. A convolution between the map
        and robot's mask will make the new map that informs us if we can access 
        a particular point(territory).
        @param[in] inImage [const cv::Mat&] The input image.
        @param[in] outImage [cv::Mat*] The output edges image will unkown areas.
        @return void
       **/
      void robotMaskOnMap(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Apply the canny edge detection algorithm.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[in] outImage [cv::Mat*] The output edges image
        @return void
       **/
      void applyCanny(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Apply the sharr edge detection algorithm.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[in] outImage [cv::Mat*] The output edges image
        @return void
       **/
      void applyScharr(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Apply the sobel edge detection algorithm.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[in] outImage [cv::Mat*] The output edges image
        @return void
       **/
      void applySobel(const cv::Mat& inImage, cv::Mat* outImage);

      /**
        @brief Apply edge detection algorithm. Based on configuration parameter,
        select the desired method. Finally apply threshold on the extracted
        edges to keep the ones that we consider as dangerous areas.
        @param[in] inImage [const cv::Mat&] The input image.
        @param[in] outImage [cv::Mat*] The output edges image
        @return void
       **/
      void detectEdges(const cv::Mat& inImage, cv::Mat* outImage);
  };

}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_HARD_OBSTACLE_DETECTION_HARD_OBSTACLE_DETECTOR_H
