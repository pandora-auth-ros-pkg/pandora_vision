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
 * Authors:
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_SOFT_OBSTACLE_DETECTION_SOFT_OBSTACLE_DETECTOR_H
#define PANDORA_VISION_OBSTACLE_SOFT_OBSTACLE_DETECTION_SOFT_OBSTACLE_DETECTOR_H

#include <cv_bridge/cv_bridge.h>
#include "pandora_vision_obstacle/discrete_wavelet_transform.h"
#include "pandora_vision_obstacle/obstacle_poi.h"

namespace pandora_vision
{
  class SoftObstacleDetector
  {
    public:
      /**
       * @brief Constructor that initializes dwt Haar kernels
       **/
      SoftObstacleDetector();

      /**
       * @brief Destructor
       **/
      ~SoftObstacleDetector() {}

    public:
      /**
       * @brief Detect soft obstacle by performing a number of
       * operations to the rgb and depth image
       * @param rgbImage [const cv::Mat&] The input rgb image
       * @param depthImage [const cv::Mat&] The input depth image
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<POIPtr>] The Point Of Interest that
       * includes the soft obstacle
       **/
      std::vector<POIPtr> detectSoftObstacle(const cv::Mat& rgbImage,
          const cv::Mat& depthImage, int level = 1);

    private:
      /**
       * @brief Invert image if necessary and dilate it
       * @param image [const MatPtr&] The image to be dilated
       **/
      void dilateImage(const MatPtr& image);

      /**
       * @brief Perform Probabilistic Hough Lines Transform and
       * keep only vertical lines
       * @param image [const cv::Mat&] The image that the transform
       * is applied to
       * @return [std::vector<cv::Vec4i>] The vector containing each
       * vertical line's start and end point
       **/
      std::vector<cv::Vec4i> performProbHoughLines(const cv::Mat& image);

      /**
       * @brief Create the bounding box that includes the soft obstacle
       * @param verticalLines [const std::vector<cv::Vec4i>&] The
       * input vector that contains the vertical lines found
       * @param frameHeight [int] The image height
       * @param roiPtr [const boost::shared_ptr<cv::Rect>&] The
       * bounding box that is returned
       * @return [float] The probability of the soft obstacle
       * considering the line's length
       **/
      float detectROI(const std::vector<cv::Vec4i>& verticalLines,
          int frameHeight, const boost::shared_ptr<cv::Rect>& roiPtr);

    private:
      /// The DWT class object used to perform this operation
      boost::shared_ptr<DiscreteWaveletTransform> dwtPtr_;
  };
}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_SOFT_OBSTACLE_DETECTION_SOFT_OBSTACLE_DETECTOR_H
