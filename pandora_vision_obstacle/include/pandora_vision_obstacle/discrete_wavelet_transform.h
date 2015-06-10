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
 * Authors:
 *   Chatzieleftheriou Eirini <eirini.ch0@gmail.com>
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#ifndef PANDORA_VISION_OBSTACLE_DISCRETE_WAVELET_TRANSFORM_H
#define PANDORA_VISION_OBSTACLE_DISCRETE_WAVELET_TRANSFORM_H

#include <opencv2/opencv.hpp>

namespace pandora_vision
{
  class DiscreteWaveletTransform
  {
    public:
      /**
       * @brief Constructor used to implement the Ingrid Daubechies
       * Wavelets
       * @param kernelSize [int] The size of the kernel used to
       * perform the transform
       **/
      explicit DiscreteWaveletTransform(int kernelSize);
      /**
       * @brief Constructor used to implement the DWT with a user
       * defined kernel
       * @param kernel [cost cv::Mat&] The kernel used to perform
       * the transform
       **/
      DiscreteWaveletTransform(const cv::Mat& kernelLow,
          const cv::Mat& kernelHigh);

      /**
       * @brief Virtual Destructor
       **/
      virtual ~DiscreteWaveletTransform();

    private:
      /**
       * @brief Perform convolution with a vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param kernel [const cv::Mat&] The filter used
       * @param cols [bool] Whether the type of convolution will
       * be performed column-wise
       **/
      cv::Mat optionalConv(const cv::Mat& inImage,
          const cv::Mat& kernel, bool cols);
      /**
       * @brief Perform convolution with a column vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param kernel [const cv::Mat&] The filter used
       **/
      cv::Mat convCols(const cv::Mat& inImage,
          const cv::Mat& kernel);
      /**
       * @brief Perform convolution with a row vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param kernel [const cv::Mat&] The filter used
       **/
      cv::Mat convRows(const cv::Mat& inImage,
          const cv::Mat& kernel);

      /**
       * @brief Return the final result of the DWT
       * @param inImage [const cv::Mat& inImage]
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<cv::Mat>] The list of images that are
       * the result of the transform with order LL, LH, HL, HH and
       * so on according to the level
       **/
      std::vector<cv::Mat> dwt2D(const cv::Mat& inImage,
          int level = 1);

    private:
      /// The kernel used to perform the DWT that represents the
      /// low - pass filter used
      cv::Mat kernelLow_;
      /// The kernel used to perform the DWT that represents the
      /// band - pass filter used for high frequencies
      cv::Mat kernelHigh_;
  };

}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_DISCRETE_WAVELET_TRANSFORM_H
