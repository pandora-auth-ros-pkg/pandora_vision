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

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

namespace pandora_vision
{
  class DiscreteWaveletTransform
  {
    public:
      typedef boost::shared_ptr<cv::Mat> MatPtr;
      typedef boost::shared_ptr<cv::Mat const> MatConstPtr;

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
      DiscreteWaveletTransform(const cv::Mat& rowKernelLow,
          const cv::Mat& columnKernelLow, const cv::Mat& rowKernelHigh,
          const cv::Mat& columnKernelHigh);

      /**
       * @brief Virtual Destructor
       **/
      virtual ~DiscreteWaveletTransform();

    private:
      /**
       * @brief Perform convolution with a vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param kernel [const cv::Mat&] The filter used
       * @param rows [bool] Whether the type of convolution will
       * be performed row-wise
       * @return [cv::Mat] The result of the convolution
       **/
      cv::Mat optionalConv(const cv::Mat& inImage,
          const cv::Mat& kernel, bool rows);
      /**
       * @brief Perform convolution with a column vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param kernel [const cv::Mat&] The filter used
       * @return [cv::Mat] The result of the convolution performed
       * column-wise
       **/
      cv::Mat convCols(const cv::Mat& inImage,
          const cv::Mat& kernel);
      /**
       * @brief Perform convolution with a row vector kernel
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param kernel [const cv::Mat&] The filter used
       * @return [cv::Mat] The result of the convolution performed
       * row-wise
       **/
      cv::Mat convRows(const cv::Mat& inImage,
          const cv::Mat& kernel);

      /**
       * @brief Creating an image by taking half the values of the
       * image that the convolution with the low frequency kernel
       * resulted to
       * @param imageLow [const cv::Mat&] The input low frequency image
       * @param rows [bool] Whether the images are downsampled row-wise
       * @param subImageLow [const MatPtr&] The output low frequency image
       **/
      void subSampleLow(const cv::Mat& imageLow, bool rows,
          const MatPtr& subImageLow);

      /**
       * @brief Creating an image by taking half the values of the
       * image that the convolution with the high frequency kernel
       * resulted to
       * @param imageHigh [const cv::Mat&] The input high frequency image
       * @param rows [bool] Whether the images are downsampled row-wise
       * @param subImageHigh [const MatPtr&] The output high frequency image
       **/
      void subSampleHigh(const cv::Mat& imageHigh, bool rows,
          const MatPtr& subImageHigh);

      /**
       * @brief Create two images by taking half the values of the two input
       * images respectively
       * @param imageLow [const cv::Mat&] The input low frequency image
       * @param imageHigh [const cv::Mat&] The input high frequency image
       * @param rows [bool] Whether the images are downsampled row-wise
       * @param subImageLow [const MatPtr&] The output low frequency image
       * @param subImageHigh [const MatPtr&] The output high frequency image
       **/
      void subSample(const cv::Mat& imageLow, const cv::Mat& imageHigh,
          bool rows, const MatPtr& subImageLow, const MatPtr& subImageHigh);

      /**
       * @brief Perform convolution and subsampling row-wise or
       * column-wise according to boolean input
       * @param inImage [const cv::Mat&] The image to be convolved
       * @param rows [bool] Whether the convolution is performed
       * row-wise. Then downsampling is performed column-wise
       * @param subImageLow [const MatPtr&] The output low frequency image
       * @param subImageHigh [const MatPtr&] The output high frequency image
       **/
      void convAndSubSample(const cv::Mat& inImage, bool rows,
          const MatPtr& subImageLow, const MatPtr& subImageHigh);

      /**
       * @brief Return the final LL result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of LL images that are
       * the result of the transform according to the level
       **/
      std::vector<MatPtr> getLowLow(const cv::Mat& inImage, int level = 1);

      /**
       * @brief Return the final LH result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of LH images that are
       * the result of the transform according to the level
       **/
      std::vector<MatPtr> getLowHigh(const cv::Mat& inImage, int level = 1);

      /**
       * @brief Return the final HL result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of HL images that are
       * the result of the transform according to the level
       **/
      std::vector<MatPtr> getHighLow(const cv::Mat& inImage, int level = 1);

      /**
       * @brief Return the final HH result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of HH images that are
       * the result of the transform according to the level
       **/
      std::vector<MatPtr> getHighHigh(const cv::Mat& inImage, int level = 1);

      /**
       * @brief Return the final result of the DWT
       * @param inImage [const cv::Mat& inImage] The image to which
       * the transform is performed
       * @param level [int] The number of stages of the DWT
       * @return [std::vector<MatPtr>] The list of images that are
       * the result of the transform with order LL, LH, HL, HH and
       * so on according to the level
       **/
      std::vector<MatPtr> dwt2D(const cv::Mat& inImage, int level = 1);

    private:
      /// The row kernel used to perform the DWT that represents the
      /// low - pass filter used
      cv::Mat rowKernelLow_;
      /// The column kernel used to perform the DWT that represents
      /// the  low - pass filter used
      cv::Mat columnKernelLow_;
      /// The row kernel used to perform the DWT that represents the
      /// band - pass filter used for high frequencies
      cv::Mat rowKernelHigh_;
      /// The column kernel used to perform the DWT that represents
      /// the band - pass filter used for high frequencies
      cv::Mat columnKernelHigh_;
  };
  typedef DiscreteWaveletTransform::MatPtr MatPtr;
  typedef DiscreteWaveletTransform::MatConstPtr MatConstPtr;

}  // namespace pandora_vision

#endif  // PANDORA_VISION_OBSTACLE_DISCRETE_WAVELET_TRANSFORM_H
