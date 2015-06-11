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

#include "pandora_vision_obstacle/discrete_wavelet_transform.h"

namespace pandora_vision
{
  DiscreteWaveletTransform::DiscreteWaveletTransform(int kernelSize)
  {
  }

  DiscreteWaveletTransform::DiscreteWaveletTransform(const cv::Mat& kernelLow,
      const cv::Mat& kernelHigh)
  {
    if (kernelLow.rows == 1)
    {
      kernelLow_ = kernelLow;
    }
    else
    {
      cv::transpose(kernelLow, kernelLow_);
    }

    if (kernelHigh.rows == 1)
    {
      kernelHigh_ = kernelHigh;
    }
    else
    {
      cv::transpose(kernelHigh, kernelHigh_);
    }
  }

  DiscreteWaveletTransform::~DiscreteWaveletTransform()
  {
  }

  cv::Mat DiscreteWaveletTransform::optionalConv(const cv::Mat& inImage,
      const cv::Mat& kernel, bool cols)
  {
    cv::Mat paddedImage, transKernel;
    if (cols)
    {
      cv::transpose(kernel, transKernel);
      cv::copyMakeBorder(inImage, paddedImage, 0, 1, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
    }
    else
    {
      transKernel = kernel.clone();
      cv::copyMakeBorder(inImage, paddedImage, 0, 0, 0, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    }

    cv::Mat result;
    cv::filter2D(paddedImage, result, -1, transKernel, cv::Point(-1, -1), cv::BORDER_CONSTANT);

    return result;
  }

  cv::Mat DiscreteWaveletTransform::convCols(const cv::Mat& inImage,
      const cv::Mat& kernel)
  {
    return optionalConv(inImage, kernel, true);
  }

  cv::Mat DiscreteWaveletTransform::convRows(const cv::Mat& inImage,
      const cv::Mat& kernel)
  {
    return optionalConv(inImage, kernel, false);
  }

  void DiscreteWaveletTransform::subSample(const cv::Mat& imageLow,
      const cv::Mat& imageHigh, bool rows, const MatPtr& subImageLow,
      const MatPtr& subImageHigh)
  {
    if (rows)
    {
      for (int jj = 0; jj < imageLow.rows; jj++)
      {
        if (jj % 2)
        {
          subImageLow->push_back(imageLow.row(jj));
        }
        else
        {
          subImageHigh->push_back(imageHigh.row(jj));
        }
      }
    }
    else
    {
      *subImageLow = cv::Mat(cv::Mat::zeros(imageLow.rows, static_cast<int>(std::ceil(
          imageLow.cols / 2.0f)), CV_32FC1));
      *subImageHigh = cv::Mat::zeros(imageHigh.rows, static_cast<int>(
          std::floor(imageHigh.cols / 2.0f)), CV_32FC1);
      int imageIndex = 0;
      for (int jj = 0; jj < imageLow.cols; jj += 2)
      {
        imageLow.col(jj).copyTo(subImageLow->col(imageIndex));
        if (jj < imageLow.cols - 1)
        {
          imageHigh.col(jj + 1).copyTo(subImageHigh->col(imageIndex));
        }
        imageIndex += 1;
      }
    }
  }

  void DiscreteWaveletTransform::convAndSubSample(const cv::Mat& inImage, bool rows,
      const MatPtr& subImageLow, const MatPtr& subImageHigh)
  {
    cv::Mat imageConvLow = optionalConv(inImage, kernelLow_, !rows);
    cv::Mat imageConvHigh = optionalConv(inImage, kernelHigh_, !rows);

    subSample(imageConvLow, imageConvHigh, rows, subImageLow, subImageHigh);
  }

  std::vector<MatPtr> DiscreteWaveletTransform::dwt2D(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> dwtImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      MatPtr subImageL(new cv::Mat), subImageH(new cv::Mat);
      convAndSubSample(input, false, subImageL, subImageH);

      MatPtr subImageLL(new cv::Mat), subImageLH(new cv::Mat);
      convAndSubSample(*subImageL, true, subImageLL, subImageLH);

      dwtImages.push_back(subImageLL);
      dwtImages.push_back(subImageLH);

      MatPtr subImageHL(new cv::Mat), subImageHH(new cv::Mat);
      convAndSubSample(*subImageH, true, subImageHL, subImageHH);

      dwtImages.push_back(subImageHL);
      dwtImages.push_back(subImageHH);

      if (ii < level - 1)
      {
        input = subImageLL->clone();
      }
    }
    return dwtImages;
  }

}  // namespace pandora_vision
