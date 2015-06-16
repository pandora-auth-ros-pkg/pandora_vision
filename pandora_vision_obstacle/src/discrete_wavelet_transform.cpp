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

  DiscreteWaveletTransform::DiscreteWaveletTransform(const cv::Mat& columnKernelLow,
      const cv::Mat& columnKernelHigh)
  {
      columnKernelLow_ = columnKernelLow;
      cv::transpose(columnKernelLow, rowKernelLow_);

      columnKernelHigh_ = columnKernelHigh;
      cv::transpose(columnKernelHigh, rowKernelHigh_);
  }

  DiscreteWaveletTransform::~DiscreteWaveletTransform()
  {
  }

  cv::Mat DiscreteWaveletTransform::optionalConv(const cv::Mat& inImage,
      const cv::Mat& kernel, bool rows)
  {
    cv::Mat paddedImage;
    if (rows)
    {
      cv::copyMakeBorder(inImage, paddedImage, 0, 0, 0, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    }
    else
    {
      cv::copyMakeBorder(inImage, paddedImage, 0, 1, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0));
    }
    cv::Mat result;
    cv::filter2D(paddedImage, result, -1, kernel, cv::Point(-1, -1), cv::BORDER_CONSTANT);

    return result;
  }

  cv::Mat DiscreteWaveletTransform::convCols(const cv::Mat& inImage,
      const cv::Mat& kernel)
  {
    return optionalConv(inImage, kernel, false);
  }

  cv::Mat DiscreteWaveletTransform::convRows(const cv::Mat& inImage,
      const cv::Mat& kernel)
  {
    return optionalConv(inImage, kernel, true);
  }

  void DiscreteWaveletTransform::subSampleLow(const cv::Mat& imageLow,
      bool rows, const MatPtr& subImageLow)
  {
    if (rows)
    {
      for (int jj = 0; jj < imageLow.rows; jj += 2)
      {
        subImageLow->push_back(imageLow.row(jj));
      }
    }
    else
    {
      *subImageLow = cv::Mat(cv::Mat::zeros(imageLow.rows, static_cast<int>(std::ceil(
          imageLow.cols / 2.0f)), CV_32FC1));
      int imageIndex = 0;
      for (int jj = 0; jj < imageLow.cols; jj += 2)
      {
        imageLow.col(jj).copyTo(subImageLow->col(imageIndex));
        imageIndex += 1;
      }
    }
  }

  void DiscreteWaveletTransform::subSampleHigh(const cv::Mat& imageHigh,
      bool rows, const MatPtr& subImageHigh)
  {
    if (rows)
    {
      for (int jj = 1; jj < imageHigh.rows; jj += 2)
      {
        subImageHigh->push_back(imageHigh.row(jj));
      }
    }
    else
    {
      *subImageHigh = cv::Mat::zeros(imageHigh.rows, static_cast<int>(
          std::floor(imageHigh.cols / 2.0f)), CV_32FC1);
      int imageIndex = 0;
      for (int jj = 1; jj < imageHigh.cols - 1; jj += 2)
      {
        imageHigh.col(jj).copyTo(subImageHigh->col(imageIndex));
        imageIndex += 1;
      }
    }
  }

  void DiscreteWaveletTransform::subSample(const cv::Mat& imageLow,
      const cv::Mat& imageHigh, bool rows, const MatPtr& subImageLow,
      const MatPtr& subImageHigh)
  {
    subSampleLow(imageLow, rows, subImageLow);
    subSampleHigh(imageHigh, rows, subImageHigh);
  }

  void DiscreteWaveletTransform::convAndSubSample(const cv::Mat& inImage, bool rows,
      const MatPtr& subImageLow, const MatPtr& subImageHigh)
  {
    cv::Mat imageConvLow, imageConvHigh;

    if (rows)
    {
      imageConvLow = optionalConv(inImage, rowKernelLow_, rows);
      imageConvHigh = optionalConv(inImage, rowKernelHigh_, rows);
    }
    else
    {
      imageConvLow = optionalConv(inImage, columnKernelLow_, rows);
      imageConvHigh = optionalConv(inImage, columnKernelHigh_, rows);
    }
    subSample(imageConvLow, imageConvHigh, rows, subImageLow, subImageHigh);
  }

  std::vector<MatPtr> DiscreteWaveletTransform::getLowLow(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> LLImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      cv::Mat imageConvLow = convCols(input, columnKernelLow_);

      MatPtr subImageL(new cv::Mat);
      subSampleLow(imageConvLow, false, subImageL);

      imageConvLow = convRows(*subImageL, rowKernelLow_);

      MatPtr subImageLL(new cv::Mat);
      subSampleLow(imageConvLow, true, subImageLL);

      LLImages.push_back(subImageLL);

      if (ii < level - 1)
      {
        input = subImageLL->clone();
      }
    }
    return LLImages;
  }

  std::vector<MatPtr> DiscreteWaveletTransform::getLowHigh(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> LHImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      cv::Mat imageConvLow = convCols(input, columnKernelLow_);

      MatPtr subImageL(new cv::Mat);
      subSampleLow(imageConvLow, false, subImageL);

      MatPtr subImageLL(new cv::Mat), subImageLH(new cv::Mat);
      convAndSubSample(*subImageL, true, subImageLL, subImageLH);

      LHImages.push_back(subImageLH);

      if (ii < level - 1)
      {
        input = subImageLL->clone();
      }
    }
    return LHImages;
  }

  std::vector<MatPtr> DiscreteWaveletTransform::getHighLow(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> HLImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      MatPtr subImageL(new cv::Mat), subImageH(new cv::Mat);
      convAndSubSample(input, false, subImageL, subImageH);

      cv::Mat imageConvLow = convRows(*subImageH, rowKernelLow_);

      MatPtr subImageHL(new cv::Mat);
      subSampleLow(imageConvLow, true, subImageHL);

      HLImages.push_back(subImageHL);

      if (ii < level - 1)
      {
        imageConvLow = convRows(*subImageL, rowKernelLow_);

        MatPtr subImageLL(new cv::Mat);
        subSampleLow(imageConvLow, true, subImageLL);

        input = subImageLL->clone();
      }
    }
    return HLImages;
  }

  std::vector<MatPtr> DiscreteWaveletTransform::getHighHigh(const cv::Mat& inImage, int level)
  {
    std::vector<MatPtr> HHImages;
    cv::Mat input = inImage.clone();

    for (int ii = 0; ii < level; ii++)
    {
      MatPtr subImageL(new cv::Mat), subImageH(new cv::Mat);
      convAndSubSample(input, false, subImageL, subImageH);

      cv::Mat imageConvHigh = convRows(*subImageH, rowKernelHigh_);

      MatPtr subImageHH(new cv::Mat);
      subSampleHigh(imageConvHigh, true, subImageHH);

      HHImages.push_back(subImageHH);

      if (ii < level - 1)
      {
        cv::Mat imageConvLow = convRows(*subImageL, rowKernelLow_);

        MatPtr subImageLL(new cv::Mat);
        subSampleLow(imageConvLow, true, subImageLL);

        input = subImageLL->clone();
      }
    }
    return HHImages;
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
