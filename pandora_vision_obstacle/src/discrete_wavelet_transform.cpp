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
  DiscreteWaveletTransform::DiscreteWaveletTransform(const cv::Mat& kernel)
  {
    if (kernel.rows == 1)
    {
      kernel_ = kernel;
    }
    else
    {
      cv::transpose(kernel, kernel_);
    }
  }

  DiscreteWaveletTransform::~DiscreteWaveletTransform()
  {
  }

  cv::Mat DiscreteWaveletTransform::convCols(const cv::Mat& inImage)
  {
    cv::Mat paddedImage;
    cv::copyMakeBorder(inImage, paddedImage, 0, 1, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0));

    cv::Mat transKernel;
    cv::transpose(kernel_, transKernel);

    cv::Mat result;
    cv::filter2D(paddedImage, result, -1, transKernel, cv::Point(-1, -1), cv::BORDER_CONSTANT);

    return result;
  }

  cv::Mat DiscreteWaveletTransform::convRows(const cv::Mat& inImage)
  {
    cv::Mat paddedImage;
    cv::copyMakeBorder(inImage, paddedImage, 0, 0, 0, 1, cv::BORDER_CONSTANT, cv::Scalar(0));

    cv::Mat result;
    cv::filter2D(paddedImage, result, -1, kernel_, cv::Point(-1, -1), cv::BORDER_CONSTANT);

    return result;
  }

}  // namespace pandora_vision
