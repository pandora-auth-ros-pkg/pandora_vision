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

#include <cmath>
#include <gtest/gtest.h>
#include "pandora_vision_obstacle/discrete_wavelet_transform.h"

namespace pandora_vision
{
  class DiscreteWaveletTransformTest : public ::testing::Test
  {
    public:
      DiscreteWaveletTransformTest()
      {
        cv::Mat kernelLow = (cv::Mat_<float>(2, 1) << 1 / sqrt(2), 1 / sqrt(2));
        cv::Mat kernelHigh = (cv::Mat_<float>(2, 1) << - 1 / sqrt(2), 1 / sqrt(2));

        dwtPtr_.reset(new DiscreteWaveletTransform(kernelLow, kernelHigh));
      }

      cv::Mat convRows(const cv::Mat& image, bool low)
      {
        return dwtPtr_->convRows(image, low);
      }

      cv::Mat convCols(const cv::Mat& image, bool low)
      {
        return dwtPtr_->convCols(image, low);
      }

      void subSample(const cv::Mat& imageLow, const cv::Mat& imageHigh, bool rows,
          const MatPtr& subImageLow, const MatPtr& subImageHigh)
      {
        return dwtPtr_->subSample(imageLow, imageHigh, rows, subImageLow,
            subImageHigh);
      }

    protected:
      virtual void SetUp() {}

      boost::shared_ptr<DiscreteWaveletTransform> dwtPtr_;
  };

  TEST_F(DiscreteWaveletTransformTest, isPadSizeInConvolutionCorrect)
  {
    cv::Mat image = cv::Mat::ones(3, 3, CV_32FC1);

    cv::Mat imageConvRows = convRows(image, true);
    ASSERT_EQ(4, imageConvRows.cols);
    ASSERT_EQ(3, imageConvRows.rows);

    cv::Mat imageConvCols = convCols(image, true);
    ASSERT_EQ(3, imageConvCols.cols);
    ASSERT_EQ(4, imageConvCols.rows);
  }

  TEST_F(DiscreteWaveletTransformTest, isConvolutionResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);

    cv::Mat imageConvRows = convRows(image, true);
    cv::Mat rowsResult = (cv::Mat_<float>(3, 4) << 1, 3, 5, 3, 4, 9, 11,
        6, 7, 15, 17, 9);
    rowsResult /= static_cast<float>(std::sqrt(2));

    ASSERT_TRUE(imageConvRows.rows == rowsResult.rows);
    ASSERT_TRUE(imageConvRows.cols == rowsResult.cols);
    for (int ii = 0; ii < rowsResult.rows; ii++)
    {
      for (int jj = 0; jj < rowsResult.cols; jj++)
      {
        EXPECT_NEAR(rowsResult.at<float>(ii, jj), imageConvRows.at<float>(ii, jj), 1e-6);
      }
    }

    cv::Mat imageConvCols = convCols(image, true);
    cv::Mat colsResult = (cv::Mat_<float>(4, 3) << 1, 2, 3, 5, 7, 9,
        11, 13, 15, 7, 8, 9);
    colsResult /= static_cast<float>(std::sqrt(2));

    ASSERT_TRUE(imageConvCols.rows == colsResult.rows);
    ASSERT_TRUE(imageConvCols.cols == colsResult.cols);
    for (int ii = 0; ii < colsResult.rows; ii++)
    {
      for (int jj = 0; jj < colsResult.cols; jj++)
      {
        EXPECT_NEAR(colsResult.at<float>(ii, jj), imageConvCols.at<float>(ii, jj), 1e-6);
      }
    }
  }

  TEST_F(DiscreteWaveletTransformTest, isSubSampleRowsResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 1, 1, 2, 2, 2,
        3, 3, 3);

    MatPtr subImageLow(new cv::Mat());
    MatPtr subImageHigh(new cv::Mat());

    subSample(image, image, true, subImageLow, subImageHigh);

    ASSERT_EQ(2, subImageLow->rows);
    ASSERT_EQ(3, subImageLow->cols);

    ASSERT_EQ(1, subImageHigh->rows);
    ASSERT_EQ(3, subImageHigh->cols);

    for (int ii = 0; ii < subImageLow->cols; ii++)
    {
      EXPECT_EQ(1, subImageLow->at<float>(0, ii));
      EXPECT_EQ(2, subImageHigh->at<float>(0, ii));
      EXPECT_EQ(3, subImageLow->at<float>(1, ii));
    }
  }

  TEST_F(DiscreteWaveletTransformTest, isSubSampleColsResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 2, 3, 1, 2, 3,
        1, 2, 3);

    MatPtr subImageLow(new cv::Mat());
    MatPtr subImageHigh(new cv::Mat());

    subSample(image, image, false, subImageLow, subImageHigh);

    ASSERT_EQ(2, subImageLow->cols);
    ASSERT_EQ(3, subImageLow->rows);

    ASSERT_EQ(1, subImageHigh->cols);
    ASSERT_EQ(3, subImageHigh->rows);

    for (int ii = 0; ii < subImageLow->rows; ii++)
    {
      EXPECT_EQ(1, subImageLow->at<float>(ii, 0));
      EXPECT_EQ(2, subImageHigh->at<float>(ii, 0));
      EXPECT_EQ(3, subImageLow->at<float>(ii, 1));
    }
  }

  TEST_F(DiscreteWaveletTransformTest, isLowLowResultCorrect)
  {
    cv::Mat image = (cv::Mat_<float>(3, 3) << 1, 2, 3, 4, 5, 6, 7, 8, 9);

    std::vector<MatPtr> llImages = dwtPtr_->getLowLow(image);

    ASSERT_EQ(1, llImages.size());
    //std::cout << *llImages[0] << std::endl;
    ASSERT_EQ(2, llImages[0]->rows);
    ASSERT_EQ(2, llImages[0]->cols);
  }
}  // namespace pandora_vision
