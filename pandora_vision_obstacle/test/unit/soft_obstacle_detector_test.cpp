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

#include <vector>
#include <gtest/gtest.h>
#include "pandora_vision_obstacle/soft_obstacle_detection/soft_obstacle_detector.h"

namespace pandora_vision
{
  class SoftObstacleDetectorTest : public ::testing::Test
  {
    public:
      SoftObstacleDetectorTest() :
        detector_(new SoftObstacleDetector(13, 2.0, 3.0)) {}

      bool findNonIdenticalLines(const std::vector<cv::Vec2f> lineCoeffs,
          float grad, float beta)
      {
        return detector_->findNonIdenticalLines(lineCoeffs, grad, beta);
      }

      float detectROI(const std::vector<cv::Vec4i>& verticalLines,
            int frameHeight, const boost::shared_ptr<cv::Rect>& roiPtr)
      {
        return detector_->detectROI(verticalLines, frameHeight, roiPtr);
      }

    protected:
      boost::shared_ptr<SoftObstacleDetector> detector_;
  };

  TEST_F(SoftObstacleDetectorTest, areIdenticalLinesDetected)
  {
    std::vector<cv::Vec2f> lineCoeffs;
    lineCoeffs.push_back(cv::Vec2f(89, 20));
    lineCoeffs.push_back(cv::Vec2f(85, 30));

    ASSERT_FALSE(findNonIdenticalLines(lineCoeffs, 90, 22));
    ASSERT_TRUE(findNonIdenticalLines(lineCoeffs, 85, 35));
    ASSERT_TRUE(findNonIdenticalLines(lineCoeffs, 95, 28));
  }

  TEST_F(SoftObstacleDetectorTest, isROIDetectedCorrectly)
  {
    std::vector<cv::Vec4i> lines;
    lines.push_back(cv::Vec4i(1, 1, 1, 3));
    lines.push_back(cv::Vec4i(2, 1, 2, 3));
    lines.push_back(cv::Vec4i(4, 1, 4, 3));

    boost::shared_ptr<cv::Rect> roi(new cv::Rect);

    ASSERT_EQ(0.5, detectROI(lines, 4, roi));

    EXPECT_EQ(1, roi->x);
    EXPECT_EQ(1, roi->y);
    EXPECT_EQ(3, roi->width);
    EXPECT_EQ(2, roi->height);
  }
}  // namespace pandora_vision
