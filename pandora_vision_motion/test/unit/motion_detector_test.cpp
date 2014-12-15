/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Despoina Paschalidou
*********************************************************************/
#include "pandora_vision_motion/motion_detector.h"

#include "gtest/gtest.h"
namespace pandora_vision
{
  /**
  @class MotionDetectorTests
  @brief Tests the integrity of methods of class MotionDetector
  **/
  class MotionDetectorTest : public ::testing::Test
  {
    public:
      MotionDetectorTest() {}
      /**
        @brief Constructs a rectangle of specific width and height.
        @param upperLeft [const cv::Point2f] The upper left vertex of
        the rectangle to be created
        @param x [const int] The rectangle's width
        @param height [const int] The rectangle's height
        @param color [const unsigned char color] The value for all points inside the
        rectangle
        @param image [cv::Mat&] The image on which the rectangle will
        be imprinted on
        @return void
      **/
      void drawRectangle (
        const cv::Point& upperLeft,
        const int width,
        const int height,
        const unsigned char color,
        cv::Mat &image 
      );
      
      virtual void SetUp()
      {
        WIDTH = 640;
        HEIGHT = 480;
      }
      
      int* detectMotionPosition(cv::Mat frame);
    
      int WIDTH;
      int HEIGHT;
    private:
      MotionDetector motionDetector_;
  };
  /**
    @brief Constructs a rectangle of specific width and height.
    @param upperLeft [const cv::Point2f] The upper left vertex of
    the rectangle to be created
    @param x [const int] The rectangle's width
    @param height [const int] The rectangle's height
    @param color [const unsigned char] The value for all points inside the
    rectangle
    @param image [cv::Mat&] The image on which the rectangle will
    be imprinted on
    @return void
  **/
  void MotionDetectorTest::drawRectangle (
    const cv::Point& upperLeft,
    const int width,
    const int height,
    const unsigned char color,
    cv::Mat &image
  )
  {
    for(int rows = upperLeft.y; rows < upperLeft.y + height; rows++)
    {
      for (int cols = upperLeft.x; cols < upperLeft.x + width; cols++)
      {
        for (int channels = 0; channels < image.channels(); channels++)
        {
          image.at<cv::Vec3b>(rows, cols).val[channels] = color;
        }
      }
    }
  }
  
  int* MotionDetectorTest::detectMotionPosition(cv::Mat frame)
  {
    motionDetector_.detectMotionPosition(frame);
    return motionDetector_.getMotionPosition();
  }
  
  //! Tests MotionDetector::detectMotionPosition
  TEST_F (MotionDetectorTest, detectMotionPositionBlackImage)
  {
    cv::Mat blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
    int* bounding_boxes  = detectMotionPosition(blackFrame);
    for(int i=0; i< 4; i++)
      EXPECT_EQ(0, bounding_boxes[i]);
  }
  
  TEST_F (MotionDetectorTest, detectMotionPositionBlackImageWithRectangle)
  {
    cv::Mat frame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
    frame(cv::Rect(17, 63, 8, 8))=255;
    //~ drawRectangle(point, 8, 8, 255, frame);
    int* bounding_boxes  = detectMotionPosition(frame);
    EXPECT_EQ(21, bounding_boxes[0]);
    EXPECT_EQ(67, bounding_boxes[1]);
    EXPECT_EQ(8, bounding_boxes[2]);
    EXPECT_EQ(8, bounding_boxes[3]);
  }
} // namespace pandora_vision
