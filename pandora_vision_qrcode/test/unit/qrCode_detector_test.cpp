/*********************************************************************
 * *
 * * Software License Agreement (BSD License)
 * *
 * * Copyright (c) 2014, P.A.N.D.O.R.A. Team.
 * * All rights reserved.
 * *
 * * Redistribution and use in source and binary forms, with or without
 * * modification, are permitted provided that the following conditions
 * * are met:
 * *
 * * * Redistributions of source code must retain the above copyright
 * * notice, this list of conditions and the following disclaimer.
 * * * Redistributions in binary form must reproduce the above
 * * copyright notice, this list of conditions and the following
 * * disclaimer in the documentation and/or other materials provided
 * * with the distribution.
 * * * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 * * contributors may be used to endorse or promote products derived
 * * from this software without specific prior written permission.
 * *
 * * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * * POSSIBILITY OF SUCH DAMAGE.
 * *
 * * Author: 
 * *********************************************************************/
#include "pandora_vision_qrcode/qrCode_detector.h"

#include "gtest/gtest.h"
#include "math.h"
namespace pandora_vision
{

    /**
     *     @class QrCodeDetectorTests
     *     @brief Tests the integrity of methods of class QrCodeDetector
    **/
  
    class QrCodeDetectorTest : public ::testing::Test
    {
      public:
        QrCodeDetectorTest() {}

        virtual void SetUp()
        {
          WIDTH = 640;
          HEIGHT = 480;
        }

        std::vector<QrCode> detectQrCode(cv::Mat frame);

        void drawChessboard (
          int blocksNumber,
          int WIDTH,
          int HEIGHT,
          cv::Mat &image
        );

        int WIDTH;
        int HEIGHT;

      private:
        QrCodeDetector qrCodeDetector_;
    };

    /**
      @brief Constructs a chessboard with specific number of blocks.
      @param blocksNumber [int] The number of chessboard blocks
      @param WIDTH [int] Output image's width
      @param HEIGHT [int]  Output image's height
      @param image [cv::Mat&] The final chessboard image
      @return void
    **/
    void QrCodeDetectorTest::drawChessboard (
      int blocksNumber,
      int WIDTH,
      int HEIGHT,
      cv::Mat &image
    )
    {
      int imageSize = WIDTH * HEIGHT;
      int blockSize = static_cast<int>(imageSize / blocksNumber);
      cv::Mat chessBoard(WIDTH, HEIGHT, CV_8UC3, cv::Scalar::all(0));          
      unsigned char color = 255;
      for (int i = 0; i < WIDTH - blockSize; i += blockSize)
      {
        color = -color;
        for (int j = 0; j < HEIGHT - blockSize; j += blockSize)
        {
          cv::Mat ROI = chessBoard(cv::Rect(i, j, blockSize, blockSize));
          ROI.setTo(cv::Scalar::all(color));
          color = abs(color-255);
        }
      }
      chessBoard.copyTo(image);
    }

    std::vector<QrCode> QrCodeDetectorTest::detectQrCode(cv::Mat frame)
    {
      qrCodeDetector_.detectQrCode(frame);
      return qrCodeDetector_.get_detected_qr();
    }

    //! Tests QrCodeDetector::detectQrCode
    TEST_F (QrCodeDetectorTest, detectQrCodeBlackImage)
    {
      cv::Mat blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
      std::vector<QrCode> qrcode_list = detectQrCode(blackFrame);
      // there shouldn't be any qrcodes
      EXPECT_EQ(0, qrcode_list.size());
      // neither when 3 channels are used
      blackFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
      qrcode_list = detectQrCode(blackFrame);
      EXPECT_EQ(0, qrcode_list.size());
    }

    TEST_F (QrCodeDetectorTest, detectQrCodeWhiteImage)
    {
      cv::Mat whiteFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC1);
      whiteFrame.setTo(cv::Scalar(255,255,255));
      std::vector<QrCode> qrcode_list = detectQrCode(whiteFrame);
      // there shouldn't be any qrcodes
      EXPECT_EQ(0, qrcode_list.size());
    }

    TEST_F (QrCodeDetectorTest, detectQrCodeWhiteBlackMixImage)
    {
      // Vertically concatenated
      cv::Mat blackFrame = cv::Mat::zeros(HEIGHT/2, WIDTH/2, CV_8UC3);
      cv::Mat whiteFrame = cv::Mat::zeros(HEIGHT/2, WIDTH/2, CV_8UC3);
      whiteFrame.setTo(cv::Scalar(255,255,255));
      cv::Mat H, V;
      cv::hconcat(blackFrame, whiteFrame, H);
      cv::vconcat(blackFrame, whiteFrame, V);
      std::vector<QrCode> qrcode_list = detectQrCode(H);
      // there shouldn't be any qrcodes
      EXPECT_EQ(0, qrcode_list.size());
      qrcode_list = detectQrCode(V);
      EXPECT_EQ(0, qrcode_list.size());
    }

    //TEST_F (QrCodeDetectorTest, detectQrCodeRandomChessboardImage)
    //{
    //  cv::Mat frame;
    //  int blocksNumber = 10;
    //  drawChessboard( blocksNumber, WIDTH, HEIGHT, frame);
    //  std::vector<QrCode> qrcode_list = detectQrCode(frame);
    //  // there shouldn't be any qrcodes
    //  EXPECT_EQ(0, qrcode_list.size());
    //  blocksNumber = 100;
    //  drawChessboard( blocksNumber, WIDTH, HEIGHT, frame);
    //  qrcode_list = detectQrCode(frame);
    //  // there shouldn't be any qrcodes
    //  EXPECT_EQ(0, qrcode_list.size());
    //}

    
    //TEST_F (QrCodeDetectorTest, detectBigQrCodeWhiteBackground)
    //{
    //  cv::Mat inputFrame;
    //  inputFrame = cv::imread("");
    //  //cv::resize(inputFrame, inputFrame, cv::Size(WIDTH, HEIGHT));
    //  std::vector<QrCode> qrcode_list = detectQrCode(inputFrame);
    //  // there should be two qrcodes
    //  EXPECT_EQ(2, qrcode_list.size());
    //  inputFrame = cv::imread("");
    //  qrcode_list = detectQrCode(inputFrame);
    //  // there should be one qrcode
    //  EXPECT_EQ(1, qrcode_list.size());
    //}

    //TEST_F (QrCodeDetectorTest, detectOneQrCodeCamera)
    //{
    //  cv::VideoCapture camera( 0 );  
    //  cv::Mat inputFrame;
    //  int i = 0;
    //  int qrNumber = 0;
    //  std::vector<QrCode> qrcode_list;
    //  while( i<100 )
    //  {
    //    // Get the next frame.
    //    camera.grab();
    //    camera.retrieve(inputFrame);
    //    qrcode_list = detectQrCode(inputFrame);
    //    qrNumber = qrcode_list.size();
    //    i++;
    //  }
    //  // there should be one qrcode
    //  EXPECT_EQ(1, qrNumber);
    //}
      
} // namespace_pandora_vision
