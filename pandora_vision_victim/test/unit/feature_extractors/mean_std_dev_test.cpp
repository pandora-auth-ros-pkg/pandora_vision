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
* Author: Marios Protopapas
*********************************************************************/

#include "pandora_vision_victim/feature_extractors/mean_std_dev.h"
#include "gtest/gtest.h"

namespace pandora_vision
{
  /**
    @class MeanStdDevExtractorTest 
    @brief Tests the integrity of methods of class MeanStdDevExtractor
   **/
  class MeanStdDevExtractorTest : public ::testing::Test
  {
    protected:

      MeanStdDevExtractorTest () {}
      
       virtual void SetUp()
      {

        HEIGHT = 10;
        WIDTH = 10;

        // Construct a black image
        black = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1);
        
        // Construct a white image
        white = cv::Mat::ones( HEIGHT, WIDTH, CV_32FC1);
        
        for ( int rows = 0; rows < HEIGHT; rows++ )
        {
          for ( int cols = 0; cols < WIDTH; cols++ )
          {
            white.at< float >( rows, cols ) = 255.0;
          }
        }
        
        // Construct a half black - half white image
        blackWhite = cv::Mat::zeros( HEIGHT, WIDTH, CV_32FC1);

        for ( int rows = 0; rows < HEIGHT/2; rows++ )
        {
          for ( int cols = 0; cols < WIDTH/2; cols++ )
          {
            blackWhite.at< float >( rows, cols ) = 255.0;
          }
        }
      }
      
      // The images' dimensions
      int HEIGHT;
      int WIDTH;

      // Images that will be used for testing
      cv::Mat black,white,blackWhite;
  };
  
  //! Tests MeanStdDevExtractor::extract
  TEST_F ( MeanStdDevExtractorTest, extractMeanStd )
  {
    // The output vector
    std::vector<double>  out;
    MeanStdDevExtractor msd1(&black), msd2(&white), msd3(&blackWhite);
    out = msd1.extract();
    EXPECT_EQ ( 0 , out[0] );
    EXPECT_EQ ( 0 , out[1] );

    out = msd2.extract();
    EXPECT_EQ ( 255 , out[0] );
    EXPECT_EQ ( 0 , out[1] );

    out = msd3.extract();
    EXPECT_EQ ( 127.5 , out[0] );
    EXPECT_EQ ( 134.3968 , out[1] );
  }
    
    
  
} // namespace pandora_vision
