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
 * Author: Angelos Triantafyllidis
 *********************************************************************/

#include "pandora_vision_landoltc/landoltc_2d/landoltc_detector.h"
#include "gtest/gtest.h"


namespace pandora_vision
{
  /**
    @class LandoltcDetectorTest
    @brief Tests the integrity of methods of class LandoltcDetector
   **/
  class LandoltcDetectorTest : public ::testing::Test
  {
    public:
      LandoltcDetectorTest() {}
      virtual ~LandoltcDetectorTest(){}
      
    protected:
      void fillGrad(cv::Mat& input);
      cv::Point giveCenters(int i);
      int giveVotingData(cv::Point a, cv::Point b, int y, int i);
    
      LandoltCDetector landoltCDetector;
      
      
  }; 
  
  //process input and call findcenters
  void LandoltcDetectorTest::fillGrad(cv::Mat& input)
  {
		cv::Mat gradX, gradY;
		
		cv::Sobel(input, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(input, gradY, CV_32F, 0, 1, 3);
    
     float* gradXF = reinterpret_cast<float*>(gradX.data);
     float* gradYF = reinterpret_cast<float*>(gradY.data);
    
    landoltCDetector.findCenters(input.rows, input.cols, gradXF, gradYF);
    
	}
	
	//returns possible center
	cv::Point LandoltcDetectorTest::giveCenters(int i)
	{ 
		return landoltCDetector._centers.at(i);
	}
  
  
  //y and i change order invotingData depending on the points i give to rasterize
	int LandoltcDetectorTest::giveVotingData(cv::Point a, cv::Point b , int y , int i)
	{
		cv::Mat image;
		
		image=cv::imread("/pandora_vision_landoltc/bold.jpg",-1);
		
		landoltCDetector._voting = cv::Mat::zeros(image.rows, image.cols, CV_16U);
		
		
		landoltCDetector.rasterizeLine(a,b);
		
		const uint16_t* readVoting = (const uint16_t*)landoltCDetector._voting.data;
		
		int columns = image.cols ;
		
		return readVoting[columns*y + i];
	}
    
  
  
  /** test cases **/
  
  //TEST_F(LandoltcDetectorTest, findCentersTest)
  //{
		//cv::Mat image;
		//image=cv::imread("/pandora_vision_landoltc/bold.jpg",0);
		
		//fillGrad(image);
		
		//cv::Point point(400,300);
		
		//EXPECT_EQ(point,giveCenters(0)) ;
		
		//image=cv::imread("/pandora_vision_landoltc/index.png",0);
		
		//fillGrad(image);
		
		//cv::Point point2(400,300);
		
		//EXPECT_EQ(point2,giveCenters()) ;
		
	//}
  
  
  TEST_F(LandoltcDetectorTest, rasterizeLineTest)
  {
	  cv::Point a(1,1);
		cv::Point b(2,2);
		
		EXPECT_EQ(1,giveVotingData(a, b, 2, 2));
  }	
  
  
  
   
} // namespace pandora_vision

