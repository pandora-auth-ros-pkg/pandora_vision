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
* Author:  Skartados Evangelos
*********************************************************************/

#ifndef SKINDETECTOR_H
#define SKINDETECTOR_H

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <cstdlib>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>

class SkinDetector
{ 

	public:
	
		cv::Mat imgSrc;
		cv::Mat imgHistogrammSkin;
		cv::Mat imgHistogrammWall;
		cv::Mat imgHistogrammWall2;
		cv::Mat imgThreshold;
		cv::Mat imgThresholdFiltered;
		cv::Mat imgContours;
		
		std::vector< std::vector <cv::Point> > Contour;
		
		cv::Point* contourCenter;
		
		int contourCounter;
		
		float* contourProbability;
		int* contourSize;
		
		int sizeThreshold;
		
		float imageHeight;
		float imageWidth;
		
		SkinDetector(std::string skinHist, std::string wallHist, std::string wall2Hist );
		~SkinDetector();
		
		cv::Mat getImgContoursForFace();
		
		void init();
		int detectSkin(cv::Mat imgInput);
		void deallocateMemory();
        void createCalculationImages(cv::Mat imgInput);
        bool histogrammsLoadedCorrectly();
        void getCalculationParams(int& stepSrc, uchar* &dataSrc ,int& channels, 
                                           int& stepThreshold, uchar* &dataHistogrammSkin, int& stepHistogrammSkin, 
                                           uchar* &dataHistogrammWall, int& stepHistogrammWall, uchar* &dataHistogrammWall2, 
                                           int& stepHistogrammWall2, uchar* &dataContours, int& stepContours);
        void scanForSkin(int stepSrc, uchar* dataSrc ,int channels, 
                                 int stepThreshold, uchar* dataHistogrammSkin, int stepHistogrammSkin, 
                                 uchar* dataHistogrammWall, int stepHistogrammWall, uchar* dataHistogrammWall2, int stepHistogrammWall2);
        void calculatePropability(uchar *dataContours, int stepContours);
                //void toMat(IplImage* source, cv::Mat& dest);
                
};

#endif


