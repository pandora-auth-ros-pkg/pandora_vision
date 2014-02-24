/////////////////////////////////////////////////////////////////
// Filename: classMotionDetector.h								/
// Author:   George Aprilis										/
// Refactoring: Miltiadis-Alexios Papadopoulos					/
// Date:     05.03.2011 20:04:13								/
//																/
// Description: class for Implementation of Motion Detection	/
//              using thresholding of absolute difference		/
//              between a sequence of frames. Returns 3 states:	/
//				Extensive Movement, Slight Movement,and No Move-/
//				ment at all.									/
/////////////////////////////////////////////////////////////////

#ifndef MOTIONDETECTOR_H
#define MOTIONDETECTOR_H

#include <opencv2/opencv.hpp>
#include <math.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>

using namespace std;

class MotionDetector
{
public:
	int			N;					//buffer size
	int 		diff_threshold;		//threshold between pixel (grayscale) values to be considered "different" between 2 frames
	double 		motion_high_thres;	//evaluation threshold: higher value means a lot of movement
	double 		motion_low_thres;	//evaluation threshold: higher value means a little movement - less means no movement at all

private:
	int 		flagCounter;		//Used to avoid wrong results in the first calculations
	int 		last;				//Last frame stored in buffer
	int 		count;				//Sum of different pixels between two frames (after thresholding)
	//due to empty initial frames in buffer
	cv::Mat  	*buf; 				//N-sized buffer
	cv::Mat 	tmp;				//Temporary copy of frame
	cv::Mat 	dif; 				//Image with the difference between 2 frames' pixels' values

public:
	MotionDetector();
	~MotionDetector();
	int 		detectMotion(cv::Mat frame);	//the main Motion Detector function
	int 		getCount();
	void 		resetFlagCounter();	//
	cv::Mat		getDiffImg();
};

#endif
