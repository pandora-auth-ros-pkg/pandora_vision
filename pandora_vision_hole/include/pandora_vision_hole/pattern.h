/*#############################################################
 * Pattern.h
 *
 * File Description :
 *	Pattern Header file
 *	
 * Contents :
 *
 * Author : Michael Skolarikis
 *
 * Date :
 *
 * Change History : 11.05.11
 *
 *#############################################################
 */

#ifndef PATTERN_H
#define PATTERN_H

#include <stdint.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"

#define POSITIVE 0
#define NEGATIVE 1

#define	BGR						0
#define YCrCb					1
#define CrCb					2

#define EUCLIDEAN				0
#define EUCLIDEAN_SIMPLE		1

using namespace cv;

struct histogram 
{
	int* values;
	double* valuesNorm;
	int bins;
	double norm2;	// norm ^ 2
};

class Pattern
{
	
	private:
				
																			
	public:
		
		uintptr_t m_id;
		
		histogram m_hist;
	
		IplImage* m_imgPatternGray;
		IplImage* m_imgPatternBGR;
		IplImage* m_imgPatternYCrCb;
		
		bool isPositive;
	
		Pattern();		
		~Pattern();
		
		void createPattern(IplImage* img, int type);
		void destroyPattern();
		void calculateHistogram(CvPoint3D32f* colors3d, int bins, int colorspace);
		void calculateHistogram(Mat_<Vec2f>* colors2d, int bins, int colorspace);
		void visualizeHistogram(CvPoint3D32f* colors3d);
		void visualizeHistogram(Mat_<Vec2f>* colors2d);
		
		double calculatePointDistance(CvPoint3D32f pt1, CvPoint3D32f pt2, int type);
		double calculatePointDistance(CvPoint2D32f pt1, CvPoint2D32f pt2, int type);
		
		CvScalar BGR2YCrCb(CvScalar bgr);
		CvScalar YCrCb2BGR(CvScalar ycrcb);
		
};

#endif
