/*#############################################################
 * Thing.h
 *
 * File Description :
 *	Thing Header file
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

#ifndef THING_H
#define THING_H

#include <stdint.h>

#include <cv.h>
#include <highgui.h>

#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
class Thing
{

private:

	//CvMemStorage* 	m_storage;
	//CvMemStorage* 	m_storage_cch;

	//CvMoments moments;
	//CvHuMoments hu_moments;
	cv::Moments moments;
	double hu_moments[7];

	// Spatial Moments
	double m00;
	double m01;
	double m10;
	double m11;
	double m20;
	double m02;

	// Central Moments
	double mu20;
	double mu11;
	double mu02;
	double mu30;
	double mu21;
	double mu12;
	double mu03;

	struct pattern {
		bool isPattern;
		bool isPositive;
	} m_pattern;

	float m_chainHist[8][8];

public:

	Thing();
	~Thing();

	void createThing(IplImage* img , CvContour* contour);
	void createThing(cv::Mat img , std::vector<cv::Point> contour);
	void destroyThing();

	void cloneToThing(Thing* destination);
	void setPattern(int type);
	bool isPattern();
	bool isPatternPositive();

	void calculateChainCode();
	void calculateHuMoments();

	void printInfo();

	bool m_active;			//if the blob was observed in the last frame

	uintptr_t m_id;				//Object's pointer for id
	uintptr_t m_chainId;			//Label of a matched Blob. Id of chain, the blob belongs to.
	int frameId;			//Frame id that blob was observed. If frameId < 0, then Thing is not observed, but estimated

	double probability;		//Probability of blob

	double area;			//Pixel Area
	double perimeter;		//Perimeter
	double maj_axis;		//Major-Axis
	double min_axis;		//Minor-Axis
	double eccentricity;	//Eccentricity of ellipse, 0 for circle
	double ellipseArea;		//Area of ellipse
	double theta;			//Angle between major axis and horizontal axis
	double formFactor;		//1 for perfectly circular objects
	CvRect m_rect;			//bounding rect
	CvPoint2D32f center;	//Center of Gravity

	IplImage* 	m_imgBlob;
	//CvContour*	m_contour;
	std::vector<cv::Point>	m_contour;

	CvChain*	m_freemanChain;

	CvPoint2D32f matched_center;	//Center of Gravity of the estimated, matched with this Thing, Blob. used only for visualization purposes

};

#endif
