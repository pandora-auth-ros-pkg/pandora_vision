/*#############################################################
 * Tracker.h
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

#ifndef TRACKER_H
#define TRACKER_H
#define TRACKER_DEBUG_MODE false

#include <gsl/gsl_multifit.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

#include "trackerChain.h"
#include "thing.h"
#include "ros/ros.h"
#define L1 			1.61 * pow(10,-5)		// max	320 pixels	distance	for Threshold 0.2
#define L2 			3.85					// max	40%	area 	difference	for Threshold 0.2
#define L3 			2.77					// min	50%	area	overlap		for Threshold 0.2
#define W1 			1
#define W2 			1
#define W3 			1
#define THRESHOLD 			0.2

#define MIN_INACTIVITY 				0				 
#define MAX_INACTIVITY 				10		// 30
#define MAX_CHAIN_BUFFER			30		// maximum number of blobs, that a chain can store
#define PROBABILITY_BLOB_NUM		10		// 10, minimum number of blobs, that participate in probability calculation
#define PROBABILITY_MAX_BLOB_AGE 	20		// 70, maximum age of blobs, that participate in probability calculation

#define CENTER_REGRESSION_ERROR		0.1		// max regression error fitting blobs' centers
#define AREA_REGRESSION_ERROR		1000	// max regression error fitting blobs' area
#define OVERLAP_REGRESSION_ERROR	1000	// max regression error fitting blobs' overlap
#define MAX_AGE_BLOB_ESTIMATE		18		// maximum age of blobs, that participate in blob estimation

#define OVERLAP_DILATION_ITERATIONS	3		// number of iterations the estimated blob's img will be dilated

#define SEND_ALL	false

//struct used in CamShift Tracker		
typedef struct CamShiftThing
{
	CvRect		trackRect;
	uintptr_t	id;
	double		area;
	double 		probability;
} CamShiftThing;

class Tracker
{
	
	private:
		
		double chainId;
		int frameNum;
	
	public:
		//camShift images
		IplImage* 				csBackproject;
		IplImage* 				csImage;
		IplImage* 				csHsv;
		IplImage* 				csHue;
		IplImage* 				csMask;
		IplImage*				csAnd;
													
	
		//camshift HSV parameters
		int csVmin;
		int csVmax;
		int csSmin;
		int csSmax;
	
		std::vector<TrackerChain*> 	chainList; 		// Holds all alive trackerChains
		std::vector<Thing*>*			currentList;	// Contains Blobs that are given to Tracker as inputs
		std::vector<Thing*>*			blobList;		// Contains Blob estimates that are going to be matched at the next matching operation
		
		IplImage** 				imageBuffer;
		
		std::vector<CamShiftThing> camShiftBlobs;	//Holds the blobs being tracked with CamShift method
		
		Tracker();
		~Tracker();
		
		void track( std::vector<Thing*> *currentList, std::vector<Thing*> *destinationList, int count);			//Input list of blobs for each frame.
		
		double polyFit( CvPoint2D32f p1, 
						CvPoint2D32f p2, 
						CvPoint2D32f p3, 
						double err);									//find the center of the estimated Blob
		void findBlobEstimate();										//Estimate current in-frame position of previously matched blobs
		
		void createTrackerChains();
		void checkTrackerChains();										//Delete all dead TrackerChains from chainList, and destroy all of their Things
		
		double calculateScore(Thing* a , Thing* b);
		void calculateProbability();
		
		IplImage* camShiftTrack(IplImage* frame, IplImage* imgTexture, std::vector<Thing*>* blobs);
		
		void matchmake();												//match blobs. Return matches
		void printInfo();
		void fillImageBuffer();
};

#endif
