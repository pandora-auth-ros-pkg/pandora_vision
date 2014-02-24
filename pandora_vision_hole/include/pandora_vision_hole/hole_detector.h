/*#############################################################
 * HoleFinder.h
 *
 * File Description :
 *	HoleFinder Header file
 *	
 * Contents :
 *
 * Author : 
 *
 * Date :
 *
 * Change History :
 *
 *#############################################################
 */

#ifndef HOLEFINDER_H
#define HOLEFINDER_H
#define HOLEFINDER_DEBUG_MODE false

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdlib.h>

#include "thing.h"
#include "textureFilter.h"
#include "pattern.h"

#define DEFAULT_HEIGHT 480
#define DEFAULT_WIDTH 640


class HoleFinder
{ 	
	
	private:
		bool convertToGray;
	
	public:
	
		/// Parameters
		
		//Modify Source Image Params
		int _modeA;
		int _equalizeA;
		int _smoothA;
		int _erodeA;
		int _dilateA;
		int _openA;
		int _closeA;
		
		//Edge Image Params
		int _modeB;
		
		//	mode 0 params
		int _cannyConvKernelB;
		int _cannyLowThresB;
		int _cannyHighThresB;
		int _dilateB;
		int _cannyConvKernelB1;
		int _cannyLowThresB1;
		int _cannyHighThresB1;
		
		//	mode 1 params
		int _gradientB;
		int _equalizeB;
		int _thresholdLowThresB;
		int _thresholdHighThresB;
		int _dilateB1;
		int _cannyConvKernelB2;
		int _cannyLowThresB2;
		int _cannyHighThresB2;
		
		//Threshold Image Params
		int _closeC;
		int _openC;
		int _erodeC;
		int _dilateC;
		int _thresholdLowThresC;
		int _thresholdHighThresC;
		
		//Texture Image Params
		int _smoothD;
		int _closeD;
		int _openD;
		int _erodeD;
		int _dilateD;
		int _thresholdLowThresD;
		int _thresholdHighThresD;
		
		//Contour Parameters
		int _lengthContour;
		int _areaContour;
		double _maxAreaContour;
		int _rectHeightContour;
		int _rectWidthContour;
		double _heightToWidthContour;
		double _widthToHeightContour;
		double _pixelToRectAreaContour;
		double _darkToPixelAreaContour;
		double _textureToPixelAreaContour;
		
		//Blob Parameters
		double _minFormFactor;
		double _ellipseFactorMax;
		double _ellipseFactorMin;
		double _minAxisValue;
		double _verticalAxisRatio;
		double _horizontalAxisRatio;
		double _thetaLow;
		double _thetaHigh;

		/// ------------------- ///
		
		TextureFilter* filter;
	
		CvSize size;
		cv::Mat imgSrc;
		IplImage* imgEdge;
		IplImage* imgContours;
		IplImage* imgThreshold;
		IplImage* imgTexture;
		
		CvMemStorage* strgContours;
		CvSeq* seqContours;
		CvSeq* seqCurrent;
		
		vector<Thing*> resultBlobs;
	
		int contourCounter;
		
		HoleFinder();
		~HoleFinder();
					
		void modifySourceImage(cv::Mat capturedImage);
		void findThresholdImage(cv::Mat imgInput);
		void findEdgeImage(IplImage* imgInput);
		void findTextureImage(cv::Mat capturedImage);
		void findContours(IplImage* edgeImage);
		void findBlobs(IplImage* contourImage);
		vector<Thing*> findHoles(cv::Mat sourceImage);
		
		void createWindows();
		void showImages();
		void destroyWindows();
		
		void cleanup();
		
		void setModeMod(int mode);
		void setEqualizeMod(int flag);
		void setSmoothMod(int block);
		void setDilateMod(int iterations);
		void setErodeMod(int iterations);
		void setOpenMod(int iterations);
		void setCloseMod(int iterations); 

		void setModeEdge(int mode);
		void setCannyThresEdge(int threshold1 , int threshold2);
		void setDilateEdge(int iterations);
		void setGradientEdge(int iterations);
		void setEqualizeEdge(int flag);
		void setThresholdEdge(int threshold1, int threshold2); 

		void setErodeThres(int iterations);
		void setDilateThres(int iterations);
		void setCloseThres(int iterations);
		void setOpenThres(int flag);
		void setThresholdThres(int threshold1, int threshold2); 
		
		void setMinLengthContour(int length);
		void setMinAreaContour(int area);
		void setMinRectHeightContour(int height);
		void setMinRectWidthContour(int width);
		void setHeightToWidthContour(double ratio);
		void setWidthToHeightContour(double ratio);
		void setPixelToRectAreaContour(double ratio);
		void setDarkToPixelAreaContour(double ratio);
		
		void setTexturePath(string path);
};

#endif
