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
* Author: Aprilis George
* 		  Despoina Paschalidou
*********************************************************************/

#ifndef FACEDETECTOR_H
#define FACEDETECTOR_H

#include <math.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "skin_detector.h"

namespace pandora_vision 
{
  
  class FaceDetector
  {
    private:
    
      SkinDetector* skinDetector;
      CvMemStorage* storage_total;

      std::vector<std::vector<cv::Rect_<int> > > faces;
      
      std::vector<cv::Rect_<int> > faces_total;
      
      cv::CascadeClassifier	cascade;	
          
      std::string cascade_name;
      std::string model_path_name;
      
      //parameters used in cvHaarDetectObjects functions:
      double scale;
      int	minNeighbors;
      cv::Size minFaceSize;
      cv::Size maxFaceSize; 
      
      cv::Mat* dstArray;
      
      //!< Struct used to pass multiple parameters in different threads		
      typedef struct
        {
          cv::Mat	frame;
          float	angle;
          double scale;
          int	minNeighbors;
          cv::Size minFaceSize;
          FaceDetector* thisObj;
          cv::Mat	dst;
          int	retVal;
          cv::CascadeClassifier	cascade;
        } ThParams;
      
      //!< If enabled SkinDetector output, is also considered in
      //!< probability value
      bool isSkinDetectorEnabled;	 
      
      int	_bufferSize; 
      
      //!< Number of rotation angles						
      int angleNum;			
      
      int	now, prev; 				//circular buffer iterators
      float	probability;			//total probability of face found
      
      //!< Image buffer used to store frames
      std::vector<cv::Mat> frame_buffer;
      
      //!< Vector with partial probabilities that are used to
      //!< calculate total probability of face according to 
      //!< consistency in last _bufferSize frames
      std::vector<float> probability_buffer;	
      
      //debug images:
      cv::Mat 					skinImg;
      cv::Mat 					faceNow;
      cv::Mat 					facePrev;
      //These vectors hold the images and corresponding labels:
      std::vector<cv::Mat> images;
      std::vector<int> labels;
      int imageWidth;
      int imageHeight;
      image_transport::Publisher _facePublisher;	
      cv::Ptr<cv::FaceRecognizer> model;
      
      /**
        @brief Initializes frame and probability buffer
        @param image [cv::Mat] The current frame
        @return void
      */
      void initFrameProbBuffers(cv::Mat frame);
      
    public:
      //debug switch - webNode changes it externally:
      bool						isDebugMode;
      
      //!< The Constructor
      FaceDetector(std::string cascadePath,std::string model_path, int bufferSize, bool skinEnabled, double scaleFactor, std::string skinHist, std::string wallHist, std::string wall2Hist);
      
      //!< The Destructor
      virtual ~FaceDetector();
      
      /**
        @brief Searches for faces in current frame.
        @param image [cv::Mat] The  current frame
        @return number [int] of faces found in current frame
      **/
      int findFaces(cv::Mat frame );			
      
      int	findFaces1Frame(cv::Mat frameIN );	//Normal Implementation, probability of each face equals 1
      int* 	getFaceRectTable();
      int	getFaceRectTableSize();
      float	getProbability();
      cv::Mat getFaceNow();		//Debug image getter
      cv::Mat getFacePrev();	//Debug image getter
      cv::Mat getFaceSkin();	//Debug image getter

    private:
      
      static void* 	threadRotateThenDetect( void* arg );	//the actual job being done by each different thread
      int detectFace(cv::Mat img,cv::CascadeClassifier cascade, float angle);
      
      /**
        @brief Rotates input frame according to the given angle
        @param frame [cv::Mat] the frame to be rotated.
        @param thAngle [int] angle in degrees (angle>=0)
          any angle more than 360 degrees is reduced to a primary circle 
          angle.
        @param	rotMatData pointer to the data of the rotation
          matrix values produces for this rotation (this function feels the values)
        @return the frame rotated
      */
      cv::Mat frameRotate( cv::Mat frame, float angle);	
      
      
      void 			createRectangles(cv::Mat tmp);
      void 			compareWithSkinDetector(float &probability, cv::Mat tmp, int &totalArea);
      
  };
}
#endif
