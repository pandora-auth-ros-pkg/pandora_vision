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

#ifndef FACEDETECTION_H
#define FACEDETECTION_H

#include <iostream>
#include <stdlib.h>
#include <string>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "vision_communications/FaceDirectionMsg.h"
#include "face_detector.h"
#include "time_calculator.h"
#include "state_client.h"

#define HFOV					61.14//68		//horizontal field of view in degrees : giwrgos 61.142
#define VFOV					48  //50		//vertical field of view in degrees : giwrgos 47.79
#define DEFAULT_HEIGHT			480		//default frame height
#define DEFAULT_WIDTH			640		//default frame width

namespace pandora_vision 
{
  class FaceDetection : public StateClient 
  {	private:
      
      //nodeHandle
      ros::NodeHandle _nh;
      FaceDetector*	_faceDetector;
      float ratioX;
      float ratioY;
      
      float hfov;		//horizontal Field Of View (rad)
      float vfov;		
      int frameWidth;		//frame width
      int frameHeight;	//frame height
      
      cv::Mat		faceFrame;					// frame processed by FaceDetector
      cv::Mat		extraFrame;					// copy frame processed by FaceDetector
      
      ros::Time		faceFrameTimestamp;			// FaceDetector frame timestamp
      ros::Timer		faceTimer;                              // timer for frame callback

      std::string imageTopic;
      bool isFaceFrameUpdated;	
      
      //time durations for every callback Timer in spin() function
      double faceDenseTime;
      double faceSparseTime;
      
      //Client used for testing ---by Management Team---
      ros::ServiceClient faceClient;
      
      //publishers for FaceDetector result messages
      ros::Publisher _victimDirectionPublisher;
      
      //the subscriber that listens to the frame topic advertised by the central node
      image_transport::Subscriber _frameSubscriber;
      
      //debug publishers for FaceDetector
      image_transport::Publisher _facePrevPublisher;
      image_transport::Publisher _faceNowPublisher;
      image_transport::Publisher _faceSkinPublisher;
      
      // variables for changing in dummy msg mode for debugging
      bool faceDummy;
      // variables for changing in debug mode. Publish images for debugging
      bool debugFace;
      
      //variable used for State Managing
      bool faceNowON;
      
      //mutex lock needed to prevent conflicts between
      //updating face frame and using it for face detection
      pthread_mutex_t	faceLock;
      
      //parameters for the FaceDetector:
      std::string cascadeName;
      std::string model_path;
      int bufferSize;
      bool skinEnabled; 
      double scaleFactor; 
      int mn;
      int minFaceDim;
      
      //paths for Skin Detector
      std::string skinHist;
      std::string wallHist;
      std::string wall2Hist;
      std::string packagePath;
    
    public:
          
      //constructor
      FaceDetection();
            
      //destructor			
      ~FaceDetection();	
      
      //get parameters from launch file
      void getGeneralParams();
      void getFaceParams();
      void getTimerParams();
      
      //timer callbacks
      void faceCallback(const ros::TimerEvent&);
      
      //get a new image
      void imageCallback(const sensor_msgs::ImageConstPtr& msg);
      
      void createFaceMessage(vision_communications::FaceDirectionMsg &faceMessage);
      void createDummyFaceMessage(float &center_x, float &center_y, vision_communications::FaceDirectionMsg &faceMessage);
      //Implemented from StateClient
      void startTransition(int newState);
      void completeTransition(void);
      
      int curState;		//Current state of robot
      int prevState;		//Previous state of robot
  };
}
#endif
		
		
