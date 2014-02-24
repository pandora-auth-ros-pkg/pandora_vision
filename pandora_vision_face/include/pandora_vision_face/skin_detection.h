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

#ifndef SKINDETECTION_H
#define  SKINDETECTION_H
#include <iostream>
#include <stdlib.h>
#include <pthread.h>
#include "ros/ros.h"
#include <opencv/cvwimage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include "StateClient.h"
#include "vision_communications/victimIdentificationDirectionMsg.h"
#include "skin_detector.h"
#include "time_calculator.h"

#define HFOV					61.14//68		//horizontal field of view in degrees : giwrgos 61.142
#define VFOV					48  //50		//vertical field of view in degrees : giwrgos 47.79
#define DEFAULT_HEIGHT			480		//default frame height
#define DEFAULT_WIDTH			640		//default frame width

class SkinDetection : public StateClient {
	private:

		//nodeHandle
		ros::NodeHandle _nh;
		SkinDetector*	_skinDetector;
		float ratioX;
		float ratioY;
		
		float hfov;		//horizontal Field Of View (rad)
		float vfov;		
		int frameWidth;		//frame width
		int frameHeight;	//frame height
		
		cv::Mat		skinFrame;				// frame processed by SkinDetector
		cv::Mat		extraFrame;				// frame processed by SkinDetector
		
		ros::Time		skinFrameTimestamp;		// SkinDetector frame timestamp
		ros::Timer		skinTimer;				// Timer for frame callback

		string imageTopic;
		
		//time durations for every callback Timer in spin() function
		double skinTime;
		
		ros::Publisher _victimDirectionPublisher;
		
		//the subscriber that listens to the frame topic advertised by the central node
		image_transport::Subscriber _frameSubscriber;
		
		//debug publishers for SkinDetector
		image_transport::Publisher _skinSourcePublisher;
		image_transport::Publisher _skinResultPublisher;
		
		// variables for changing in dummy msg mode for debugging
		bool skinDummy;
		// variables for changing in debug mode. Publish images for debugging
		bool debugSkin;
		
		//variable used for State Managing
		bool skinNowON;

		//mutex lock needed to prevent conflicts between
		//updating skin frame and using it for skin detection
		pthread_mutex_t	skinLock;

		//service for counting  messages sent to dataFusion
	
	public:
				
		//constructor
		SkinDetection();
					
		//destructor			
		~SkinDetection();	
		
		//get parameters from launch file
		void getGeneralParams();
		void getSkinParams();
		void getTimerParams();
		
		
		//timer callbacks
		//executes skin Detector when in state STATE_ARM_SEARCH_COMPLETED
		void skinCallback(const ros::TimerEvent&);
		
		//get a new image
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		
		void spin();
		
		//Implemented from StateClient
		void startTransition(int newState);
		void completeTransition(void);
		
		int curState;		//Current state of robot
		int prevState;		//Previous state of robot
};

#endif
		
		
