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
* Author: Miltiadis-Alexios Papadopoulos
*********************************************************************/

#ifndef QrCodeDetection_H
#define QrCodeDetection_H

#include <iostream>
#include <stdlib.h>
#include <opencv/cvwimage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "state_client.h"
#include <std_srvs/Empty.h>
#include "vision_communications/QRAlertsVectorMsg.h"
#include "qrCode_detector.h"

#define HFOV					61.14//68	//horizontal field of view in degrees : giwrgos 61.142
#define VFOV					48  //50	//vertical field of view in degrees : giwrgos 47.79
#define DEFAULT_HEIGHT			480			//default frame height
#define DEFAULT_WIDTH			640			//default frame width


class QrCodeDetection : public StateClient {
private:

	//nodeHandle
	ros::NodeHandle _nh;
	QrCodeDetector	_qrcodeDetector;
	float ratioX;
	float ratioY;

	float hfov;		//horizontal Field Of View (rad)
	float vfov;
	int frameWidth;		//frame width
	int frameHeight;	//frame height

	cv::Mat		qrcodeFrame;				// frame processed by MotionDetector

	ros::Time	qrcodeFrameTimestamp;		// MotionDetector frame timestamp

	std::string imageTopic;
	std::string imageTopicback;

	//publishers for QrCodeDetector result messages
	ros::Publisher _qrcodePublisher;

	//the subscriber that listens to the frame topic advertised by the central node
	image_transport::Subscriber _frameSubscriberFront;
	image_transport::Subscriber _frameSubscriberBack;

	//debug publisher for MotionDetector
	image_transport::Publisher _qrcodeDebugPublisher;

	// variables for changing in dummy msg mode for debugging
	bool qrcodeDummy;
	// variables for changing in debug mode. Publish images for debugging
	bool debugQrCode;

	//variable used for State Managing
	bool qrcodeNowON;

	//get parameters from launch file
	void getGeneralParams();
	void getQrCodeParams();

	void publish_debug_images();

	//timer callbacks
	//executes qrcode Detector when in every state
	//called my qrcodeCallback
	void qrcodeDetectAndPost(std::string frame_id);

	//get a new image
	void imageCallbackFront(const sensor_msgs::ImageConstPtr& msg);
	void imageCallbackBack(const sensor_msgs::ImageConstPtr& msg);

	int curState;		//Current state of robot
	int prevState;		//Previous state of robot
public:

	//constructor
	QrCodeDetection();

	//destructor
	virtual ~QrCodeDetection();

	//Implemented from StateClient
	void startTransition(int newState);
	void completeTransition(void);

};

#endif
