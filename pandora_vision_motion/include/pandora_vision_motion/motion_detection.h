/*#############################################################
 * MotionDetection.h
 *
 * File Description :
 *	ROS Node of motion detection, vision package 
 *	
 * Contents :
 *
 * Author : Aprilis George
 *
 * Refactoring: Miltiadis-Alexios Papadopoulos
 *
 * Date :	24-10-2011
 *
 * Change History :
 *
 *#############################################################
 */
 
 #ifndef MOTIONDETECTION_H
#define  MOTIONDETECTION_H

#include "ros/ros.h"

#include "vision_communications/MotionMsg.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv/cvwimage.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

//~ #include "customStream.h"
#include "motion_detector.h"
//~ #include "TimeCalculator.h"

#include "state_client.h"

#include <iostream>
#include <stdlib.h>

//~ #include <controllers_and_sensors_communications/controlSwitchSrv.h>

//~ #include <std_srvs/Empty.h>

#define HFOV					61.14//68		//horizontal field of view in degrees : giwrgos 61.142
#define VFOV					48  //50		//vertical field of view in degrees : giwrgos 47.79
#define DEFAULT_HEIGHT			480		//default frame height
#define DEFAULT_WIDTH			640		//default frame width


class MotionDetection : public StateClient {
	private:

		//nodeHandle
		ros::NodeHandle _nh;
		MotionDetector*	_motionDetector;
		float ratioX;
		float ratioY;
		
		float hfov;		//horizontal Field Of View (rad)
		float vfov;		
		int frameWidth;		//frame width
		int frameHeight;	//frame height
		
		cv::Mat		motionFrame;				// frame processed by MotionDetector
		cv::Mat		extraFrame;					// copy frame processed by MotionDetector
		
		ros::Time		motionFrameTimestamp;		// MotionDetector frame timestamp

		string imageTopic;
		
		//publishers for MotionDetector result messages
		ros::Publisher _motionPublisher;
		
		//the subscriber that listens to the frame topic advertised by the central node
		image_transport::Subscriber _frameSubscriber;
		
		//debug publisher for MotionDetector
		image_transport::Publisher _motionDiffPublisher;
		image_transport::Publisher _motionFrmPublisher;
		
		// variables for changing in dummy msg mode for debugging
		bool motionDummy;
		// variables for changing in debug mode. Publish images for debugging
		bool debugMotion;
		
		//variable used for State Managing
		bool motionNowON;
	
		void publish_debug_images();
	public:
				
		//constructor
		MotionDetection();
					
		//destructor			
		~MotionDetection();	
		
		//get parameters from launch file
		void getGeneralParams();
		void getMotionParams();
		
		//executes motion Detector when in state STATE_ARM_SEARCH_COMPLETED
		void motionDetectAndPost();

		//get a new image
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		
		//Implemented from StateClient
		void startTransition(int newState);
		void completeTransition(void);
		
		int curState;		//Current state of robot
		int prevState;		//Previous state of robot
};

#endif
		
		
