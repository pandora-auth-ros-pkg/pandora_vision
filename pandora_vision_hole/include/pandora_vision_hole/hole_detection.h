/*#############################################################
 * HoleFindNode.h
 *
 * File Description :
 *	ROS Node responsible for the administration of image
 *  segmentation and hole detection procedure, created for
 *  use in RoboCup RoboRescue Competition Arena 
 *	
 * Contents : 
 *
 * Author : Michael Skolarikis
 *
 * Date :	24-10-2011
 *
 *#############################################################
 */
 
#ifndef HOLEFINDNODE_H
#define  HOLEFINDNODE_H
#define DEBUG_HOLES 0

#include "ros/ros.h"
#include <ros/package.h>
#include "vision_communications/HolesDirectionsVectorMsg.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "hole_detector.h"
#include "tracker.h"

#include "state_client.h"

#include <iostream>
#include <stdlib.h>


// Field of view for our Logitech Camera, measured at the lab
#define HFOV					61.14	//horizontal field of view (degrees)
#define VFOV					48  	//vertical field of view (degrees)

#define DEFAULT_HEIGHT			480		//default frame height
#define DEFAULT_WIDTH			640		//default frame width

class HoleFindNode : public StateClient 
{
	private:

		//nodeHandle
		ros::NodeHandle 	_nh;
		HoleFinder*			_holeFinder;
		Tracker*			_tracker;
		
		float ratioX;
		float ratioY;	
		float hfov;			//horizontal Field Of View (rad)
		float vfov;			//vertical Field Of View (rad)
		int frameNum;		//current frame id
		int frameWidth;		//frame width
		int frameHeight;	//frame height
		
		std::string packagePath;
		
		IplImage*		holeFrame;					// frame processed by HoleFinder
		IplImage*		extraFrame;					// copy frame processed by HoleFinder
		
		ros::Time		holeFrameTimestamp;		// holeFinder frame timestamp

		string imageTopic;
		
		//time durations for every callback Timer in spin() function
		double holeTime;
		
		
		//publishers for HoleFinder result messages
		ros::Publisher _holesDirectionPublisher;
		
		//the subscriber that listens to the frame topic advertised by the central node
		image_transport::Subscriber _frameSubscriber;
		
		//debug publishers for HoleFinder
		image_transport::Publisher _holeSourcePublisher;
		image_transport::Publisher _holeEdgePublisher;
		image_transport::Publisher _holeThresholdPublisher;
		image_transport::Publisher _holeResultPublisher;
		image_transport::Publisher _holeTexturePublisher;
		
		//debug publishers for Tracker
		image_transport::Publisher _trackerChainPublisher1;
		image_transport::Publisher _trackerChainPublisher2;
		image_transport::Publisher _trackerChainPublisher3;
		image_transport::Publisher _trackerChainPublisher4;
		image_transport::Publisher _trackerChainPublisher5;
		
		image_transport::Publisher _camShiftPublisher;
		
		// variables for changing in dummy msg mode for debugging
		bool holeDummy;
		// variables for changing in debug mode. Publish images for debugging
		bool debugHole;
		
		//variable used for State Managing
		bool holeNowON;

		void debug_publisher(ros::Time, IplImage*, image_transport::Publisher, std::string);
		void debug_holes(cv::Mat, std::vector<Thing*>);
	public:
				
		//constructor
		HoleFindNode();
					
		//destructor			
		~HoleFindNode();	
		
		//get parameters from launch file
		void getGeneralParams();
		void getHoleParams();
		void getTrackerParams();
			
		//timer callbacks
		void holeCallback(const ros::TimerEvent&);
		void holeCallback();
			
		//get a new image
		void imageCallback(const sensor_msgs::ImageConstPtr& msg);
		
		//Implemented from StateClient
		void startTransition(int newState);
		void completeTransition(void);
		
		int curState;		//Current state of robot
		int prevState;		//Previous state of robot
};

#endif
		
		
