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
* Author: <Name>
*********************************************************************/
#include "pandora_vision_landoltc/landoltC_detection.h"
/**
 * Constructor
 */
LandoltCDetection::LandoltCDetection() :	_nh()
{
	

    // Get General Parameters, such as frame width & height , camera id
	getGeneralParams();
	
	
	//Convert field of view from degrees to rads
	hfov = HFOV * CV_PI / 180;
	vfov = VFOV * CV_PI / 180;

	ratioX = hfov / frameWidth;
	ratioY = vfov / frameHeight;

	//hazmatFrame = cv::Mat( frameWidth,frameHeight, IPL_DEPTH_8U, 3 );
    landoltCFrame = cv::Mat( frameWidth,frameHeight, CV_8U );
    
	
	//subscribe to input image's topic
	sub = image_transport::ImageTransport(_nh).subscribe(imageTopic, 1, &LandoltCDetection::imageCallback,this);

	//initialize states - robot starts in STATE_OFF 
	curState = state_manager_communications::robotModeMsg::MODE_OFF;
	prevState = state_manager_communications::robotModeMsg::MODE_OFF;

	//initialize state Managing Variables
	landoltCNowON 	= false;
    
    clientInitialize();
    
	ROS_INFO("[landoltCNode] : Created Hazmat Detection instance");
}

/**
 * Destructor
 */
LandoltCDetection::~LandoltCDetection()
{
	ROS_INFO("[landoltCNode] : Destroying Hazmat Detection instance");

}

/**
 * Get parameters referring to view and frame characteristics
 */
void LandoltCDetection::getGeneralParams()
{
	

	//get package path in the pc
	if (_nh.hasParam("/vision/packagepath")){
		_nh.getParam("/vision/packagepath", packagePath);
		ROS_DEBUG_STREAM("path : " << packagePath);
	}
	else
	{
		ROS_DEBUG("[landoltCNode] : Parameter path not found. Using Default");
		packagePath = ros::package::getPath("vision");
	}

	// Get the Height parameter if available;
	if (_nh.hasParam("height")) {
		_nh.getParam("height", frameHeight);
		ROS_DEBUG_STREAM("height : " << frameHeight);
	}
	else {
		ROS_DEBUG("[landoltCNode] : Parameter frameHeight not found. Using Default");
		frameHeight = DEFAULT_HEIGHT;
	}

	// Get the images's topic;
	if (_nh.hasParam("imageTopic")) {
		_nh.getParam("imageTopic", imageTopic);
		ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
	}
	else {
		ROS_DEBUG("[landoltCNode] : Parameter imageTopic not found. Using Default");
		imageTopic = "/camera_head/image_raw";
	}

	// Get the Width parameter if available;
	if (_nh.hasParam("width")) {
		_nh.getParam("width", frameWidth);
		ROS_DEBUG_STREAM("width : " << frameWidth);
	}
	else {
		ROS_DEBUG("[landoltCNode] : Parameter frameWidth not found. Using Default");
		frameWidth = DEFAULT_WIDTH;
	}
}
/**
 * Called in case of incoming ROS image message.
 * @param msg
 */
void LandoltCDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	
    cv_bridge::CvImagePtr in_msg;
	in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat temp = in_msg->image.clone();	
    landoltCFrame = new IplImage(temp);
	landoltCFrameTimestamp = msg->header.stamp;
	if ( landoltCFrame.empty() )
	{               
		ROS_ERROR("[landoltCNode] : No more Frames");
		ros::shutdown();
		return;
	}
}



/**
 * Starts state transition
 * @param newState
 */
void LandoltCDetection::startTransition(int newState){
	
    curState = newState;
    
	//check if each algorithm should be running now
	landoltCNowON		=	( curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION ) ||
			( curState == state_manager_communications::robotModeMsg::MODE_IDENTIFICATION ) ||
			( curState == state_manager_communications::robotModeMsg::MODE_ARM_APPROACH ) ||
			( curState == state_manager_communications::robotModeMsg::MODE_TELEOPERATED_LOCOMOTION ) ||
			( curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD );
			
	if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING){
		ros::shutdown();
		return;
	}

	prevState=curState;

	transitionComplete(curState);
}

/**
 * Called when state transition is completed
 */
void LandoltCDetection::completeTransition(void){
	ROS_INFO("[landoltCNode] : Transition Complete");
}

/**
 * Node's main method
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{	
	ros::init(argc,argv,"landoltCNode");

	LandoltCDetection* landoltCDetection = new LandoltCDetection();

	ros::spin();

	delete landoltCDetection;

	return 0;	
}
