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
* Author:  Tsakalis Vasilis
* 		   Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_hazmat/hazmat_detection.h"

/**
 * Constructor
 */
HazmatDetection::HazmatDetection() :	_nh()
{
	//initialize hazmat detector
	_hazmatDetector = new HazmatEpsilonDetector();

    // Get General Parameters, such as frame width & height , camera id
	getGeneralParams();
	
	//Get HazmatDetector Parameters
	getHazmatParams();

	
	//Convert field of view from degrees to rads
	hfov = HFOV * CV_PI / 180;
	vfov = VFOV * CV_PI / 180;

	ratioX = hfov / frameWidth;
	ratioY = vfov / frameHeight;

	//hazmatFrame = cv::Mat( frameWidth,frameHeight, IPL_DEPTH_8U, 3 );
    hazmatFrame = cv::Mat( frameWidth,frameHeight, CV_8U );
    
	// Declare publisher and advertise topic where algorithm results are posted
	_hazmatPublisher = _nh.advertise<vision_communications::HazmatAlertsVectorMsg>("hazmat_alert", 10);

	//subscribe to input image's topic
	sub = image_transport::ImageTransport(_nh).subscribe(imageTopic, 1, &HazmatDetection::imageCallback,this);

	//initialize states - robot starts in STATE_OFF 
	curState = state_manager_communications::robotModeMsg::MODE_OFF;
	prevState = state_manager_communications::robotModeMsg::MODE_OFF;

	//initialize state Managing Variables
	hazmatNowON 	= false;
    
    clientInitialize();
    
	ROS_INFO("[hazmatNode] : Created Hazmat Detection instance");
}

/**
 * Destructor
 */
HazmatDetection::~HazmatDetection()
{
	ROS_INFO("[hazmatNode] : Destroying Hazmat Detection instance");
	delete _hazmatDetector;

}

/**
 * Get parameters referring to view and frame characteristics
 */
void HazmatDetection::getGeneralParams()
{
	// Get the hazmatDummy parameter if available;
	if (_nh.hasParam("hazmatDummy")) {
		_nh.getParam("hazmatDummy", hazmatDummy);
		ROS_DEBUG("hazmatDummy : %d", hazmatDummy);
	}
	else {
		ROS_DEBUG("[webNode] : Parameter hazmatDummy not found. Using Default");
		hazmatDummy = false;
	}

	//get package path in the pc
	if (_nh.hasParam("/vision/packagepath")){
		_nh.getParam("/vision/packagepath", packagePath);
		ROS_DEBUG_STREAM("path : " << packagePath);
	}
	else
	{
		ROS_DEBUG("[hazmatNode] : Parameter path not found. Using Default");
		packagePath = ros::package::getPath("vision");
	}

	//get the store location of the image
	if (_nh.hasParam("saveImagePath")){
		_nh.getParam("saveImagePath", saveImagePath);
		ROS_DEBUG_STREAM("path : " << saveImagePath);
	}
	else
	{
		ROS_DEBUG("[hazmatNode] : Parameter saveImagePath not found. Using Default");
		saveImagePath = "/home/pandora/Desktop/eyeCharts/";
	}


	// Get the Height parameter if available;
	if (_nh.hasParam("height")) {
		_nh.getParam("height", frameHeight);
		ROS_DEBUG_STREAM("height : " << frameHeight);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter frameHeight not found. Using Default");
		frameHeight = DEFAULT_HEIGHT;
	}

	// Get the images's topic;
	if (_nh.hasParam("imageTopic")) {
		_nh.getParam("imageTopic", imageTopic);
		ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter imageTopic not found. Using Default");
		imageTopic = "/camera_head/image_raw";
	}

	// Get the Width parameter if available;
	if (_nh.hasParam("width")) {
		_nh.getParam("width", frameWidth);
		ROS_DEBUG_STREAM("width : " << frameWidth);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter frameWidth not found. Using Default");
		frameWidth = DEFAULT_WIDTH;
	}
}

/**
 * Get parameters referring to hazmat detection algorithm
 */
void HazmatDetection::getHazmatParams()
{	
	// Get the test parameter if available;
	int colorVariance;
	if (_nh.hasParam("colorVariance")) {
		_nh.getParam("colorVariance", colorVariance);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter colorVariance not found. Using Default");
		colorVariance = 10;
	}

	double votingThreshold;
	if (_nh.hasParam("votingThreshold")) {
		_nh.getParam("votingThreshold", votingThreshold);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter votingThreshold not found. Using Default");
		votingThreshold = 39900;
	}

	//get the minimum area threshold of the hazmat in the image
	double minAreaThreshold;
	if (_nh.hasParam("minAreaThreshold")) {
		_nh.getParam("minAreaThreshold", minAreaThreshold);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter minAreaThreshold not found. Using Default");
		minAreaThreshold = 1000;
	}

	//get the maximum area threshold of the hazmat in the image
	double maxAreaThreshold;
	if (_nh.hasParam("maxAreaThreshold")) {
		_nh.getParam("maxAreaThreshold", maxAreaThreshold);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter maxAreaThreshold not found. Using Default");
		maxAreaThreshold = 100000;
	}

	//get the sidelenght parameter of the rectangle in which to test for colour
	int sideLength;
	if (_nh.hasParam("sideLength")) {
		_nh.getParam("sideLength", sideLength);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter sideLength not found. Using Default");
		//sideLength = 200
		sideLength = 100;
	}

	//get the minimum number of features threshold
	int featureThreshold;
	if (_nh.hasParam("featureThreshold")) {
		_nh.getParam("featureThreshold", featureThreshold);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter featureThreshold not found. Using Default");
		featureThreshold = 20;
	}

	//how many hazmats i have to search for
	int hazmatNumber;
	if (_nh.hasParam("hazmatNumber")) {
		_nh.getParam("hazmatNumber", hazmatNumber);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter hazmatNumber not found. Using Default");
		hazmatNumber = 9;
	}

	//get the MO threshold
	double MOThreshold;
	if (_nh.hasParam("MOThreshold")) {
		_nh.getParam("MOThreshold", MOThreshold);
	}
	else {
		ROS_DEBUG("[hazmatNode] : Parameter MOThreshold not found. Using Default");
		MOThreshold = 120000;
	}

	_hazmatDetector->setHazmatParameters(colorVariance,(float)votingThreshold,(float)minAreaThreshold,(float)maxAreaThreshold,sideLength,featureThreshold,(float)MOThreshold);

}

/**
 * Called in case of incoming ROS image message.
 * @param msg
 */
void HazmatDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//update image contents
//	sensor_msgs::CvBridge bridge;
//	hazmatFrame = bridge.imgMsgToCv(msg, "bgr8");
//	hazmatFrameTimestamp = msg->header.stamp;
    cv_bridge::CvImagePtr in_msg;
	in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat temp = in_msg->image.clone();	
    hazmatFrame = new IplImage(temp);
	hazmatFrameTimestamp = msg->header.stamp;
	if ( hazmatFrame.empty() )
	{               
		ROS_ERROR("[hazmatNode] : No more Frames");
		ros::shutdown();
		return;
	}
	hazmatCallback();
}

/**
 * Method called only when a new image message is present
 */
void HazmatDetection::hazmatCallback()
{
     cv::Mat allblack = cv::Mat( frameWidth,frameHeight, CV_8U );
	if(!hazmatNowON)
	{
		return;
	}
	 if(std::equal (hazmatFrame.begin<uchar>(), hazmatFrame.end<uchar>(), allblack.begin<uchar>() ))
	{
		return;
	}
	// Create Msg for hazmat
	vision_communications::HazmatAlertsVectorMsg hazmatVectorMsg;
	vision_communications::HazmatAlertMsg hazmatMsg;

	if (hazmatDummy)
	{
		/*
		 * Dummy Hazmat Message
		 */
		hazmatVectorMsg.header.frame_id="Hazmat";
		hazmatVectorMsg.header.stamp = ros::Time::now();
		for (int i=0 ; i < 3 ; i++)
		{
			hazmatMsg.yaw = 0;
			hazmatMsg.pitch = 0;
			hazmatMsg.patternType = 0;

			hazmatVectorMsg.hazmatAlerts.push_back(hazmatMsg);
		}	
		if(hazmatVectorMsg.hazmatAlerts.size()>0)_hazmatPublisher.publish(hazmatVectorMsg);

		//dummy delay
		usleep(1000 * 2000);
	}
	else
	{
		//
		// Hazmat Message
		//

		// run hazmat detector
	
		vector<HazmatEpsilon> a = _hazmatDetector->DetectHazmatEpsilon(hazmatFrame);

		//if hazmat found
		if (a.size() > 0)
		{
			hazmatVectorMsg.header.frame_id="headCamera";
			hazmatVectorMsg.header.stamp = hazmatFrameTimestamp;
			for (unsigned int i=0; i < a.size(); i++)
			{
				//hazmat message information
				hazmatMsg.yaw = ratioX * ( a[i].x - frameWidth/2 );
				hazmatMsg.pitch = -ratioY * ( a[i].y - frameHeight/2 );
				hazmatMsg.patternType = a[i].pattern_num;
				//add the message to vector
				hazmatVectorMsg.hazmatAlerts.push_back(hazmatMsg);
              
                   ROS_INFO("[hazmatNode] : Hazmat found!");
				//check if eye chart
				if (  (a[i].pattern_num >hazmatNumber) )
				{			
					std::stringstream ss;
					//save Image to the desired location
					ss<<saveImagePath<<hazmatFrameTimestamp<<".jpg";
					imwrite(ss.str().c_str(),hazmatFrame);
					//cvSaveImage(ss.str().c_str(),hazmatFrame);
				}
			}
			if(hazmatVectorMsg.hazmatAlerts.size()>0)_hazmatPublisher.publish(hazmatVectorMsg);

		}
		a.erase(a.begin(),a.end());
	}
}	

/**
 * ROS spin function
 */
void HazmatDetection::spin()
{
	ros::spin();
}

/**
 * Starts state transition
 * @param newState
 */
void HazmatDetection::startTransition(int newState){
	
    curState = newState;
    
	//check if each algorithm should be running now
	hazmatNowON		=	( curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION ) ||
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
void HazmatDetection::completeTransition(void){
	ROS_INFO("[hazmatNode] : Transition Complete");
}

/**
 * Node's main method
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char** argv)
{	
	ros::init(argc,argv,"hazmatNode");

	HazmatDetection* hazmatDetection = new HazmatDetection();

	hazmatDetection->spin();

	delete hazmatDetection;

	return 0;	
}

