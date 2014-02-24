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
* Author: Skartados Evangelos	
*********************************************************************/

#include "pandora_vision_face/skin_detection.h"
 
 using namespace std;
 
 //constructor
SkinDetection::SkinDetection() :	_nh(),
{
	 //initialize skin detector 
	 _skinDetector		=	new SkinDetector();
	 
	//Get Skin Detector Paths for images
	getSkinPaths();
	 
	//Get Skin Detector Parameters
	getSkinParams();
	
	// Get General Parameters, such as frame width & height , camera id
	getGeneralParams();
	
	//memory will be allocated in the imageCallback
	skinFrame = 0;
	
	//Declare publisher and advertise topic where algorithm results are posted
	_victimDirectionPublisher = _nh.advertise<vision_communications::victimIdentificationDirectionMsg>("victimDirection", 10);
	
	//Advertise topics for debugging if we are in debug mode
	if (debugSkin)
	{
		_skinSourcePublisher = image_transport::ImageTransport(_nh).advertise("skinSource", 1);
		_skinResultPublisher = image_transport::ImageTransport(_nh).advertise("skinResult", 1);
	}
	
	//subscribe to input image's topic
	//image_transport::ImageTransport it(_nh);
	std::string transport = "theora";
	_frameSubscriber = image_transport::ImageTransport(_nh).subscribe(imageTopic , 1, &SkinDetection::imageCallback, this );
	
	//initialize states - robot starts in STATE_OFF 
	curState = state_manager_communications::robotModeMsg::MODE_OFF;
	prevState = state_manager_communications::robotModeMsg::MODE_OFF;
	
	//Initialize mutex lock
	skinLock = PTHREAD_MUTEX_INITIALIZER;
	
	//initialize state Managing Variables
	skinNowON 	= false;
	
	//initialize flag used to sync the callbacks
	isSkinFrameUpdated = false;
	
	ROS_INFO("[SkinNode] : Created Skin Detection instance");
}

SkinDetection::~SkinDetection()
{
	ROS_INFO("[SkinNode] : Destroying Skin Detection instance");
	
	pthread_mutex_destroy(&skinLock);
	
	delete _skinDetector;
	cvReleaseImage( &skinFrame);
	
}

//***************************************//
//         Get the timer parameters      //
//***************************************//
void SkinDetection::getTimerParams()
{
	// Get the SkinTime parameter if available
	if (_nh.hasParam("skinTime")) {
		_nh.getParam("skinTime", skinTime);
		ROS_DEBUG_STREAM("skinTime : " << skinTime);
	}
	else {
		ROS_DEBUG("[SkinNode] : Parameter skinTime not found. Using Default");
		skinTime = 0.05;
	}
}

//**************************************//
// 	Get parameters referring to view	// 
//		and frame characteristics		//
//**************************************//
void SkinDetection::getGeneralParams()
{
	// Get the skinDummy parameter if available;
	if (_nh.hasParam("skinDummy")) {
		_nh.getParam("skinDummy", skinDummy);
		ROS_DEBUG_STREAM("skinDummy: %d", skinDummy);
	}
	else {
		ROS_DEBUG("[SkinNode] : Parameter skinDummy not found. Using Default");
		skinDummy = false;
	}
	
	// Get the debugSkin parameter if available;
	if (_nh.hasParam("debugSkin")) {
		_nh.getParam("debugSkin", debugSkin);
		ROS_DEBUG_STREAM("debugSkin : " << debugSkin);
	}
	else {
		ROS_DEBUG("[SkinNode] : Parameter debugSkin not found. Using Default");
		debugSkin = true;
	}
	
	// Get the Height parameter if available;
	if (_nh.hasParam("height")) {
		_nh.getParam("height", frameHeight);
		ROS_DEBUG_STREAM("height : " << frameHeight);
	}
	else {
		ROS_DEBUG("[SkinNode] : Parameter frameHeight not found. Using Default");
		frameHeight = DEFAULT_HEIGHT;
	}
	
	// Get the listener's topic;
	if (_nh.hasParam("imageTopic")) {
		_nh.getParam("imageTopic", imageTopic);
		ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
	}
	else {
		ROS_DEBUG("[SkinNode] : Parameter imageTopic not found. Using Default");
		imageTopic = "/vision/image";
	}
	
	// Get the Width parameter if available;
	if (_nh.hasParam("width")) {
		_nh.getParam("width", frameWidth);
		ROS_DEBUG_STREAM("width : " << frameWidth);
	}
	else {
		ROS_DEBUG("[SkinNode] : Parameter frameWidth not found. Using Default");
		frameWidth = DEFAULT_WIDTH;
	}
}

//**************************************//
// 		Get parameters referring to 	// 
//		skin detection algorithm 		//
//**************************************//
void WebVision::getSkinParams()
{	
	// Get skin parameters if available;
	if (_nh.hasParam("sizeThreshold")) {
		_nh.getParam("sizeThreshold", _skinDetector->sizeThreshold);
		ROS_DEBUG_STREAM("sizeThreshold : " << _skinDetector->sizeThreshold);
	}
	else {
		ROS_DEBUG("[webNode] : Parameter sizeThreshold not found. Using Default");
		_skinDetector->sizeThreshold = 300;
	}
}

//**************************************//
// 		       Get paths for			// 
//		skin detection algorithm 		//
//**************************************//
void WebVision::getSkinPaths()
{
	// Get line color parameters if available;
	
	if (_nh.hasParam("skinHist")) {
		_nh.getParam("skinHist", skinHist);
		ROS_DEBUG_STREAM("skinHist : " << skinHist);
	}
	else {
		ROS_DEBUG("[webNode] : Parameter skinHist not found. Using Default");
		skinHist = "/home/despoina/pandora/pandora_ros_pkgs/pandora_robot/src/vision/data/histogramms/histogramm_skin.jpg";
	}
	
	if (_nh.hasParam("wallHist")) {
		_nh.getParam("wallHist", wallHist);
		ROS_DEBUG_STREAM("wallHist : " << wallHist);
	}
	else {
		ROS_DEBUG("[webNode] : Parameter wallHist not found. Using Default");
		redHist = "/home/despoina/pandora/pandora_ros_pkgs/pandora_robot/src/vision/data/histogramms/histogramm_wall.jpg";
	}
	
	if (_nh.hasParam("wall2Hist")) {
		_nh.getParam("wall2Hist", wall2Hist);
		ROS_DEBUG_STREAM("wall2Hist : " << wall2Hist);
	}
	else {
		ROS_DEBUG("[webNode] : Parameter wall2Hist not found. Using Default");
		redHist = "/home/despoina/pandora/pandora_ros_pkgs/pandora_robot/src/vision/data/histogramms/histogramm_wall2.jpg";
	}
		
}

//**************************************//
//      The Image callback function     //
//**************************************//
void SkinDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	int res = -1;
	
	cv_bridge::CvImagePtr in_msg;
	in_msg = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	skinFrame= in_msg->image.clone();
	skinFrameTimestamp = msg->header.stamp;
		
	if (!skinFrame.empty())			
	{               
		ROS_ERROR("[skinNode] : No more Frames or something went wrong with bag file");
		ros::shutdown();
		return;
	}
	
	//try to lock skin variables
	res = pthread_mutex_trylock(&skinLock);
	
	//if lock was successful: update frame, set boolean and unlock
	if(res == 0)
	{
		//memcpy( skinFrame->imageData, skinFrame->imageSize );
		memcpy( (uchar *)skinFrame.data(), skinFrame.height *skinFrame.step);
		isSkinFrameUpdated = true;
		pthread_mutex_unlock(&skinLock);
	}

}

//**************************************//
//      The callback function           //
//**************************************// 
void SkinDetection::skinCallback(const ros::TimerEvent&)
{	
	
	if(!skinNowON)
	{
		return;
	}
	
	int retries = 0;
	while (true)
	{
		pthread_mutex_lock(&skinLock);
		if (isSkinFrameUpdated)
			break;
		else
		{
			//if input frame is not yet set
			//sleep for a while to let
			//imageCallback()  to catch up
			pthread_mutex_unlock(&skinLock);
			
			usleep(50 * 1000);
			
			if (retries > 10)
			{
				ROS_INFO("[skinNode] : Timed out waiting for skinFrame!");
				return;
			}
			retries++;
		}
	}
	
	// create message of Skin Detector
	vision_communications::victimIdentificationDirectionMsg skinMessage;
	
	//Hold the center coordinates of skin blob
	float center_x = -1;
	float center_y = -1;
	
	if (skinDummy)
	{
		/*
		 * Dummy Skin Message
		*/
		for (int i=0 ; i < 3 ; i++)
		{
			center_x = ratioX * ( 600 - frameWidth/2 );
			center_y = -1 * ratioY * ( 400 + frameHeight/2 );
			
			skinMessage.x = center_x;
			skinMessage.y = center_y;
			skinMessage.probability = 1;
			skinMessage.area = 1500;
			skinMessage.header.frame_id="Skin";
			skinMessage.type = vision_communications::victimIdentificationDirectionMsg::SKIN;
			skinMessage.header.stamp = ros::Time::now();
			_victimDirectionPublisher.publish(skinMessage);
		}	
		
		//dummy delay
		usleep(1000 * 70);
	}
	else
	{
		/*
		 * Skin Message
		*/
		_skinDetector->init();
		int x;
		x = _skinDetector->detectSkin(holeFrame);
		if ( x == 1 )
		{
			ROS_ERROR( "histogramm images could not be loaded" );
			return;
		}
		int skinNum = _skinDetector->contourCounter;
		//Send a message for every skin blob in the frame
		for (int i=0 ; i < skinNum ; i++)
		{
			center_x = ratioX * ( _skinDetector->contourCenter[i].x - frameWidth/2 );
			center_y = -ratioY * ( _skinDetector->contourCenter[i].y - frameHeight/2 );
			
			skinMessage.x = center_x;
			skinMessage.y = center_y;
			skinMessage.probability = _skinDetector->contourProbability[i];
			skinMessage.area = _skinDetector->contourSize[i];
			skinMessage.header.frame_id = "headCamera";
			skinMessage.type = vision_communications::victimIdentificationDirectionMsg::SKIN;
			skinMessage.header.stamp = ros::Time::now();
			_victimDirectionPublisher.publish(skinMessage);
			
			//Added for counting the messages sent to dataFusion (for diagnostics)
			//~ skinCounter.request.count = 1;
			//~ if (!skinClient.call(skinCounter))
				//~ ROS_ERROR("Message for skin detection was sent, but could not increment counter");
		}
		
		if (debugSkin){
			cv::WImageBuffer3_b skinSource;
			skinSource.SetIpl( (IplImage*)cvClone( _skinDetector->imgSrc ) );
			sensor_msgs::ImagePtr msgSkinSource = sensor_msgs::CvBridge::cvToImgMsg(skinSource.Ipl() , "bgr8");
			_skinSourcePublisher.publish(msgSkinSource);
			
			cv::WImageBuffer3_b skinResult;
			skinResult.SetIpl( (IplImage*)cvClone( _skinDetector->imgContours ) );
			sensor_msgs::ImagePtr msgSkinResult = sensor_msgs::CvBridge::cvToImgMsg(skinResult.Ipl() , "mono8");
			_skinResultPublisher.publish(msgSkinResult);			
		}		
		_skinDetector->deallocateMemory();
	}		
	
	
	//reset the flag
	isSkinFrameUpdated = false;	
	//and unlock before leaving
	pthread_mutex_unlock(&skinLock);
}

void SkinDetection::spin()
{
	getTimerParams();
	skinTimer = _nh.createTimer(ros::Duration(skinTime), &SkinDetection::skinCallback , this);

	skinTimer.start();
	ros::spin();
}

void SkinDetection::startTransition(int newState)
{
	curState = newState;

	//check if skin algorithm should be running now
	skinNowON		=	( curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) ||
			( curState == state_manager_communications::robotModeMsg::MODE_ARM_APPROACH ) ||
			( curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD );
	
	//everytime state changes, Skin Detector needs to be reset so that it will
	//discard frames from previous calls in buffer.
	if(skinNowON){
		_skinDetector->resetFlagCounter();
	}
	
		//shutdown if the robot is switched off
	if (curState == tate_manager_communications::robotModeMsg::MODE_TERMINATING){
		ros::shutdown();
		return;
	}
	
	prevState=curState;

	transitionComplete(curState); //this needs to be called everytime a node finishes transition
}

//************************************//
//       Main Function                //
//************************************//
int main(int argc, char** argv)
{	
	ros::init(argc,argv,"SkinNode");
	
	SkinDetection* skinDetection = new SkinDetection();
	
	skinDetection->spin();
	
	delete skinDetection;
		
	return 0;	
}
