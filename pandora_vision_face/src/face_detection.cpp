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
* Author: George Aprilis	
* 		  Despoina Paschalidou
*********************************************************************/

#include <vision_communications/FaceDirectionMsg.h>

#include "pandora_vision_face/face_detection.h"


//constructor
FaceDetection::FaceDetection() : _nh()
{
	std::cout<<"FaceDetection constructor"<<std::endl;
    // Get General Parameters, such as frame width & height , camera id
	getGeneralParams();
	
	//Get Face Detector Parameters
	getFaceParams();

	//Convert field of view from degrees to rads
	hfov = HFOV * CV_PI / 180;
	vfov = VFOV * CV_PI / 180;

	ratioX = hfov / frameWidth;
	ratioY = vfov / frameHeight;

	//initialize face detector
	_faceDetector =	new FaceDetector(cascadeName,csvName, bufferSize, skinEnabled, scaleFactor, skinHist, wallHist, wall2Hist);

	//memory will be allocated in the imageCallback
	faceFrame= cv::Mat::zeros(frameWidth,frameHeight,CV_8UC3);
	extraFrame=cv::Mat::zeros(frameWidth,frameHeight,CV_8UC3);
	

	//Declare publisher and advertise topic where algorithm results are posted
	_victimDirectionPublisher = _nh.advertise<vision_communications::FaceDirectionMsg>("face_direction", 10);

	//Advertise topics for debugging if we are in debug mode
	if (debugFace)
	{
		_faceDetector->isDebugMode = debugFace; //the debug parameter is passed into the face detector class
		_faceNowPublisher = image_transport::ImageTransport(_nh).advertise("faceNow", 1);
		_facePrevPublisher = image_transport::ImageTransport(_nh).advertise("facePrev", 1);
		if(skinEnabled)
		{
			_faceSkinPublisher = image_transport::ImageTransport(_nh).advertise("faceSkin", 1);
		}
	}

	//subscribe to input image's topic
	image_transport::ImageTransport it(_nh);
	std::string transport = "theora";
	_frameSubscriber = image_transport::ImageTransport(_nh).subscribe(imageTopic, 1, &FaceDetection::imageCallback,this /*,transport*/ );

	//initialize states - robot starts in STATE_OFF 
	curState = state_manager_communications::robotModeMsg::MODE_OFF;
	prevState =state_manager_communications::robotModeMsg::MODE_OFF;

	//initialize state Managing Variables
	faceNowON 	= false;

	//Initialize mutex lock
    pthread_mutex_init ( &faceLock, NULL);

	//initialize flag used to sync the callbacks
	isFaceFrameUpdated = false;

	clientInitialize();

	ROS_INFO("[faceNode] : Created Face Detection instance");
}


FaceDetection::~FaceDetection()
{
	ROS_DEBUG("[faceNode] : Destroying Face Detection instance");
	delete _faceDetector;
	

}

//***************************************//
//         Get the timer parameters      //
//***************************************//
void FaceDetection::getTimerParams()
{	
	// Get the FaceDenseTime parameter if available
	if (_nh.hasParam("faceDenseTime")) {
		_nh.getParam("faceDenseTime", faceDenseTime);
		ROS_DEBUG_STREAM("faceDenseTime : " << faceDenseTime);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter faceDenseTime not found. Using Default");
		faceDenseTime = 0.5;
	}

	// Get the FaceSparseTime parameter if available
	if (_nh.hasParam("faceSparseTime")) {
		_nh.getParam("faceSparseTime", faceSparseTime);
		ROS_DEBUG_STREAM("faceSparseTime : " << faceSparseTime);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter faceSparseTime not found. Using Default");
		faceSparseTime = 1.5;
	}
}


//**************************************//
// 	Get parameters referring to view	// 
//		and frame characteristics		//
//**************************************//
void FaceDetection::getGeneralParams()
{
	packagePath = ros::package::getPath("vision");
	// Get the faceDummy parameter if available;
	if (_nh.hasParam("faceDummy")) {
		_nh.getParam("faceDummy", faceDummy);
		ROS_DEBUG("faceDummy: %d", faceDummy);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter faceDummy not found. Using Default");
		faceDummy = false;
	}

	// Get the Height parameter if available;
	if (_nh.hasParam("height")) {
		_nh.getParam("height", frameHeight);
		ROS_DEBUG_STREAM("height : " << frameHeight);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter frameHeight not found. Using Default");
		frameHeight = DEFAULT_HEIGHT;
	}

	// Get the images's topic;
	if (_nh.hasParam("imageTopic")) {
		_nh.getParam("imageTopic", imageTopic);
		ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter imageTopic not found. Using Default");
		imageTopic = "/camera_head/image_raw";
	}

	// Get the Width parameter if available;
	if (_nh.hasParam("width")) {
		_nh.getParam("width", frameWidth);
		ROS_DEBUG_STREAM("width : " << frameWidth);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter frameWidth not found. Using Default");
		frameWidth = DEFAULT_WIDTH;
	}
}

//**************************************//
// 		Get parameters referring to 	// 
//		face detection algorithm 		//
//**************************************//
void FaceDetection::getFaceParams()
{		
	// Get the debugFace parameter if available;
	if (_nh.hasParam("debugFace")) {
		_nh.getParam("debugFace", debugFace);
		ROS_DEBUG_STREAM("debugFace : " << debugFace);
	}
	else {
		ROS_DEBUG("[FaceNode] : Parameter debugFace not found. Using Default");
		debugFace = true;
	}

	// Get the path of haar_cascade xml file if available;
	if (_nh.hasParam("cascadeName")) {
		_nh.getParam("cascadeName", cascadeName);
		ROS_DEBUG_STREAM("cascadeName : " << cascadeName);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter cascadeName not found. Using Default");
		std::string temp="/data/haarcascade_frontalface_alt_tree.xml";
		cascadeName.assign(packagePath);
		cascadeName.append(temp);

	}
	
	if (_nh.hasParam("csvName")) {
		_nh.getParam("csvName", csvName);
		ROS_DEBUG_STREAM("csvName : " << csvName);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter csv not found. Using Default");
		std::string temp="/data/csv.ext";
		csvName.assign(packagePath);
		csvName.append(temp);
	}

	//
	if (_nh.hasParam("bufferSize")) {
		_nh.getParam("bufferSize", bufferSize);
		ROS_DEBUG_STREAM("bufferSize : " << bufferSize);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter bufferSize not found. Using Default");
		bufferSize = 5;
	}

	//
	if (_nh.hasParam("skinEnabled")) {
		_nh.getParam("skinEnabled", skinEnabled);
		ROS_DEBUG_STREAM("skinEnabled : " << skinEnabled);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter skinEnabled not found. Using Default");
		skinEnabled = false;
	}

	//
	if (_nh.hasParam("scaleFactor")) {
		_nh.getParam("scaleFactor", scaleFactor);
		ROS_DEBUG_STREAM("scaleFactor : " << scaleFactor);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter scaleFactor not found. Using Default");
		scaleFactor = 1.1;
	}

	//
	if (_nh.hasParam("mn")) {
		_nh.getParam("mn", mn);
		ROS_DEBUG_STREAM("minimum Neighbors : " << mn);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter mn not found. Using Default");
		mn = 5;
	}

	//
	if (_nh.hasParam("minFaceDim")) {
		_nh.getParam("minFaceDim", minFaceDim);
		ROS_DEBUG_STREAM("minFaceDim : " << minFaceDim);
	}
	else {
		ROS_DEBUG("[faceNode] : Parameter minFaceDim not found. Using Default");
		minFaceDim = 70;
	}

	////////////////////////////////////////////////////////////////////////

	// Params used to pass values to the skin detector used internally in FaceDetector

	if (_nh.hasParam("skinHist")) {
		_nh.getParam("skinHist", skinHist);
		ROS_DEBUG_STREAM("skinHist : " << skinHist);
	}
	else {
		ROS_DEBUG("[FaceNode] : Parameter skinHist not found. Using Default");
		std::string temp="/data/histogramms/histogramm_skin.jpg";
		skinHist.assign(packagePath);
		skinHist.append(temp);
	}

	if (_nh.hasParam("wallHist")) {
		_nh.getParam("wallHist", wallHist);
		ROS_DEBUG_STREAM("wallHist : " << wallHist);
	}
	else {
		ROS_DEBUG("[FaceNode] : Parameter wallHist not found. Using Default");
		std::string temp="/data/histogramms/histogramm_wall.jpg";
		wallHist.assign(packagePath);
		wallHist.append(temp);
	}

	if (_nh.hasParam("wall2Hist")) {
		_nh.getParam("wall2Hist", wall2Hist);
		ROS_DEBUG_STREAM("wall2Hist : " << wall2Hist);
	}
	else {
		ROS_DEBUG("[FaceNode] : Parameter wall2Hist not found. Using Default");
		std::string temp="/data/histogramms/histogramm_wall2.jpg";
		wall2Hist.assign(packagePath);
		wall2Hist.append(temp);
	}

}

//**************************************//
//      The Image callback function     //
//**************************************//
void FaceDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	ROS_DEBUG_NAMED("Image Callback","Entering Image Callback");
	int res = -1;

	cv_bridge::CvImagePtr in_msg;
	in_msg = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
	faceFrame= in_msg->image.clone();
	faceFrameTimestamp = msg->header.stamp;
	
	if (faceFrame.empty() )			
	{
		ROS_ERROR("[faceNode] : No more Frames or something went wrong with bag file");
		ros::shutdown();
		return;
	}

	//try to lock face variables
	res = pthread_mutex_trylock(&faceLock);

	//if lock was successful: update frame, set boolean and unlock
	if(res == 0)
	{
		//memcpy( (uchar *)extraFrame.data , (uchar *)faceFrame.data , faceFrame.size().height*faceFrame.step );
		extraFrame=faceFrame.clone();
		isFaceFrameUpdated = true;
		pthread_mutex_unlock(&faceLock);
	}
}

//**************************************//
//      The callback function           //
//**************************************// 
void FaceDetection::faceCallback(const ros::TimerEvent&)
{   
	if(!faceNowON)
	{
		return;
	}
	ROS_DEBUG_NAMED("Face Callback","Enters Face Callback");

	int retries = 0;
	while (true)
	{
		pthread_mutex_lock(&faceLock);

		if (isFaceFrameUpdated)
			break;
		else
		{
			//if input frame is not yet set
			//sleep for a while to let
			//imageCallback() to catch up
			pthread_mutex_unlock(&faceLock);

			ros::Duration(0.5).sleep();

			if (retries > 10)
			{
				ROS_INFO("[faceNode] : Timed out waiting for faceFrame!");
				return;
			}
			retries++;
		}
	}

	// Create Msg for faces
	vision_communications::FaceDirectionMsg faceMessage;

	//Hold the center coordinates of face
	float center_x = -1;
	float center_y = -1;

	if (faceDummy)
	{
		createDummyFaceMessage(center_x, center_y, faceMessage);
	}
	else
	{ 
		createFaceMessage(faceMessage);               
	}

	//reset the flag
	isFaceFrameUpdated = false;
	//and unlock before leaving
	pthread_mutex_unlock(&faceLock);
}

void FaceDetection::createDummyFaceMessage(float &center_x, float &center_y, vision_communications::FaceDirectionMsg &faceMessage )
{
	for( int i = 0; i < 3; i++){

		center_x = ratioX * ( 300 - frameWidth/2 );
		center_y = -1 * ratioY * ( 200 + frameHeight/2 );

		faceMessage.yaw = center_x;
		faceMessage.pitch = center_y;
		//faceMessage.area = 2000;
		faceMessage.header.frame_id="Face";
		faceMessage.probability = 1;
		//faceMessage.type = vision_communications::victimIdentificationDirectionMsg::FACE;
		faceMessage.header.stamp = ros::Time::now();
		_victimDirectionPublisher.publish(faceMessage);

	}

	//dummy delay
	usleep(1000 * 350);
}

void FaceDetection::createFaceMessage(vision_communications::FaceDirectionMsg &faceMessage)
{
	ROS_DEBUG_NAMED("Face Callback","Detecting face");
	//do the detection
	int facesNum = _faceDetector->findFaces(extraFrame);

	//if there is a problem with loading, findFaces returns -2
	if (facesNum == -2) ROS_ERROR( "[faceNode] : Problem with loading Skin images" );

	if (facesNum == 0)
	{
		ROS_DEBUG_NAMED("Face Callback","Face not found");
	}
	//send message if faces are found
	if (facesNum){
		ROS_DEBUG_NAMED("Face Callback","Face found");
		int* facesTable = _faceDetector->getFaceRectTable();
		//Send a message for every face found in the frame
		for(int i=0 ; i < facesNum ; i++){


			faceMessage.yaw = ratioX * ( facesTable[i*4] - (double)frameWidth/2 );
			faceMessage.pitch = -ratioY * ( facesTable[i*4+1] - (double)frameHeight/2 );
			faceMessage.header.frame_id="headCamera";
			faceMessage.probability = _faceDetector->getProbability();
			faceMessage.header.stamp = ros::Time::now();
			std::cout<<"[FaceNode]:Face found"<<std::endl;
			_victimDirectionPublisher.publish(faceMessage);
		}
		delete facesTable;
	}

	if (debugFace){

		cv_bridge::CvImage out_msg_faceNow;
		out_msg_faceNow.encoding = sensor_msgs::image_encodings::MONO8;
		out_msg_faceNow.image = _faceDetector->getFaceNow();

		_faceNowPublisher.publish(out_msg_faceNow.toImageMsg());

		cv_bridge::CvImage out_msg_facePrev;
		out_msg_facePrev.encoding = sensor_msgs::image_encodings::MONO8;
		out_msg_facePrev.image = _faceDetector->getFacePrev();

		_faceNowPublisher.publish(out_msg_facePrev.toImageMsg());

		if(skinEnabled)
		{
			cv_bridge::CvImage out_msg_faceSkin;
			out_msg_faceSkin.encoding = sensor_msgs::image_encodings::MONO8;
			out_msg_faceSkin.image = _faceDetector->getFaceSkin();

			_faceNowPublisher.publish(out_msg_faceSkin.toImageMsg());
		}

	}
}

void FaceDetection::spin()
{
	getTimerParams();
	faceTimer = _nh.createTimer(ros::Duration(faceSparseTime), &FaceDetection::faceCallback , this);

	faceTimer.start();
	ros::spin();
}

//*************************************//
//       State Manager                 //
//*************************************//
void FaceDetection::startTransition(int newState){
	
	curState = newState;

	//check if face detection algorithm should be running now
	faceNowON=	( curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) ||
			( curState == state_manager_communications::robotModeMsg::MODE_ARM_APPROACH ) ||
			( curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD );

	//set the Face Detector Timer period according to the current State
	if( (curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) ){
		faceTimer.setPeriod( ros::Duration(faceSparseTime) );
	}
	if( (curState == state_manager_communications::robotModeMsg::MODE_ARM_APPROACH) ||
	 (curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD) ){
		faceTimer.setPeriod( ros::Duration(faceDenseTime) );
	}

	//shutdown if the robot is switched off
	if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING){
		ros::shutdown();
		return;
	}

	prevState=curState;

	transitionComplete(curState); //this needs to be called everytime a node finishes transition
}

void FaceDetection::completeTransition(void){
	ROS_INFO("[FaceNode] : Transition Complete");
}


//************************************//
//       Main Function                //
//************************************//
int main(int argc, char** argv)
{
	ros::init(argc,argv,"FaceNode");

	FaceDetection* faceDetection = new FaceDetection();

	faceDetection->spin();

	delete faceDetection;

	return 0;
}
