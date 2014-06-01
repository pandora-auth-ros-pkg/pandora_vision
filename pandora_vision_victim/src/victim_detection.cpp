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
* Author: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/victim_detection.h"

namespace pandora_vision
{
/**
  @brief Constructor
**/
VictimDetection::VictimDetection(const std::string& ns) : _nh(ns), victimNowON(false)
{
  /// Get general parameters for image processing
  getGeneralParams();
  
  /// Get parameters referring to faceDetector instance
  getFaceDetectorParameters();

  /// Convert field of view from degrees to rads
  hfov = hfov * CV_PI / 180;
  vfov = vfov * CV_PI / 180;

  ratioX = hfov / frameWidth;
  ratioY = vfov / frameHeight;

  /// Memory will allocated in the imageCallback
  victimFrame = cv::Mat::zeros(frameWidth, frameHeight, CV_8UC3);

  /// Subscribe to input image's topic
  /// image_transport::ImageTransport it(_nh);
  _frameSubscriber = image_transport::ImageTransport(_nh).subscribe(
                       imageTopic, 1, &VictimDetection::imageCallback, this);
  
   /// Initialize face detector
  _faceDetector = new FaceDetector(cascade_path, model_path, bufferSize);
  
  /// Initialize states - robot starts in STATE_OFF
  curState = state_manager_communications::robotModeMsg::MODE_OFF;
  prevState = state_manager_communications::robotModeMsg::MODE_OFF;

  clientInitialize();

  ROS_INFO("[victim_node] : Created Victim Detection instance");
}

/**
  @brief Destructor
*/
VictimDetection::~VictimDetection()
{
  ROS_DEBUG("[victim_node] : Destroying Victim Detection instance");
  delete _faceDetector;
}

/**
 @brief Get parameters referring to the view and
 *frame characteristics
 @return void
**/
void VictimDetection::getGeneralParams()
{
  packagePath = ros::package::getPath("pandora_vision_victim");
  
   //! Publishers
    
  //! Declare publisher and advertise topic
  //! where algorithm results are posted
  if (_nh.getParam("published_topic_names/victim_alert", param))
  {
    _victimDirectionPublisher = 
      _nh.advertise<pandora_common_msgs::GeneralAlertMsg>(param, 10, true);
  }
  else
  {
    ROS_FATAL("[victim_node] : Victim alert topic name param not found");
    ROS_BREAK();
  }
    
  //!< Get the camera to be used by hole node;
  if (_nh.getParam("camera_name", cameraName))
  {
    ROS_DEBUG_STREAM("camera_name : " << cameraName);
  }
  else
  {
    ROS_FATAL("[victim_node] : Camera name not found");
    ROS_BREAK();
  }
  
  //!< Get the images's frame_id;
  if (_nh.getParam("/" + cameraName + "/camera_frame_id", cameraFrameId))
  {
    ;
    ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
  }
  else
  {
    cameraFrameId = "/camera";
    ROS_DEBUG_STREAM("camera_frame_id : " << cameraFrameId);
  }
  
  //!< Get the Height parameter if available;
  if (_nh.getParam("/" + cameraName + "/image_height", frameHeight))
  {
    ROS_DEBUG_STREAM("height : " << frameHeight);
  }
  else
  {
    frameHeight = DEFAULT_HEIGHT;
    ROS_DEBUG_STREAM("height : " << frameHeight);
  }

  //!< Get the Width parameter if available;
  if (_nh.getParam("/" + cameraName + "/image_width", frameWidth))
  {
    ROS_DEBUG_STREAM("width : " << frameWidth);
  }
  else
  {
    frameWidth = DEFAULT_WIDTH;
    ROS_DEBUG_STREAM("width : " << frameWidth);
  }
  
   //!< Get the HFOV parameter if available;
  if ( _nh.getParam("/" + cameraName + "/hfov", hfov))
  {
    ROS_DEBUG_STREAM("HFOV : " << hfov);
  }
  else
  {
    hfov = HFOV;
    ROS_DEBUG_STREAM("HFOV : " << hfov);
  }

  //!< Get the VFOV parameter if available;
  if ( _nh.getParam("/" + cameraName + "/vfov", vfov))
  {
    ROS_DEBUG_STREAM("VFOV : " << vfov);
  }
  else
  {
    vfov = VFOV;
    ROS_DEBUG_STREAM("VFOV : " << vfov);
  }
  
  //!< Get the images's topic;
  if (_nh.hasParam("/" + cameraName + "/topic_name"))
  {
    _nh.getParam("/" + cameraName + "/topic_name", imageTopic);
    ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
  }
  else
  {
    imageTopic = "/camera_head/image_raw";
    ROS_DEBUG_STREAM("imageTopic : " << imageTopic);
  }

}

/**
  @brief Get parameters referring to the face detection algorithm
  @return void
**/
void VictimDetection::getFaceDetectorParameters()
{

  //!< Get the path of haar_cascade xml file if available;
  if (_nh.hasParam("cascade_path"))
  {
    _nh.getParam("cascade_path", cascade_path);
    ROS_DEBUG_STREAM("cascade_path : " << cascade_path);
  }
  else
  {
    std::string temp = "/data/haarcascade_frontalface_alt_tree.xml";
    cascade_path.assign(packagePath);
    cascade_path.append(temp);
    ROS_DEBUG_STREAM("cascade_path : " << cascade_path);
  }

  //!< Get the model.xml url;
  if (_nh.hasParam("model_url"))
  {
    _nh.getParam("model_url", model_url);
    ROS_DEBUG_STREAM("modelURL : " << model_url);
  }
  else
  {
    model_url = "https://pandora.ee.auth.gr/vision/model.xml";
    ROS_DEBUG_STREAM("modelURL : " << model_url);
  }

  //!< Get the path of model_path xml file to be loaded
  if (_nh.hasParam("model_path"))
  {
    _nh.getParam("model_path",  model_path);
    ROS_DEBUG_STREAM(" model_path : " <<  model_path);
  }
  else
  {
    model_path = packagePath + "/data/model.xml";
    ROS_DEBUG_STREAM(" model_path : " <<  model_path);
  }


  if (_nh.hasParam("bufferSize"))
  {
    _nh.getParam("bufferSize", bufferSize);
    ROS_DEBUG_STREAM("bufferSize : " << bufferSize);
  }
  else
  {
    bufferSize = 5;
    ROS_DEBUG_STREAM("bufferSize : " << bufferSize);
  }
}

/**
 * @brief Function called when new ROS message appears, for camera
 * @param msg [const sensor_msgs::ImageConstPtr&] The message
 * @return void
 */
void VictimDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr in_msg;
  in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  victimFrame = in_msg->image.clone();
  victimFrameTimestamp = msg->header.stamp;

  if (victimFrame.empty() )
  {
    ROS_ERROR("[victim_node] : No more Frames ");
    return;
  }
  bool isDepthEnabled = true;
  bool isMaskEnabled = true;
  victimDetect(isDepthEnabled, isMaskEnabled);
}

/**
 * @brief This method uses a FaceDetector instance to detect all
 * present faces in a given frame
 * @return void
*/
void VictimDetection::victimDetect(bool isDepthEnabled, bool isMaskEnabled)
{
  if(!victimNowON)
    return;
  
  if(isDepthEnabled == true && isMaskEnabled == true){
    ///Enable Viola Jones for rgb image
      _faceDetector->findFaces(victimFrame);
    ///Enable Viola Jones for depth image
    ///Enable rgb_system validator for rgb image
    ///Enable rgb_system validator for depth image  
  }  
  
  else if(isDepthEnabled == false && isMaskEnabled == true){
    ///Enable Viola Jones for rgb image
      _faceDetector->findFaces(victimFrame);
    ///Enable rgb_system validator for rgb image
  }
   
  else if(isDepthEnabled == true && isMaskEnabled == false){
    ///Enable Viola Jones for rgb image
      _faceDetector->findFaces(victimFrame);
    ///Enable Viola Jones for depth image
  }
  else if(isDepthEnabled == false && isMaskEnabled == false){
    ///Enable Viola Jones for rgb image
      _faceDetector->findFaces(victimFrame);
  }  
}


/**
  * @brief Node's state manager
  * @param newState [int] The robot's new state
  * @return void
 */
void VictimDetection::startTransition(int newState)
{

  curState = newState;

  //!< check if face detection algorithm should be running now
  victimNowON = 
  (curState == state_manager_communications::robotModeMsg::MODE_EXPLORATION) ||
  (curState == state_manager_communications::robotModeMsg::MODE_ARM_APPROACH) ||
  (curState == state_manager_communications::robotModeMsg::MODE_DF_HOLD);

  //!< shutdown if the robot is switched off
  if (curState == state_manager_communications::robotModeMsg::MODE_TERMINATING)
  {
    ros::shutdown();
    return;
  }

  prevState = curState;

  //!< this needs to be called everytime a node finishes transition
  transitionComplete(curState);
}

/**
 * @brief After completion of state transition
 * @return void
 */
void VictimDetection::completeTransition(void)
{
  ROS_INFO("[victim_node] : Transition Complete");
}
}// namespace pandora_vision
