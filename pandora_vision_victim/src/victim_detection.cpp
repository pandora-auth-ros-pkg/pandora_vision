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
    getVictimDetectorParameters();

    /// Convert field of view from degrees to rads
    hfov = hfov * CV_PI / 180;
    vfov = vfov * CV_PI / 180;

    ratioX = hfov / frameWidth;
    ratioY = vfov / frameHeight;
    
    //!< Subscribe to input image's topic
    //!< image_transport::ImageTransport it(_nh);
    //~ _frameSubscriber = _nh.subscribe(
                       //~ "/kinect/rgb/image_color", 1, &VictimDetection::dummyimageCallback, this);
                       
    /// Subscribe to input image's topic
    /// image_transport::ImageTransport it(_nh);
    _frameSubscriber = _nh.subscribe(
              _enhancedHolesTopic, 1, &VictimDetection::imageCallback, this);
    
     /// Initialize victim detector
    _victimDetector = new VictimDetector(cascade_path, model_path, bufferSize,
      rgb_classifier_path, depth_classifier_path);
    
    /// Initialize states - robot starts in STATE_OFF
    curState = state_manager_communications::robotModeMsg::MODE_OFF;
    prevState = state_manager_communications::robotModeMsg::MODE_OFF;

    clientInitialize();
    _rgbImage = cv::Mat::zeros(frameWidth, frameHeight, CV_8UC3);
    
    ROS_INFO("[victim_node] : Created Victim Detection instance");
  }

  /**
    @brief Destructor
  */
  VictimDetection::~VictimDetection()
  {
    ROS_DEBUG("[victim_node] : Destroying Victim Detection instance");
   
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
    if (_nh.getParam("published_topic_names/victim_alert", param)){
      _victimDirectionPublisher = 
        _nh.advertise<pandora_common_msgs::GeneralAlertMsg>(param, 10, true);
    }
    else{
      ROS_FATAL("[victim_node] : Victim alert topic name param not found");
      ROS_BREAK();
    }
     
    //! Subscribers
      
    //! Declare subsciber's topic name
    if (_nh.getParam("subscribed_topic_names/enhanded_hole_alert", param))
    {
      ROS_INFO_STREAM("PARAM"<< param);
      _enhancedHolesTopic = param;
    }  
    else{
      ROS_FATAL("[victim_node] : Victim subscribed topic name param not found");
      ROS_BREAK();
    }  
    
    //!< Get the camera to be used by hole node;
    if (_nh.getParam("camera_name", cameraName))
      ROS_DEBUG_STREAM("camera_name : " << cameraName);
    else{
      ROS_FATAL("[victim_node] : Camera name not found");
      ROS_BREAK();
    }
      
    //!< Get the Height parameter if available;
    if (_nh.getParam("/" + cameraName + "/image_height", frameHeight))
      ROS_DEBUG_STREAM("height : " << frameHeight);
    else{
      frameHeight = DEFAULT_HEIGHT;
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }

    //!< Get the Width parameter if available;
    if (_nh.getParam("/" + cameraName + "/image_width", frameWidth))
      ROS_DEBUG_STREAM("width : " << frameWidth);
    else{
      frameWidth = DEFAULT_WIDTH;
      ROS_DEBUG_STREAM("width : " << frameWidth);
    }
    
     //!< Get the HFOV parameter if available;
    if ( _nh.getParam("/" + cameraName + "/hfov", hfov))
      ROS_DEBUG_STREAM("HFOV : " << hfov);
    else{
      hfov = HFOV;
      ROS_DEBUG_STREAM("HFOV : " << hfov);
    }

    //!< Get the VFOV parameter if available;
    if ( _nh.getParam("/" + cameraName + "/vfov", vfov))
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    else{
      vfov = VFOV;
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    }
    
  }

  /**
    @brief Get parameters referring to the face detection algorithm
    @return void
  **/
  void VictimDetection::getVictimDetectorParameters()
  {
    //!< Get the path of haar_cascade xml file if available;
    if ( _nh.getParam("cascade_path", cascade_path)){
      cascade_path = packagePath + cascade_path;
      ROS_INFO_STREAM("[victim_node]: cascade_path : " << cascade_path);
    }
    else{
      model_path = packagePath + "/data/haarcascade_frontalface_alt_tree.xml";
      ROS_INFO_STREAM("[victim_node]: cascade_path : " << cascade_path);
    }

    //!< Get the model.xml url;
    if (_nh.getParam("model_url", model_url))
      ROS_INFO_STREAM("[victim_node]: modelURL : " << model_url);
    else{
      model_url = "https://pandora.ee.auth.gr/vision/model.xml";
      ROS_INFO_STREAM("[victim_node]: modelURL : " << model_url);
    }

    //!< Get the path of model_path xml file to be loaded
    if (_nh.getParam("model_path",  model_path)){
      model_path = packagePath + model_path;
      ROS_INFO_STREAM("[victim_node]: model_path : " <<  model_path);
    }
    else{
      model_path = packagePath + "/data/model.xml";
      ROS_INFO_STREAM("[victim_node]: model_path : " <<  model_path);
    }
    
    //!< Get the path of rgb classifier
    if (_nh.getParam("rgb_classifier_path",  rgb_classifier_path)){
      rgb_classifier_path = packagePath + rgb_classifier_path;
      ROS_INFO_STREAM("[victim_node]: rgb_training_path classifier  : " 
            <<  rgb_classifier_path);
    }
    else{
      rgb_classifier_path = packagePath + "/data/rgb_svm_classifier.xml";
      ROS_INFO_STREAM("[victim_node]: rgb_training_path classifier  : " 
            <<  rgb_classifier_path);
    }
    
    //!< Get the path of depth classifier
    if (_nh.getParam("depth_classifier_path",  depth_classifier_path)){
      depth_classifier_path = packagePath + depth_classifier_path;
      ROS_INFO_STREAM("[victim_node]: depth_training_path classifier  : " 
            <<  depth_classifier_path);
    }
    else{
      depth_classifier_path = packagePath + "/data/depth_svm_classifier.xml";
      ROS_INFO_STREAM("[victim_node]: depth_training_path classifier  : " 
            <<  depth_classifier_path);
    }
    
    /// Parameter that changes respectivly if we have depth information
    if ( _nh.getParam("isDepthEnabled", isDepthEnabled))
      ROS_DEBUG_STREAM("[victim_node] : isDepthEnabled : " << isDepthEnabled);
    else
      isDepthEnabled = false;
      
    /// Parameter that changes respectivly if we have information
    ///about the position of the hole
    if ( _nh.getParam("isHole", isHole))
      ROS_DEBUG_STREAM("[victim_node] : isHole : " << isHole);
    else
      isDepthEnabled = false;
      
    if ( _nh.getParam("bufferSize", bufferSize))
      ROS_DEBUG_STREAM("[victim_node] : bufferSize : " << bufferSize);
    else
      bufferSize = 5;
  }
  
  /**
   * @brief Function called when new ROS message appears, for camera
   * @param msg [const sensor_msgs::Image&] The message
   * @return void
  */
  void VictimDetection::dummyimageCallback(const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    _rgbImage = in_msg->image.clone();
    victimFrameTimestamp = msg.header.stamp;

    if (_rgbImage.empty() )
    {
      ROS_ERROR("[face_node] : No more Frames ");
      return;
    }
    isDepthEnabled = false;
    isHole = true;
    checkState();
  }

  /**
   * @brief Function called when new message appears from hole_detector_node
   * @param msg [vision_communications::EnhancedHolesVectorMsg&] The message
   * @return void
   */
  void VictimDetection::imageCallback(
      const vision_communications::EnhancedHolesVectorMsg& msg)
  {
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg.rgbImage, sensor_msgs::image_encodings::TYPE_8UC3);
    _rgbImage = in_msg->image.clone();
     
    if (_rgbImage.empty()){
      ROS_FATAL("[victim_node] : No more frames ");
      ROS_BREAK();
    }
 
    in_msg = 
      cv_bridge::toCvCopy(msg.depthImage, sensor_msgs::image_encodings::TYPE_8UC1);
    _depthImage = in_msg->image.clone();
    
    isDepthEnabled = msg.isDepth;
    
    _enhancedHoles = msg.enhancedHoles;
    if (_enhancedHoles.size() > 0)
      isHole = true;
      
    victimFrameTimestamp = in_msg->header.stamp;
    cameraFrameId= in_msg->header.frame_id;
    
    checkState();
    
  }
  
  /**
   * @brief This method check in which state we are, according to
   * the information sent from hole_detector_node
   * @return void
  */
  void VictimDetection::checkState()
  {
    _rgbdImages.clear();
    ///!< First case, where all subsystems are enabled
    if(isDepthEnabled == true && isHole == true){
      _stateIndicator = 1;
      _rgbdImages.push_back(_rgbImage);
      _rgbdImages.push_back(_depthImage);
    }  
    ///!< Second case, where only rgb systems are enabled  
    if(isDepthEnabled == false && isHole == true){
        _stateIndicator = 2;
        _rgbdImages.push_back(_rgbImage);
    }    
    ///!< Third case, where only Viola Jones subsystems for both
    ///!< rgb and depth Image are enabled
    if(isDepthEnabled == true && isHole == false){
        _stateIndicator = 3;
        _rgbdImages.push_back(_rgbImage);
        _rgbdImages.push_back(_depthImage);
    }    
    ///!< Fourth case, where only Viola Jones subsystem for rgb image
    ///!< is enabled
    if(isDepthEnabled == false && isHole == false){
        _stateIndicator = 4;
        _rgbdImages.push_back(_rgbImage);
    }
    victimDetect();    
  }
  
  /**
   * @brief This method uses a FaceDetector instance to detect all
   * present faces in a given frame
   * @return void
  */
  void VictimDetection::victimDetect()
  {
    if(!victimNowON)
      return;
    
    _victimDetector->victimFusion(4, _rgbdImages);
    //~ _victimDetector->victimFusion(_stateIndicator, _rgbdImages);
    
    if(!_rgbdImages.size())
      _rgbdImages.erase(_rgbdImages.begin(), 
        _rgbdImages.size()+ _rgbdImages.begin());
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
