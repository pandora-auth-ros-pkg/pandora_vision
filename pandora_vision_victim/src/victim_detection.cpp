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
  VictimDetection::VictimDetection(const std::string& ns) : 
    _nh(ns), 
    params(ns)
  {
     //!< Set initial value of parent frame id to null
    _parent_frame_id = "";
    _frame_id = "";

    /// Convert field of view from degrees to rads
    VictimParameters::hfov = VictimParameters::hfov * CV_PI / 180;
    VictimParameters::vfov = VictimParameters::vfov * CV_PI / 180;
                       
    //! Declare publisher and advertise topic
    //! where algorithm results are posted
    _victimDirectionPublisher = 
      _nh.advertise<pandora_common_msgs::GeneralAlertMsg>(
        VictimParameters::victimAlertTopic, 10, true);
                       
    /// Subscribe to input image's topic
    /// image_transport::ImageTransport it(_nh);
    _frameSubscriber = _nh.subscribe(
      VictimParameters::enhancedHolesTopic, 
        1, &VictimDetection::imageCallback, this);
    
     /// Initialize victim detector
    _victimDetector = new VictimDetector( 
      VictimParameters::cascade_path, 
      VictimParameters::model_path, 
      VictimParameters::bufferSize,
      VictimParameters::rgb_classifier_path, 
      VictimParameters::depth_classifier_path);
    
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
   
  }


  
  /**
  @brief Function that retrieves the parent to the frame_id.
  @param void
  @return bool Returns true is frame_id found or false if not
  **/
  bool VictimDetection::getParentFrameId()
  {
    // Parse robot description
    const std::string model_param_name = "/robot_description";
    bool res = _nh.hasParam(model_param_name);

    std::string robot_description = "";

    if(!res || !_nh.getParam(model_param_name, robot_description))
    {
      ROS_ERROR("[Motion_node]:Robot description couldn't be \
        retrieved from the parameter server.");
      return false;
    }
  
    boost::shared_ptr<urdf::ModelInterface> model(
      urdf::parseURDF(robot_description));

    // Get current link and its parent
    boost::shared_ptr<const urdf::Link> currentLink = model->getLink(_frame_id);
    if(currentLink){
      boost::shared_ptr<const urdf::Link> parentLink = currentLink->getParent();
      // Set the parent frame_id to the parent of the frame_id
      _parent_frame_id = parentLink->name;
      return true;
    }
    else
      _parent_frame_id = _frame_id;
      
    return false;
  }

  /**
   * @brief Function called when new message appears from hole_detector_node
   * @param msg [vision_communications::EnhancedHolesVectorMsg&] The message
   * @return void
   */
  void VictimDetection::imageCallback(
      const vision_communications::EnhancedHolesVectorMsg& msg)
  {
    
    if(
      (curState != 
        state_manager_communications::robotModeMsg::MODE_ARM_APPROACH) ||
      (curState != 
        state_manager_communications::robotModeMsg::MODE_DF_HOLD)
    )
    {
      ROS_WARN("A");
      return;
    }
    
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg.rgbImage, 
      sensor_msgs::image_encodings::TYPE_8UC3);
    
    cv::Mat rgbImage = in_msg->image.clone();
    
    if(_frame_id.c_str()[0] == '/')
      _frame_id = _frame_id.substr(1);
      
       
    if (rgbImage.empty()){
      ROS_FATAL("[victim_node] : No more frames ");
      ROS_BREAK();
    }
    
    in_msg = cv_bridge::toCvCopy(msg.depthImage, 
      sensor_msgs::image_encodings::TYPE_8UC1);
    
    cv::Mat depthImage = in_msg->image.clone();

    _frame_id = msg.header.frame_id; 
    victimFrameTimestamp = msg.header.stamp;
    _enhancedHoles = msg;

    victimFrameTimestamp = in_msg->header.stamp;
    cameraFrameId= in_msg->header.frame_id;
    
    //! The actual victim detection
    detectVictims(
      msg.isDepth, 
      _enhancedHoles.enhancedHoles.size() > 0,
      rgbImage,
      depthImage
    );
    
    //! Resolve frame ids (must explain more)
    std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
    if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) 
    {
      bool _indicator = getParentFrameId();
      _frame_ids_map.insert( it , std::pair<std::string, std::string>(
         _frame_id, _parent_frame_id));
    } 
    
  }
  
  /**
   * @brief This method check in which state we are, according to
   * the information sent from hole_detector_node
   * @return void
  */
  void VictimDetection::detectVictims(
    bool depthEnabled, 
    bool holesEnabled,
    const cv::Mat& rgbImage,
    const cv::Mat& depthImage
  )
  {
    DetectionImages imgs; 
    int stateIndicator = 2 * depthEnabled + holesEnabled + 1;
    
    {
      EnhancedMat emat;
      rgbImage.copyTo(emat.img);
      imgs.rgb = emat;
      imgs.rgb.bounding_box = cv::Rect(0, 0, 0, 0);
      imgs.rgb.keypoint = cv::Point2f(0, 0);
    }
      
    switch(stateIndicator)
    {
      case 1:
        _victimDetector->detectionMode = GOT_NOTHING;
        break;
      case 2:
        _victimDetector->detectionMode = GOT_MASK;
        break;
      case 3:
        _victimDetector->detectionMode = GOT_DEPTH;
        {
          EnhancedMat emat;
          depthImage.copyTo(emat.img);
          imgs.depth = emat;
          imgs.depth.bounding_box = cv::Rect(0, 0, 0, 0);
          imgs.depth.keypoint = cv::Point2f(0, 0);
        }
        break;
      case 4:
      _victimDetector->detectionMode = GOT_ALL;
        {
          EnhancedMat emat;
          depthImage.copyTo(emat.img);
          imgs.depth = emat;
          imgs.depth.bounding_box = cv::Rect(0, 0, 0, 0);
          imgs.depth.keypoint = cv::Point2f(0, 0);
        }
        break;
    }
    for(unsigned int i = 0 ; i < _enhancedHoles.enhancedHoles.size();
      i++)
    {
      
      int minx = 10000, maxx = -1, miny = 10000, maxy = -1;
      for(unsigned int j = 0 ; j < 4 ; j++)
      {
        int xx = _enhancedHoles.enhancedHoles[i].verticesX[j];
        int yy = _enhancedHoles.enhancedHoles[i].verticesY[j];
        minx = xx < minx ? xx : minx;
        maxx = xx > maxx ? xx : maxx;
        miny = yy < miny ? yy : miny;
        maxy = yy > maxy ? yy : maxy;
      }
      cv::Rect rect(minx, miny, maxx - minx, maxy - miny);
      
      EnhancedMat emat;
      emat.img = rgbImage(rect);
      cv::resize(emat.img, emat.img, cv::Size(640, 480));
      emat.bounding_box = rect;
      emat.keypoint = cv::Point2f(
        _enhancedHoles.enhancedHoles[i].keypointX,
        _enhancedHoles.enhancedHoles[i].keypointY
      );

      imgs.rgbMasks.push_back(emat);
      if(depthEnabled)
      {
        emat.img = depthImage(rect);
        imgs.depthMasks.push_back(emat);
      }
    }

    std::vector<DetectedVictim> final_victims = 
      _victimDetector->victimFusion(imgs);

    //!< Message alert creation
    for(int i = 0;  i < final_victims.size() ; i++)
    {
      
      if(final_victims[i].probability < 0.1)
      {
        continue;
      }
      
      float x = final_victims[i].keypoint.x
          - static_cast<float>(VictimParameters::frameWidth) / 2;
      float y = static_cast<float>(VictimParameters::frameHeight) / 2
          - final_victims[i].keypoint.y;
          
      //!< Create message of Victim Detector
      pandora_common_msgs::GeneralAlertMsg victimMessage;
                                      
      victimMessage.header.frame_id = _frame_ids_map.find(_frame_id)->second;
      
      victimMessage.header.stamp = victimFrameTimestamp;
      
      victimMessage.yaw = 
        atan(2 * x / VictimParameters::frameWidth 
          * tan(VictimParameters::hfov / 2));
      
      victimMessage.pitch = 
        atan(2 * y / VictimParameters::frameHeight 
          * tan(VictimParameters::vfov / 2));
          
      victimMessage.probability = final_victims[i].probability;
      
      _victimDirectionPublisher.publish(victimMessage);
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

    //!< shutdown if the robot is switched off
    if (curState == 
      state_manager_communications::robotModeMsg::MODE_TERMINATING)
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
