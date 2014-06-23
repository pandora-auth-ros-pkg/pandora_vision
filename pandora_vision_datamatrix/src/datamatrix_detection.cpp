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

#include "pandora_vision_datamatrix/datamatrix_detection.h"

namespace pandora_vision
{
  /**
   *@brief Constructor
  **/
  DatamatrixDetection::DatamatrixDetection(const std::string& ns) : _nh(ns), datamatrixNowON(false)
  {
    //!< Set initial value of parent frame id to null
    _parent_frame_id = "";
    _frame_id = "";
    
    //!< Get General Parameters, such as frame width & height , camera id
    getGeneralParams();
    
    //!< Convert field of view from degrees to rads
    hfov = hfov * CV_PI / 180;
    vfov = vfov * CV_PI / 180;

    ratioX = hfov / frameWidth;
    ratioY = vfov / frameHeight;
    
    for(int ii = 0; ii < _imageTopics.size(); ii++ ){
      //!< subscribe to input image's topic
      _frameSubscriber = _nh.subscribe(
        _imageTopics.at(ii), 1, &DatamatrixDetection::imageCallback, this);
      _frameSubscribers.push_back(_frameSubscriber);
    }
 
    //!< initialize states - robot starts in STATE_OFF
    curState = state_manager_communications::robotModeMsg::MODE_OFF;
    prevState = state_manager_communications::robotModeMsg::MODE_OFF;

    clientInitialize();

    ROS_INFO("[Datamatrix_node] : Created Datamatrix Detection instance");
    
  }
  
  
  /**
    @brief Destructor
   */
  DatamatrixDetection::~DatamatrixDetection()
  {
    ROS_INFO("[Datamatrix_node] : Destroying datamatrix Detection instance");
  }
  
  
  /**
   * @brief Get parameters referring to view and frame characteristics 
   * from launch file
   * @return void
   */
  void DatamatrixDetection::getGeneralParams()
  {
    
    packagePath = ros::package::getPath("pandora_vision_datamatrix");
    
    //! Publishers
    
    //! Declare publisher and advertise topic
    //! where algorithm results are posted
    if (_nh.getParam("published_topic_names/datamatrix_alert", param))
    {
      _datamatrixCodePublisher =
        _nh.advertise<vision_communications::DataMatrixAlertsVectorMsg>(param, 10, true);
    }
    else
    {
      ROS_FATAL("Datamatrix alert topic name param not found");
      ROS_BREAK();
    }
    
     XmlRpc::XmlRpcValue cameras_list;
    if(_nh.getParam("camera_name", cameras_list)){
      ROS_ASSERT(cameras_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
      
      for(int ii = 0; ii < cameras_list.size(); ii++){
        ROS_ASSERT(cameras_list[ii].getType() == XmlRpc::XmlRpcValue::TypeString);
        cameraName = static_cast<std::string>(cameras_list[ii]);
        ROS_INFO_STREAM("[Datamatrix_node]: Camera_name : " << cameraName);
        
        //!< Get the listener's topic for camera
        if (_nh.getParam("/" + cameraName + "/topic_name", imageTopic))
        {
          ROS_INFO_STREAM("[Datamatrix_node]: ImageTopic for camera : " << imageTopic);
        }
        else
        {
         ROS_FATAL("[Datamatrix_node]: Image topic name not found");
         ROS_BREAK(); 
        }
        _imageTopics.push_back("/"+imageTopic);
        
      }
    }
    else
    {
      ROS_FATAL("Camera_name not found");
      ROS_BREAK(); 
    }  
    
    //!< Get the Height parameter if available;
    if (_nh.getParam("/" + cameraName + "/image_height", frameHeight))
      ROS_DEBUG_STREAM("height : " << frameHeight);
    else
    {
      ROS_DEBUG("[Datamatrix_node] : Parameter frameHeight not found. Using Default");
      frameHeight = DEFAULT_HEIGHT;
    }

    //!< Get the Width parameter if available;
    if (_nh.getParam("/" + cameraName + "/image_width", frameWidth))
      ROS_DEBUG_STREAM("width : " << frameWidth);
    else
    {
      ROS_DEBUG("[Datamatrix_node] : Parameter frameWidth not found. Using Default");
      frameWidth = DEFAULT_WIDTH;
    }
    
    //!< Get the HFOV parameter if available;
    if (_nh.getParam("/" + cameraName + "/hfov", hfov))
      ROS_DEBUG_STREAM("HFOV : " << hfov);
    else
    {
      ROS_DEBUG("[Datamatrix_node] : Parameter frameWidth not found. Using Default");
      hfov = HFOV;
    }

    //!< Get the VFOV parameter if available;
    if (_nh.getParam("/" + cameraName + "/vfov", vfov))
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    else
    {
      ROS_DEBUG("[Datamatrix_node] : Parameter frameWidth not found. Using Default");
      vfov = VFOV;
    }
  }
  
    /**
  @brief Function that retrieves the parent to the frame_id.
  @param void
  @return bool Returns true is frame_id found or false if not
  **/
  bool DatamatrixDetection::getParentFrameId()
  {
    // Parse robot description
    const std::string model_param_name = "/robot_description";
    bool res = _nh.hasParam(model_param_name);

    std::string robot_description = "";

    if(!res || !_nh.getParam(model_param_name, robot_description))
    {
      ROS_ERROR("[Datamatrix_node]:Robot description couldn't be retrieved from the parameter server.");
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
   * @brief Function called when new ROS message appears from camera
   * @param msg [const sensor_msgs::Image&] The message
   * @return void
  */
  void DatamatrixDetection::imageCallback(const sensor_msgs::Image& msg)
  {
    
    cv_bridge::CvImagePtr in_msg;
    in_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    datamatrixFrame = in_msg->image.clone();
    datamatrixFrameTimestamp = msg.header.stamp;
    _frame_id = msg.header.frame_id;
    
    if(_frame_id.c_str()[0] == '/')
      _frame_id = _frame_id.substr(1);
      
    if (!datamatrixFrame.data)
    {
      ROS_ERROR("[Datamatrix_node] : No more Frames!");
      return;
    }
    
    std::map<std::string, std::string>::iterator it = _frame_ids_map.begin();
      
    if(_frame_ids_map.find(_frame_id) == _frame_ids_map.end() ) {
      bool _indicator = getParentFrameId();
      
      _frame_ids_map.insert( it , std::pair<std::string, std::string>(
         _frame_id, _parent_frame_id));
      
       for (it =_frame_ids_map.begin(); it !=_frame_ids_map.end(); ++it)
          ROS_DEBUG_STREAM("" << it->first << " => " << it->second );
    } 
    
    datamatrixDetect();
  }
  
  /**
   * @brief This method uses a DatamatrixDetector instance to detect 
   * all present datamatrixes in a given frame
   * @return void
  */
  void DatamatrixDetection::datamatrixDetect()
  {
    if(!datamatrixNowON)
    {
      return;
    }
    //!< Create message of DatamatrixCode Detector
    vision_communications::DataMatrixAlertsVectorMsg datamatrixcodeVectorMsg;
    vision_communications::DataMatrixAlertMsg datamatrixcodeMsg;
    datamatrixcodeVectorMsg.header.frame_id = _frame_ids_map.find(_frame_id)->second;
    datamatrixcodeVectorMsg.header.stamp = ros::Time::now();

    _datamatrixDetector.detect_datamatrix(datamatrixFrame);
    std::vector<DataMatrixQode> list_datamatrixes = _datamatrixDetector.get_detected_datamatrix();
    
    for(int i = 0; i < static_cast<int>(list_datamatrixes.size()); i++)
    {
      datamatrixcodeMsg.datamatrixContent = list_datamatrixes[i].message;
      datamatrixcodeMsg.yaw = ratioX *
        (list_datamatrixes[i].datamatrix_center.x - 
          static_cast<double>(frameWidth) / 2);
      datamatrixcodeMsg.pitch = -ratioY *
        (list_datamatrixes[i].datamatrix_center.y - 
          static_cast<double>(frameWidth) / 2);
      datamatrixcodeVectorMsg.dataMatrixAlerts.push_back(datamatrixcodeMsg);

      ROS_INFO("[Datamatrix_node]:Datamatrix found.");
    }

    if(datamatrixcodeVectorMsg.dataMatrixAlerts.size() > 0)
    {
      _datamatrixCodePublisher.publish(datamatrixcodeVectorMsg);
    }
  }
  
  /**
   * @brief Node's state manager
   * @param newState [int] The robot's new state
   * @return void
  */
  void DatamatrixDetection::startTransition(int newState)
  {
    curState = newState;

    //!< check if datamatrix algorithm should be running now
    datamatrixNowON =
      (curState ==
       state_manager_communications::robotModeMsg::MODE_EXPLORATION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_IDENTIFICATION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_ARM_APPROACH)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_TELEOPERATED_LOCOMOTION)
      || (curState ==
          state_manager_communications::robotModeMsg::MODE_DF_HOLD);

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
  void DatamatrixDetection::completeTransition()
  {
    ROS_INFO("[Datamatrix_node] : Transition Complete");
  }
  
}// namespace pandora_vision
