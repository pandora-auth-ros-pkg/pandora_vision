/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Despoina Paschalidou
*********************************************************************/

#include "pandora_vision_victim/victim_parameters.h"

namespace pandora_vision
{
  //----------------------------Parameters----------------------------//
  //!< Dynamic reconfigure parameters
  double VictimParameters::rgb_vj_weight = 0.2;
  double VictimParameters::depth_vj_weight = 0;
  double VictimParameters::rgb_svm_weight = 0.9;
  double VictimParameters::depth_svm_weight = 0;
  
  bool VictimParameters::debug_img = false;
  
  //!< Static parameters
  std::string VictimParameters::packagePath = "";
  std::string VictimParameters::victimAlertTopic = "";
  std::string VictimParameters::enhancedHolesTopic = "";
  std::string VictimParameters::cameraName = "";
  int VictimParameters::frameHeight = "";
  int VictimParameters::frameWidth = "";
  
  //----------------------------Methods----------------------------//
  
  VictimParameters::VictimParameters(void)
  {
    //!< The dynamic reconfigure (depth) parameter's callback
    server.setCallback(boost::bind(&VictimParameters::parametersCallback,
        this, _1, _2));
        
    getGeneralParams();
  }
  
    /**
    @brief The function called when a parameter is changed
    @param[in] config [const pandora_vision_motion::motion_cfgConfig&]
    @param[in] level [const uint32_t] The level 
    @return void
  **/
  void VictimParameters::parametersCallback(
    const pandora_vision_victim::victim_dyn_reconfConfig& config,
    const uint32_t& level)
  {
    VictimParameters::rgb_vj_weight = config.rgb_vj_weight;
    VictimParameters::depth_vj_weight = config.depth_vj_weight;
    VictimParameters::rgb_svm_weight = config.rgb_svm_weight;
    VictimParameters::depth_svm_weight = config.depth_svm_weight;
    VictimParameters::debug_img = config.debug_img;
  }
  
  /**
   @brief Get parameters referring to the view and
   *frame characteristics
   @return void
  **/
  void VictimParameters::getGeneralParams(void)
  {
    VictimParameters::packagePath = ros::package::getPath("pandora_vision_victim");
    
    std::string str_param;
    int int_param;
    
    if (_nh.getParam("published_topic_names/victim_alert", str_param))
    {
      VictimParameters::victimAlertTopic = str_param;
    }
    else
    {
      ROS_FATAL("[victim_node] : Victim alert topic name param not found");
      ROS_BREAK();
    }

    //! Declare subsciber's topic name
    if (_nh.getParam("subscribed_topic_names/enhanded_hole_alert", str_param))
    {
      ROS_INFO_STREAM("PARAM"<< str_param);
      VictimParameters::enhancedHolesTopic = str_param;
    }  
    else
    {
      ROS_FATAL("[victim_node] : Victim subscribed topic name param not found");
      ROS_BREAK();
    }  
    
        
    //!< Get the camera to be used by motion node;
    if (_nh.getParam("camera_name", str_param)) 
    {
      VictimParameters::cameraName = str_param;
      ROS_DEBUG_STREAM("camera_name : " << str_param);
    }
    else 
    {
      ROS_FATAL("[Motion_node]: Camera name not found");
      ROS_BREAK(); 
    }

    //! Get the Height parameter if available;
    if (_nh.getParam("image_height", int_param)) 
    {
      
      ROS_DEBUG_STREAM("height : " << frameHeight);
    }
    else 
    {
      ROS_FATAL("[motion_node] : Parameter frameHeight not found. Using Default");
      ROS_BREAK();
    }
    
    //! Get the Width parameter if available;
    if ( _nh.getParam("image_width", int_param)) 
      ROS_DEBUG_STREAM("width : " << frameWidth);
    else 
    {
      ROS_FATAL("[motion_node] : Parameter frameWidth not found. Using Default");
      ROS_BREAK();
    }
  
    //!< Get the HFOV parameter if available;
    if (_nh.getParam("hfov", hfov)) 
      ROS_DEBUG_STREAM("HFOV : " << hfov);
    else 
    {
     ROS_FATAL("[motion_node]: Horizontal field of view not found");
     ROS_BREAK();
    }
    
    //!< Get the VFOV parameter if available;
    if (_nh.getParam("vfov", vfov)) 
      ROS_DEBUG_STREAM("VFOV : " << vfov);
    else 
    {
     ROS_FATAL("[motion_node]: Vertical field of view not found");
     ROS_BREAK();
    }  
    
  }
}// namespace pandora_vision
