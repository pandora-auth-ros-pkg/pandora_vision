/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Angelos Triantafyllidis <aggelostriadafillidis@gmail.com>
 *********************************************************************/

#include "thermal_node/thermal_cropper.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  ThermalCropper::ThermalCropper(void)
  {
    // Acquire the names of topics which the thermal_cropper node will be having
    // transactionary affairs with
    getTopicNames();

    // Subscribe to the thermal poi published by thermal node
    thermalPoiSubscriber_ = nodeHandle_.subscribe(thermalPoiTopic_, 1,
      &ThermalCropper::inputThermalPoiCallback, this);

    // Subscribe to rgb image published by synchronizer node.
    rgbImageSubscriber_ = nodeHandle_.subscribe(rgbImageTopic_, 1,
      &ThermalCropper::inputRgbImageCallback, this);

    // Subscribe to rgb image published by synchronizer node.
    depthImageSubscriber_ = nodeHandle_.subscribe(depthImageTopic_, 1,
      &ThermalCropper::inputDepthImageCallback, this);

    victimThermalPublisher_ = nodeHandle_.advertise
      <pandora_vision_msgs::EnhancedImage>(victimThermalTopic_, 1000);

    // Advertise empty message to synchronizer node so the thermal process
    // circle will start again
    unlockThermalProcedurePublisher_ = nodeHandle_.advertise
      <std_msgs::Empty>(unlockThermalProcedureTopic_, 1000);
    
    // When this node starts by the launch file, sends message to synchronizer
    // in order for thermal procedure to start.
    std_msgs::Empty unlockThermalProcedure;
    unlockThermalProcedurePublisher_.publish(unlockThermalProcedure);

    ROS_INFO_NAMED(PKG_NAME, "[ThermalCropper node] Initiated");
  }

  /**
    @brief Default destructor
    @return void
   **/
  ThermalCropper::~ThermalCropper(void)
  {
    ROS_INFO_NAMED(PKG_NAME, "[ThermalCropper node] Terminated");
  }

  /**
    @brief Callback for the thermal point of interest received 
    by the thermal node.

    The thermal poi message received by the thermal node is unpacked.
    A counter is set. When this counter reach 2 it means both rgbDepth and
    thermal poi message have been subscribed and are ready to be sent to victim. 
    @param msg [const pandora_vision_hole::CandidateHolesVectorMsg&]
    The thermal image message
    @return void
   **/
  void ThermalCropper::inputThermalPoiCallback(
    const pandora_vision_hole::CandidateHolesVectorMsg& msg)
  {
  }


  /**
    @brief Callback for the rgb image message received by 
    synchronizer node.

    The message received by the synchronizer node is stored in private variable.
    A counter is set. When this counter reaches 3 it means both rgb Depth and
    thermal poi message have been subscribed and are ready to be sent to victim. 
    @param msg [const sensor_msgs::Image&]
    The input rgb image message
    @return void
   **/
  void ThermalCropper::inputRgbImageCallback(const sensor_msgs::Image& msg)
  {
  }

  /**
    @brief Callback for the depth image message received by 
    synchronizer node.

    The message received by the synchronizer node is stored in private variable.
    A counter is set. When this counter reaches 3 it means both rgb Depth and
    thermal poi message have been subscribed and are ready to be sent to victim. 
    @param msg [const sensor_msgs::Image&]
    The input depth image message
    @return void
   **/
  void ThermalCropper::inputDepthImageCallback(const sensor_msgs::Image& msg)
  {
  }

  /**
    @brief Acquires topics' names needed to be subscribed to and advertise
    to by the thermal_cropper node
    @param void
    @return void
   **/
  void ThermalCropper::getTopicNames ()
  {
    // The namespace dictated in the launch file
    std::string ns = nodeHandle_.getNamespace();

    // Read the name of the topic from where the thermalcropper node acquires
    // the thermal poi message and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/subscribed_topics/thermal_poi_topic",
        thermalPoiTopic_ ))
    {
    
      // Make topic's name absolute  
      thermalPoiTopic_ = ns + "/" + thermalPoiTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Subscribed to the input thermal Poi");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic thermal_Poi_topic");
    }

    // Read the name of the topic from where the thermalcropper node acquires
    // the rgb image and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/subscribed_topics/rgb_image_topic",
        rgbImageTopic_ ))
    {
    
      // Make topic's name absolute  
      rgbImageTopic_ = ns + "/" + rgbImageTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Subscribed to the input Rgb image");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic rgb_image_topic");
    }

    // Read the name of the topic from where the thermalcropper node acquires
    // the depth image and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/subscribed_topics/depth_image_topic",
        rgbImageTopic_ ))
    {
    
      // Make topic's name absolute  
      depthImageTopic_ = ns + "/" + depthImageTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Subscribed to the input Depth image");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic depth_image_topic");
    }

     //Read the name of the topic to which the thermal node will be publishing
     //information directly to victim node about the candidate holes found 
     //and store it in a private member variable
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/published_topics/thermal_victim_node_topic",
        victimThermalTopic_))
    {
      // Make the topic's name absolute
      victimThermalTopic_ = ns + "/" + victimThermalTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Advertising to the Thermal-victim node topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic thermal_victim_node_topic");
    }

     //Read the name of the topic to which the thermal node will be publishing
     //information directly to synchronizer node.
    if (nodeHandle_.getParam(
        ns + "/thermal_cropper_node/published_topics/thermal_unlock_synchronizer_topic",
        unlockThermalProcedureTopic_))
    {
      // Make the topic's name absolute
      unlockThermalProcedureTopic_ = ns + "/" + unlockThermalProcedureTopic_;

      ROS_INFO_NAMED(PKG_NAME,
        "[ThermalCropper Node] Advertising to the Synchronizer node topic");
    }
    else
    {
      ROS_ERROR_NAMED(PKG_NAME,
        "[ThermalCropper Node] Could not find topic thermal_unlock_synchronizer_topic");
    }
  }

} // namespace pandora_vision
