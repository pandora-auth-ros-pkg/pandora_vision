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

#ifndef THERMAL_NODE_THERMAL_CROPPER_H
#define THERMAL_NODE_THERMAL_CROPPER_H

#include <std_msgs/Empty.h>
#include "thermal_node/thermal.h"
#include "pandora_vision_msgs/EnhancedImage.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class ThermalCropper
    @brief Is responsible to sent the point of interest detected from thermal
    analysis of image to victim node if it exists. Because victim needs enchanced
    message, it also aquires rgb and depth images. When a message is sent
    to victim node, an empty message is sent to synchronizer node to restart 
    the whole thermal process. 
   **/
  class ThermalCropper
  {
    private:

      // The ROS node handle
      ros::NodeHandle nodeHandle_;

      // Subscriber of thermal node result ( the exctracted point of interest)
      ros::Subscriber thermalPoiSubscriber_;

      // The name of the topic where the thermal poi is acquired from
      std::string thermalPoiTopic_;

      // Subscriber of synchronizer node from where we acquire the rgb image.
      ros::Subscriber rgbImageSubscriber_;

      // The name of the topic where rgb image is acquired from
      std::string rgbImageTopic_;

      // Subscriber of synchronizer node from where we acquire the depth image.
      ros::Subscriber depthImageSubscriber_;

      // The name of the topic where depth image is acquired from
      std::string depthImageTopic_;

      // Ros publisher for enchanced message directly to victim node.
      ros::Publisher victimThermalPublisher_;

      // The name of the topic where the thermal_cropper node publishes 
      // directly to victim node.
      std::string victimThermalTopic_;

      // Ros Publisher to synchronizer.
      // Synchronizer dictates thermal standalone process to start.
      ros::Publisher unlockThermalProcedurePublisher_;

      // The name of the topic where the thermal cropper node publishes 
      // to synchronizer node.
      std::string unlockThermalProcedureTopic_;

      /**
        @brief Acquires topics' names needed to be subscribed by the
        the thermal_cropper node.
        @param void
        @return void
       **/
      void getTopicNames();

      /**
        @brief Callback for the thermal point of interest received 
        by the thermal node.

        The thermal poi message received by the thermal node is unpacked.
        A counter is set. When this counter reaches 3 it means both rgb Depth and
        thermal poi message have been subscribed and are ready to be sent to victim. 
        @param msg [const pandora_vision_hole::CandidateHolesVectorMsg&]
        The thermal image message
        @return void
       **/
      void inputThermalPoiCallback(
        const pandora_vision_hole::CandidateHolesVectorMsg& msg);


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
      void inputRgbImageCallback(const sensor_msgs::Image& msg);

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
      void inputDepthImageCallback(const sensor_msgs::Image& msg);

    public:

      /**
        @brief Default constructor. Initiates communications, loads parameters.
        @return void
       **/
      ThermalCropper(void);

      /**
        @brief Default destructor
        @return void
       **/
      ~ThermalCropper(void);
  };

} // namespace pandora_vision

#endif  // THERMAL_NODE_THERMAL_CROPPER_H
