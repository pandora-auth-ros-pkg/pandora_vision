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
* Authors: Alexandros Filotheou, Manos Tsardoulias
*********************************************************************/

#ifndef RBG_DEPTH_SYNCHRONIZER_H
#define RBG_DEPTH_SYNCHRONIZER_H

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <depth_node/defines.h>

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace vision
{
  /**
   @class RgbDepthSynchronizer
   @brief The class responsible for synchronizing two topics' messages:
   a message containing a point cloud and a message containing a rgb image
   **/
  class RgbDepthSynchronizer
  {
    private:

      //!< The ROS node handle
      ros::NodeHandle nodeHandle_;

      //!< The publishers which will advertise the two synchronized topics;
      //!< one for the point cloud and one for the rgb image
      ros::Publisher synchronizedPointCloudPublisher;
      ros::Publisher synchronizedRGBPublisher;

      /**
        @brief The synchronized callback for the point cloud and rgb image
        obtained by the depth sensor.
        @param pointCloudMessage [const sensor_msgs::PointCloud2ConstPtr&]
        The input point cloud
        @param rgbImageMessage [const sensor_msgs::ImageConstPtr&]
        The input rgb image
        @return void
       **/
      void synchronizedCallback(
        const sensor_msgs::PointCloud2ConstPtr& pointCloudMessage,
        const sensor_msgs::ImageConstPtr& rgbImageMessage);

    public:

      /**
        @brief The constructor
       **/
      RgbDepthSynchronizer(void);

      /**
        @brief The default constructor
       **/
      ~RgbDepthSynchronizer(void);
  };
}
#endif
