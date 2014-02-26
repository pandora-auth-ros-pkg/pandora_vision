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

#include "synchronizer_node/rgb_depth_synchronizer.h"

namespace vision
{
  /**
    @brief The constructor
   **/
  RgbDepthSynchronizer::RgbDepthSynchronizer(void)
  {
    ros::Duration(0.5).sleep();

    //!< pointCloudSubscriber will subscribe to the topic to which the depth
    //!< sensor publishes the point cloud
    message_filters::Subscriber<sensor_msgs::PointCloud2>
      pointCloudSubscriber(nodeHandle_, "/camera/depth/points", 1);

    //!< imageSubscriber will subscribe to the topic to which the depth
    //!< sensor publishes the RGB image
    message_filters::Subscriber<sensor_msgs::Image>
      imageSubscriber(nodeHandle_, "/camera/rgb/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime
      <sensor_msgs::PointCloud2, sensor_msgs::Image> syncPolicy;

    //!< ApproximateTime takes a queue size as its constructor argument,
    //!< hence syncPolicy(10)
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10),
      pointCloudSubscriber, imageSubscriber);

    sync.registerCallback(
      boost::bind(&RgbDepthSynchronizer::synchronizedCallback, this, _1, _2));

    //!< Advertise the synchronized point cloud
    synchronizedPointCloudPublisher = nodeHandle_.advertise
      <sensor_msgs::PointCloud2>("/synchronized/camera/depth/points", 1000);

    //!< Advertise the synchronized RGB image
    synchronizedRGBPublisher = nodeHandle_.advertise
      <sensor_msgs::Image>("/synchronized/camera/rgb/image_raw", 1000);
  }



  /**
    @brief Default destructor
    @return void
   **/
  RgbDepthSynchronizer::~RgbDepthSynchronizer(void) {}



  /**
    @brief The synchronized callback for the point cloud and rgb image
    obtained by the depth sensor.
    @param pointCloudMessage [const sensor_msgs::PointCloud2ConstPtr&]
    The input point cloud
    @param rgbImageMessage [const sensor_msgs::ImageConstPtr&]
    The input rgb image
    @return void
   **/
  void RgbDepthSynchronizer::synchronizedCallback(
    const sensor_msgs::PointCloud2ConstPtr& pointCloudMessage,
    const sensor_msgs::ImageConstPtr& rgbImageMessage)
  {
    //!< Publish the synchronized point cloud
    synchronizedPointCloudPublisher.publish(pointCloudMessage);

    //!< Publish the synchronized rgb image
    synchronizedRGBPublisher.publish(rgbImageMessage);
  }
}
