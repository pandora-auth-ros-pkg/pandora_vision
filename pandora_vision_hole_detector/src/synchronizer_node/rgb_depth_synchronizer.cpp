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

    //!< Subscribe to the RGB point cloud topic
    pointCloudSubscriber_ = nodeHandle_.subscribe(
      "/camera/depth_registered/points", 1,
      &RgbDepthSynchronizer::synchronizedCallback, this);

    //!< Advertise the synchronized point cloud
    synchronizedPointCloudPublisher_ = nodeHandle_.advertise
      <sensor_msgs::PointCloud2>("/synchronized/camera/depth/points", 1000);

    //!< Advertise the synchronized rgb image
    synchronizedRGBImagePublisher_ = nodeHandle_.advertise
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
    const sensor_msgs::PointCloud2ConstPtr& pointCloudMessage)
  {
    //!< Extract the RGB image from the point cloud
    cv::Mat rgbImage = pointCloudToRGBImage(pointCloudMessage);

    //!< Convert the cv::Mat image to a ROS message
    cv_bridge::CvImagePtr imageMessagePtr(new cv_bridge::CvImage());

    imageMessagePtr->header = pointCloudMessage->header;
    imageMessagePtr->encoding = sensor_msgs::image_encodings::BGR8;
    imageMessagePtr->image = rgbImage;

    //!< Publish the synchronized point cloud
    synchronizedPointCloudPublisher_.publish(pointCloudMessage);
    //!< Publish the synchronized rgb image
    synchronizedRGBImagePublisher_.publish(imageMessagePtr->toImageMsg());
  }



  /**
    @brief Extracts a RGB image from a point cloud message
    @param pointCloud[in] [const sensor_msgs::PointCloud2ConstPtr&]
    The input point cloud message
    @return cv::Mat The output rgb image
   **/
  cv::Mat RgbDepthSynchronizer::pointCloudToRGBImage(
    const sensor_msgs::PointCloud2ConstPtr& pointCloudMessage)
  {
    PointCloud pointCloud;

    //!< convert the point cloud from sensor_msgs::PointCloud2ConstrPtr
    //!< to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*pointCloudMessage, pointCloud);

    //!< convert the point cloud from pcl::PCLPointCloud2 to pcl::PointCLoud
    PointCloudXYZRGBPtr pointCloudXYZRGB (new PointCloudXYZRGB);
    pcl::fromPCLPointCloud2 (pointCloud, *pointCloudXYZRGB);

    //!< prepare to convert the array to an opencv image
    cv::Mat rgbImage(pointCloudXYZRGB->height, pointCloudXYZRGB->width,
      CV_8UC3);

    for (unsigned int row = 0; row < pointCloudXYZRGB->height; ++row)
    {
      for (unsigned int col = 0; col < pointCloudXYZRGB->width; ++col)
      {
        rgbImage.at<unsigned char>(row, 3 * col + 2) =
          pointCloudXYZRGB->points[col + pointCloudXYZRGB->width * row].r;
        rgbImage.at<unsigned char>(row, 3 * col + 1) =
          pointCloudXYZRGB->points[col + pointCloudXYZRGB->width * row].g;
        rgbImage.at<unsigned char>(row, 3 * col + 0) =
          pointCloudXYZRGB->points[col + pointCloudXYZRGB->width * row].b;
      }
    }
    return rgbImage;
  }
}
