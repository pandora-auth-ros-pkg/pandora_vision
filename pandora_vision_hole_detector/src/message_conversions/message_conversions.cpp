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

#include "message_conversions/message_conversions.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace vision
{
  /**
    @brief Extracts a PointCloudXYZPtr (see defines.h)
    from a point cloud message
    @param[in] msg [const sensor_msgs::PointCloud2ConstPtr&] The input point
    cloud message
    @param[out] pointCloudXYZ [PointCloudXYZPtr&] The extracted point cloud
    @return void
   **/
  void MessageConversions::extractPointCloudXYZFromMessage(
    const sensor_msgs::PointCloud2ConstPtr& msg,
    PointCloudXYZPtr& pointCloudXYZ)
  {
    PointCloud pointCloud;

    //!< convert the point cloud from sensor_msgs::PointCloud2ConstrPtr
    //!< to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*msg, pointCloud);

    //!< Convert the pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>::Ptr
    //!< aka PointCloudXYZPtr
    pcl::fromPCLPointCloud2 (pointCloud, *pointCloudXYZ);
  }



  /**
    @brief Extracts a PointCloudXYZPtr (see defines.h)
    from a point cloud message container
    @param[in] msg [const sensor_msgs::PointCloud2ConstPtr&] The input point
    cloud message
    @param[out] pointCloudXYZ [PointCloudXYZPtr&] The extracted point cloud
    @return void
   **/
  void MessageConversions::extractPointCloudXYZFromMessageContainer(
    const vision_communications::DepthCandidateHolesVectorMsg& msg,
    PointCloudXYZPtr& pointCloudXYZ)
  {
    PointCloud pointCloud;

    //!< convert the point cloud from sensor_msgs::PointCloud2ConstrPtr
    //!< to pcl::PCLPointCloud2
    pcl_conversions::toPCL(msg.pointCloud, pointCloud);

    //!< Convert the pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>::Ptr
    //!< aka PointCloudXYZPtr
    pcl::fromPCLPointCloud2 (pointCloud, *pointCloudXYZ);
  }



  /**
    @brief Converts a point cloud of type PointCloudXYZPtr to
    a point cloud of type PointCloud and packs it in a message
    @param[in] pointCloudXYZ [const PointCloudXYZPtr&] The point cloud to be
    converted
    @param[out] pointCloud [sensor_msgs::PointCloud2&]
    The converted point cloud message
    @return void
   **/
  void MessageConversions::convertPointCloudXYZToMessage(
    const PointCloudXYZPtr& pointCloudXYZPtr,
    sensor_msgs::PointCloud2& pointCloudMsg)
  {
    PointCloud pointCloud;

    //!< Convert the pcl::PointCloud<pcl::PointXYZ>::Ptr aka PointCloudXYZPtr
    //!< to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2(*pointCloudXYZPtr, pointCloud);

    //!< Pack the point cloud to a ROS message
    pcl_conversions::fromPCL(pointCloud, pointCloudMsg);
  }



  /**
    @brief Extracts a cv::Mat image from a ROS image message pointer
    @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS image
    message pointer
    @param[out] image [cv::Mat&] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessage(
    const sensor_msgs::ImageConstPtr& msg, cv::Mat& image,
    const std::string& encoding)
  {
    cv_bridge::CvImagePtr in_msg;

    in_msg = cv_bridge::toCvCopy(msg, encoding);

    image = in_msg->image.clone();
  }



  /**
    @brief Extracts a cv::Mat image from a ROS image message
    @param[in] msg [const sensor_msgs::Image&] The input ROS image
    message
    @param[out] image [cv::Mat&] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessage(
    const sensor_msgs::Image& msg, cv::Mat& image, const std::string& encoding)
  {
    cv_bridge::CvImagePtr in_msg;

    in_msg = cv_bridge::toCvCopy(msg, encoding);

    image = in_msg->image.clone();
  }



  /**
    @brief Extracts a cv::Mat image from a custom ROS message  of type
    vision_communications::DepthCandidateHolesVectorMsg
    containing the interpolated depth image
    @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
    @param[out] image [cv::Mat&] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractDepthImageFromMessageContainer(
    const vision_communications::DepthCandidateHolesVectorMsg& msg,
    cv::Mat& image, const std::string& encoding)
  {
    sensor_msgs::Image imageMsg = msg.interpolatedDepthImage;
    extractImageFromMessage(imageMsg, image, encoding);
  }



  /**
    @brief Extracts a cv::Mat image from a custom ROS message  of type
    vision_communications::RgbCandidateHolesVectorMsg
    containing the rgb image
    @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
    @param[out] image [cv::Mat&] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractRgbImageFromMessageContainer(
    const vision_communications::RgbCandidateHolesVectorMsg& msg,
    cv::Mat& image, const std::string& encoding)
  {
    sensor_msgs::Image imageMsg = msg.rgbImage;
    extractImageFromMessage(imageMsg, image, encoding);
  }



  /**
    @brief Extracts a RGB image from a point cloud message
    @param pointCloud[in] [const sensor_msgs::PointCloud2ConstPtr&]
    The input point cloud message
    @return cv::Mat The output rgb image
   **/
  cv::Mat MessageConversions::pointCloudToRGBImage(
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
