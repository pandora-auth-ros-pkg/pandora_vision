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

#ifndef UTILS_MESSAGE_CONVERSIONS_H
#define UTILS_MESSAGE_CONVERSIONS_H

#include "utils/defines.h"
#include "vision_communications/DepthCandidateHolesVectorMsg.h"
#include "vision_communications/RgbCandidateHolesVectorMsg.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class MessageConversions
    @brief Provides methods for converting images and point clouds
    from and to ROS messages
   **/
  class MessageConversions
  {
    public:

      /**
        @brief Extracts a PointCloudXYZPtr (see defines.h)
        from a point cloud message
        @param[in] msg [const sensor_msgs::PointCloud2ConstPtr&] The input point
        cloud message
        @param[out] pointCloudXYZ [PointCloudXYZPtr*] The extracted point cloud
        @return void
       **/
      static void extractPointCloudXYZFromMessage(
        const sensor_msgs::PointCloud2ConstPtr& msg,
        PointCloudXYZPtr* pointCloudXYZ);

      /**
        @brief Extracts a PointCloudXYZPtr (see defines.h)
        from a point cloud message container
        @param[in] msg [const sensor_msgs::PointCloud2ConstPtr&] The input point
        cloud message
        @param[out] pointCloudXYZ [PointCloudXYZPtr*] The extracted point cloud
        @return void
       **/
      static void extractPointCloudXYZFromMessageContainer(
        const vision_communications::DepthCandidateHolesVectorMsg& msg,
        PointCloudXYZPtr* pointCloudXYZ);

      /**
        @brief Converts a point cloud of type PointCloudXYZPtr to
        a point cloud of type PointCloud and packs it in a message
        @param[in] pointCloudXYZ [const PointCloudXYZPtr&] The point cloud to be
        converted
        @param[out] pointCloud [sensor_msgs::PointCloud2*]
        The converted point cloud message
        @return void
       **/
      static void convertPointCloudXYZToMessage(
        const PointCloudXYZPtr& pointCloudXYZPtr,
        sensor_msgs::PointCloud2* pointCloudMsg);

      /**
        @brief Extracts a cv::Mat image from a ROS image message pointer
        @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS image
        message pointer
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractImageFromMessage(
        const sensor_msgs::ImageConstPtr& msg, cv::Mat* image,
        const std::string& encoding);

      /**
        @brief Extracts a cv::Mat image from a ROS image message
        @param[in] msg [const sensor_msgs::Image&] The input ROS image
        message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractImageFromMessage(
        const sensor_msgs::Image& msg, cv::Mat* image,
        const std::string& encoding);

      /**
        @brief Extracts a cv::Mat image from a custom ROS message  of type
        vision_communications::DepthCandidateHolesVectorMsg
        containing the interpolated depth image
        @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractDepthImageFromMessageContainer(
        const vision_communications::DepthCandidateHolesVectorMsg& msg,
        cv::Mat* image, const std::string& encoding);

      /**
        @brief Extracts a cv::Mat image from a custom ROS message  of type
        vision_communications::RgbCandidateHolesVectorMsg
        containing the rgb image
        @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void extractRgbImageFromMessageContainer(
        const vision_communications::RgbCandidateHolesVectorMsg& msg,
        cv::Mat* image, const std::string& encoding);

      /**
        @brief Extracts a RGB image from a point cloud message
        @param[in] pointCloud [const sensor_msgs::PointCloud2ConstPtr&]
        The input point cloud message
        @return cv::Mat The output rgb image
       **/
      static cv::Mat pointCloudToRGBImage(
        const sensor_msgs::PointCloud2ConstPtr& pointCloudMessage);
  };

} // namespace pandora_vision

#endif  // UTILS_MESSAGE_CONVERSIONS_H
