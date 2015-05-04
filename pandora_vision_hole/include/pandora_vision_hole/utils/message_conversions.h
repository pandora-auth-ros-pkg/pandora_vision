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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef UTILS_MESSAGE_CONVERSIONS_H
#define UTILS_MESSAGE_CONVERSIONS_H

#include "std_msgs/Header.h"
#include "sensor_msgs/Image.h"

#include "utils/defines.h"
#include "utils/outline_discovery.h"
#include "utils/blob_vector.h"
#include "pandora_vision_msgs/BlobVector.h"

/**
  @namespace pandora_vision
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
        @brief Converts a cv::Mat image into a sensor_msgs::Image message
        @param[in] image [const cv::Mat&] The image
        @param[in] encoding [const std::string&] The image message's encoding
        @param[in] header [const std_msgs::Header&] header
        @return [sensor_msgs::Image] The output image message
       **/
      static sensor_msgs::Image
        convertImageToMessage(
            const cv::Mat& image, const std::string& encoding,
            const std_msgs::Header& header);

      /**
        @brief Extracts an image from a point cloud message
        @param pointCloud[in] [const PointCloudPtr&]
        The input point cloud message
        @param[in] id [const int&] The enconding of the converted image.
        CV_32FC1 for depth image, CV_8UC3 for rgb image
        @return cv::Mat The output image
       **/
      static cv::Mat
        convertPointCloudMessageToImage(
            const PointCloudPtr& pointCloud, const int& encoding);

      /**
        @brief Extracts a cv::Mat image from a ROS image message
        @param[in] msg [const sensor_msgs::Image&] The input ROS image
        message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/
      static void
        extractImageFromMessage(
            const sensor_msgs::Image& msg, cv::Mat* image,
            const std::string& encoding);

      /**
        @brief Extracts a cv::Mat image from a custom ROS message  of type
        pandora_vision_msgs::CandidateHolesVectorMsg
        containing the interpolated depth image
        @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
        @param[out] image [cv::Mat*] The output image
        @param[in] encoding [const std::string&] The image encoding
        @return void
       **/

      /**
        @brief Recreates the HolesConveyor struct for the candidate holes
        from the pandora_vision_msgs::CandidateHolerMsg message
        @param[in] candidateHolesVector
        [const std::vector<pandora_vision_msgs::CandidateHoleMsg>&]
        The input candidate holes
        @param[out] conveyor [HolesConveyor*] The output conveyor
        struct
        @param[in] inImage [const cv::Mat&] An image used for its size.
        It is needed if the wavelet method is used in the keypoints' extraction,
        in order to obtain the coherent shape of holes' outline points
        @param[in] representationMethod [const int&] The @param inImage
        representation method. 0 for normal mode, 1 for wavelet mode
        @param[in] raycastKeypointPartitions [const int&] The number of rays
        used, if @param representationMethod = 1, in order to recreate a
        blob's outline
        @return void
       **/

      // TODO
      /**
        @brief Unpacks the the HolesConveyor struct for the
        candidate holes, the interpolated depth image or the RGB image
        from the pandora_vision_msgs::CandidateHolesVectorMsg message
        @param[in] holesMsg
        [pandora_vision_msgs::CandidateHolesVectorMsg&] The input
        candidate holes message obtained through the depth node
        @param[out] conveyor [HolesConveyor*] The output conveyor
        struct
        @param[out] image [cv::Mat*] The output image
        @param[in] representationMethod [const int&] The @param inImage
        representation method. 0 for normal mode, 1 for wavelet mode
        @param[in] encoding [const std::string&] The encoding used for
        @param[in] raycastKeypointPartitions [const int&] The number of rays
        used, if @param representationMethod = 1, in order to recreate a
        blob's outline
        @return void
       **/

    static pandora_vision_msgs::Keypoint
      cvToMsg(const cv::Point2f& point);

    static cv::Point2f
      msgToCv(const pandora_vision_msgs::Keypoint& point);

    static pandora_vision_msgs::Keypoint
      keypointToMsg(const cv::KeyPoint& point);

    static cv::KeyPoint
      msgToKeypoint(const pandora_vision_msgs::Keypoint& point);
    
    static std::vector<pandora_vision_msgs::Keypoint>
      vecToMsg(const std::vector<cv::Point2f>& vec);

    static pandora_vision_msgs::AreaOfInterest
      vecToArea(const std::vector<cv::Point2f>& vert);
    
    static std::vector<cv::Point2f>
      areaToVec(const pandora_vision_msgs::AreaOfInterest& area);
  };

} // namespace pandora_vision

#endif  // UTILS_MESSAGE_CONVERSIONS_H
