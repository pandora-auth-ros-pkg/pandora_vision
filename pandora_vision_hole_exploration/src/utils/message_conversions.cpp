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
 * Authors: Alexandros Philotheou, Manos Tsardoulias, Vasilis Bosdelekidis
 *********************************************************************/

#include "utils/message_conversions.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @brief Converts a cv::Mat image into a sensor_msgs::Image message
    @param[in] image [const cv::Mat&] The image
    @param[in] encoding [const std::string&] The image message's encoding
    @param[in] msg [const sensor_msgs::Image&] A message needed for
    setting the output message's header by extracting its header
    @return [sensor_msgs::Image] The output image message
   **/
  sensor_msgs::Image MessageConversions::convertImageToMessage(
      const cv::Mat& image, const std::string& encoding,
      const sensor_msgs::Image& msg)
  {
    cv_bridge::CvImagePtr msgPtr(new cv_bridge::CvImage());

    msgPtr->header = msg.header;
    msgPtr->encoding = encoding;
    msgPtr->image = image;

    return *msgPtr->toImageMsg();
  }



  /**
    @brief Extracts an image from a point cloud message
    @param pointCloud[in] [const PointCloudPtr&]
    The input point cloud message
    @param[in] id [const int&] The enconding of the converted image.
    CV_32FC1 for a depth image, CV_8UC3 for a rgb image
    @return cv::Mat The output image
   **/
  cv::Mat MessageConversions::convertPointCloudMessageToImage(
      const PointCloudPtr& pointCloud, const int& encoding)
  {
#ifdef DEBUG_TIME
    Timer::start("convertPointCloudMessageToImage");
#endif

    // Prepare the output image
    cv::Mat image(pointCloud->height, pointCloud->width, encoding);

    // For the depth image
    if (encoding == CV_32FC1)
    {
      for (unsigned int row = 0; row < pointCloud->height; ++row)
      {
        for (unsigned int col = 0; col < pointCloud->width; ++col)
        {
          image.at<float>(row, col) =
            pointCloud->points[col + pointCloud->width * row].z;

          // if element is nan make it a zero
          if (image.at<float>(row, col) != image.at<float>(row, col))
          {
            image.at<float>(row, col) = 0.0;
          }
        }
      }
    }
    else if (encoding == CV_8UC3) // For the rgb image
    {
      for (unsigned int row = 0; row < pointCloud->height; ++row)
      {
        for (unsigned int col = 0; col < pointCloud->width; ++col)
        {
          image.at<unsigned char>(row, 3 * col + 2) =
            pointCloud->points[col + pointCloud->width * row].r;
          image.at<unsigned char>(row, 3 * col + 1) =
            pointCloud->points[col + pointCloud->width * row].g;
          image.at<unsigned char>(row, 3 * col + 0) =
            pointCloud->points[col + pointCloud->width * row].b;
        }
      }
    }

#ifdef DEBUG_TIME
    Timer::tick("convertPointCloudMessageToImage");
#endif

    return image;
  }



  /**
    @brief Constructs a pandora_vision_msgs/CandidateHolesVectorMsg
    message
    @param[in] conveyor [HolesConveyor&] A struct containing
    vectors of the holes' keypoints and bounding rectangles
    @param[out] candidateHolesVector
    [std::vector<pandora_vision_msgs::ExplorerCandidateHolesVectorMsg>*]
    The vector containing the conveyor's holes in
    pandora_vision_msgs::ExplorerCandidateHolesVectorMsg format
    @return void
   **/
  void MessageConversions::createCandidateHolesVector(
      const HolesConveyor& conveyor,
      std::vector<pandora_vision_msgs::ExplorerCandidateHoleMsg>* candidateHolesVector)
  {
#ifdef DEBUG_TIME
    Timer::start("createCandidateHolesVector");
#endif

    // Fill the pandora_vision_msgs::CandidateHolesVectorMsg's
    // candidateHoles vector
    for (unsigned int i = 0; i < conveyor.rectangle.size(); i ++)
    {
      pandora_vision_msgs::ExplorerCandidateHoleMsg holeMsg;

      // Push back the keypoint
      holeMsg.keypointX = conveyor.keypoint[i].x;
      holeMsg.keypointY = conveyor.keypoint[i].y;

      // Push back the bounding rectangle's upper left vertice
      holeMsg.verticeX = 
        (conveyor.keypoint[i].x - (conveyor.rectangle[i].width / 2) >= 0) ? 
        conveyor.keypoint[i].x - (conveyor.rectangle[i].width / 2) : 
        0;
      holeMsg.verticeY = (conveyor.keypoint[i].y - (conveyor.rectangle[i].height / 2) > 0) 
        ? conveyor.keypoint[i].y - (conveyor.rectangle[i].height / 2) 
        : 0;

      //// Push back the blob's outline points
      //for (int o = 0; o < conveyor.holes[i].outline.size(); o++)
      //{
      //  holeMsg.outlineX.push_back(conveyor.holes[i].outline[o].x);
      //  holeMsg.outlineY.push_back(conveyor.holes[i].outline[o].y);
      //}

      // Push back one hole to the holes vector message
      candidateHolesVector->push_back(holeMsg);
    }

#ifdef DEBUG_TIME
    Timer::tick("createCandidateHolesVector");
#endif
  }



  /**
    @brief Constructs a pandora_vision_msgs/ExplorerCandidateHolesVectorMsg
    message
    @param[in] conveyor [HolesConveyor&] A struct containing
    vectors of the holes' keypoints and bounding rectangles
    @param[in] image [cv::Mat&] The image to be packed in the message
    @param[out] candidateHolesVectorMsg
    [pandora_vision_msgs::CandidateHolesVectorMsg*] The output message
    @param[in] encoding [std::string&] The image's encoding
    @param[in] msg [const sensor_msgs::Image&] Needed to extract
    its header and place it as the header of the output message
    @return void
   **/
  void MessageConversions::createCandidateHolesVectorMessage(
      const HolesConveyor& conveyor,
      const cv::Mat& image,
      pandora_vision_msgs::ExplorerCandidateHolesVectorMsg* candidateHolesVectorMsg,
      const std::string& encoding,
      const sensor_msgs::Image& msg)
  {
#ifdef DEBUG_TIME
    Timer::start("createCandidateHolesVectorMessage");
#endif

    // Fill the pandora_vision_msgs::CandidateHolesVectorMsg's
    // candidateHoles vector
    std::vector<pandora_vision_msgs::ExplorerCandidateHoleMsg> candidateHolesVector;
    createCandidateHolesVector(conveyor, &candidateHolesVector);

    candidateHolesVectorMsg->explorerCandidateHoles = candidateHolesVector;

    // Fill the pandora_vision_msgs::CandidateHolesVectorMsg's
    // image
    candidateHolesVectorMsg->image =
      convertImageToMessage(image, encoding, msg);

    // Fill the pandora_vision_msgs::CandidateHolesVectorMsg's
    // header
    candidateHolesVectorMsg->header = msg.header;

#ifdef DEBUG_TIME
    Timer::tick("createCandidateHolesVectorMessage");
#endif
  }



  /**
    @brief Extracts a cv::Mat image from a ROS image message
    @param[in] msg [const sensor_msgs::Image&] The input ROS image
    message
    @param[out] image [cv::Mat*] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessage(
      const sensor_msgs::Image& msg,
      cv::Mat* image,
      const std::string& encoding)
  {
#ifdef DEBUG_TIME
    Timer::start("extractImageFromMessage");
#endif

    cv_bridge::CvImagePtr in_msg;

    in_msg = cv_bridge::toCvCopy(msg, encoding);

    *image = in_msg->image.clone();

#ifdef DEBUG_TIME
    Timer::tick("extractImageFromMessage");
#endif
  }



  /**
    @brief Extracts a cv::Mat image from a custom ROS message of type
    pandora_vision_msgs::CandidateHolesVectorMsg
    containing the interpolated depth image
    @param[in] msg [const sensor_msgs::ImageConstPtr&] The input ROS message
    @param[out] image [cv::Mat*] The output image
    @param[in] encoding [const std::string&] The image encoding
    @return void
   **/
  void MessageConversions::extractImageFromMessageContainer(
      const pandora_vision_msgs::ExplorerCandidateHolesVectorMsg& msg,
      cv::Mat* image, const std::string& encoding)
  {
#ifdef DEBUG_TIME
    Timer::start("extractDepthImageFromMessageContainer");
#endif

    sensor_msgs::Image imageMsg = msg.image;
    extractImageFromMessage(imageMsg, image, encoding);

#ifdef DEBUG_TIME
    Timer::tick("extractDepthImageFromMessageContainer");
#endif
  }



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
  void MessageConversions::fromCandidateHoleMsgToConveyor(
      const std::vector<pandora_vision_msgs::ExplorerCandidateHoleMsg>&
      candidateHolesVector,
      HolesConveyor* conveyor,
      const cv::Mat& inImage)
  {
#ifdef DEBUG_TIME
    Timer::start("fromCandidateHoleMsgToConveyor", "unpackMessage");
#endif
    std::vector<cv::Point2f> mc;
    std::vector<cv::Rect> boundRect;

    for (unsigned int i = 0; i < candidateHolesVector.size(); i++)
    {
      // Recreate the hole's keypoint
      cv::Point2f keypointTemp;
      keypointTemp.x = candidateHolesVector[i].keypointX;
      keypointTemp.y = candidateHolesVector[i].keypointY;
      cv::Rect rectTemp(
          candidateHolesVector[i].verticeX, 
          candidateHolesVector[i].verticeY, 
          (candidateHolesVector[i].verticeX 
           + 2 * (candidateHolesVector[i].keypointX - candidateHolesVector[i].verticeX) < inImage.cols) ? 
          2 * (candidateHolesVector[i].keypointX - candidateHolesVector[i].verticeX) :
          (inImage.cols - candidateHolesVector[i].verticeX),
          (candidateHolesVector[i].verticeY 
           + 2 * (candidateHolesVector[i].keypointY - candidateHolesVector[i].verticeY) < inImage.rows) ? 
          2 * (candidateHolesVector[i].keypointY - candidateHolesVector[i].verticeY) : 
          (inImage.rows - candidateHolesVector[i].verticeY));
      mc.push_back(keypointTemp);
      boundRect.push_back(rectTemp);
    }
    conveyor -> keypoint = mc;
    conveyor -> rectangle = boundRect;

#ifdef DEBUG_TIME
    Timer::tick("fromCandidateHoleMsgToConveyor");
#endif
  }



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
  void MessageConversions::unpackMessage(
      const pandora_vision_msgs::ExplorerCandidateHolesVectorMsg& holesMsg,
      HolesConveyor* conveyor,
      cv::Mat* image,
      const std::string& encoding)
  {
#ifdef DEBUG_TIME
    Timer::start("unpackMessage");
#endif

    // Unpack the image
    extractImageFromMessageContainer(holesMsg, image, encoding);

    // Recreate the conveyor
    fromCandidateHoleMsgToConveyor(
        holesMsg.explorerCandidateHoles,
        conveyor,
        *image);

#ifdef DEBUG_TIME
    Timer::tick("unpackMessage");
#endif
  }

} // namespace pandora_vision
