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

#include "hole_fusion_node/hole_fusion.h"

namespace pandora_vision
{
  /**
    @brief The HoleFusion constructor
   **/
  HoleFusion::HoleFusion(void) : pointCloudXYZ_(new PointCloudXYZ)
  {
    #ifdef DEBUG_TIME
    Timer::start("HoleFusion");
    #endif

    ros::Duration(0.5).sleep();

    //!< Initialize the numNodesReady variable
    numNodesReady_ = 0;

    //!< Advertise the topic that the rgb_depth_synchronizer will be
    //!< subscribed to in order for the hole_fusion_node to unlock it
    unlockPublisher_ = nodeHandle_.advertise <std_msgs::Empty>
      ("/vision/hole_fusion/unlock_rgb_depth_synchronizer", 1000, true);

    //!< Subscribe to the topic where the depth node publishes
    //!< candidate holes
    depthCandidateHolesSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/depth/candidate_holes", 1,
      &HoleFusion::depthCandidateHolesCallback, this);

    //!< Subscribe to the topic where the rgb node publishes
    //!< candidate holes
    rgbCandidateHolesSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/rgb/candidate_holes", 1,
      &HoleFusion::rgbCandidateHolesCallback, this);

    ROS_INFO("HoleFusion node initiated");

    //!< Start the synchronizer
    unlockSynchronizer();

    #ifdef DEBUG_TIME
    Timer::tick("HoleFusion");
    #endif
  }



  /**
    @brief The HoleFusion deconstructor
   **/
  HoleFusion::~HoleFusion(void)
  {
    ROS_INFO("HoleFusion node terminated");
  }



  /**
    @brief Callback for the candidate holes via the depth node
    @param[in] depthCandidateHolesVector
    [const vision_communications::DepthCandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the depth node
    @return void
   **/
  void HoleFusion::depthCandidateHolesCallback(
    const vision_communications::DepthCandidateHolesVectorMsg&
    depthCandidateHolesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("depthCandidateHolesCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Hole Fusion Depth callback");
    #endif

    //!< Clear the current depthHolesConveyor struct
    //!< (or else keyPoints, rectangles and outlines accumulate)
    depthHolesConveyor_.keyPoints.clear();
    depthHolesConveyor_.rectangles.clear();
    depthHolesConveyor_.outlines.clear();

    //!< Unpack the message
    unpackDepthMessage(depthCandidateHolesVector,
      &depthHolesConveyor_,
      &pointCloudXYZ_,
      &interpolatedDepthImage_);

    #ifdef DEBUG_SHOW
    if (HoleFusionParameters::debug_show_find_holes)
    {
      Visualization::showScaled(
        "interpolated depth image arrived in Hole Fusion node",
        interpolatedDepthImage_, 1);
    }
    #endif

    numNodesReady_++;

    //!< If both the RGB and the depth nodes are ready
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 2)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("depthCandidateHolesCallback");
    #endif
  }



  /**
    @brief Callback for the candidate holes via the rgb node
    @param[in] depthCandidateHolesVector
    [const vision_communications::RgbCandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the rgb node
    @return void
   **/
  void HoleFusion::rgbCandidateHolesCallback(
    const vision_communications::RgbCandidateHolesVectorMsg&
    rgbCandidateHolesVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("rgbCandidateHolesCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Hole Fusion RGB callback");
    #endif

    //!< Clear the current rgbHolesConveyor struct
    //!< (or else keyPoints, rectangles and outlines accumulate)
    rgbHolesConveyor_.keyPoints.clear();
    rgbHolesConveyor_.rectangles.clear();
    rgbHolesConveyor_.outlines.clear();

    //!< Unpack the message
    unpackRgbMessage(rgbCandidateHolesVector,
      &rgbHolesConveyor_,
      &rgbImage_);

    #ifdef DEBUG_SHOW
    if (HoleFusionParameters::debug_show_find_holes)
    {
      Visualization::showScaled(
        "RGB image arrived in Hole Fusion node", rgbImage_, 1);
    }
    #endif

    numNodesReady_++;

    //!< If both the RGB and the depth nodes are ready
    //!< unlock the rgb_depth_synchronizer and process the candidate holes
    //!< from both sources
    if (numNodesReady_ == 2)
    {
      numNodesReady_ = 0;

      unlockSynchronizer();

      processCandidateHoles();
    }

    #ifdef DEBUG_TIME
    Timer::tick("rgbCandidateHolesCallback");
    #endif
  }



  /**
    @brief Recreates the HoleFilters::HolesConveyor struct for the
    candidate holes from the
    vision_communications::CandidateHolerMsg message
    @param[in]candidateHolesVector
    [const std::vector<vision_communications::CandidateHoleMsg>&]
    The input candidate holes
    @param[out] conveyor [HoleFilters::HolesConveyor*] The output conveyor
    struct
    @return void
   **/
  void HoleFusion::fromCandidateHoleMsgToConveyor(
    const std::vector<vision_communications::CandidateHoleMsg>&
    candidateHolesVector,
    HoleFilters::HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("fromCandidateHoleMsgToConveyor");
    #endif

    for (unsigned int i = 0; i < candidateHolesVector.size(); i++)
    {
      //!< Recreate conveyor.keypoints
      cv::KeyPoint holeKeypoint;
      holeKeypoint.pt.x = candidateHolesVector[i].keypointX;
      holeKeypoint.pt.y = candidateHolesVector[i].keypointY;
      conveyor->keyPoints.push_back(holeKeypoint);

      //!< Recreate conveyor.rectangles
      std::vector<cv::Point2f> renctangleVertices;
      for (unsigned int v = 0;
        v < candidateHolesVector[i].verticesX.size(); v++)
      {
        cv::Point2f vertex;
        vertex.x = candidateHolesVector[i].verticesX[v];
        vertex.y = candidateHolesVector[i].verticesY[v];
        renctangleVertices.push_back(vertex);
      }
      conveyor->rectangles.push_back(renctangleVertices);

      //!< Recreate conveyor.outlines
      std::vector<cv::Point> outlinePoints;
      for (unsigned int o = 0;
        o < candidateHolesVector[i].outlineX.size(); o++)
      {
        cv::Point outlinePoint;
        outlinePoint.x = candidateHolesVector[i].outlineX[o];
        outlinePoint.y = candidateHolesVector[i].outlineY[o];
        outlinePoints.push_back(outlinePoint);
      }
      conveyor->outlines.push_back(outlinePoints);
    }

    #ifdef DEBUG_TIME
    Timer::tick("fromCandidateHoleMsgToConveyor");
    #endif
  }



  /**
    @brief Waits for both hole sources(rgb and depth nodes) to have sent
    their candidate holes and then it implements a strategy to combine
    information from both sources in order to accurately find valid holes
    @return void
   **/
  void HoleFusion::processCandidateHoles()
  {
    #ifdef DEBUG_SHOW
    ROS_INFO ("numNodesReady: %d", numNodesReady_);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Processing candidate holes");
    #endif

    #ifdef DEBUG_TIME
    Timer::start("processCandidateHoles", "depthCandidateHolesCallback");
    #endif

    //!< Do some processing

    //!< check holes for debugging purposes
    DepthFilters::checkHoles(
      interpolatedDepthImage_,
      pointCloudXYZ_,
      &depthHolesConveyor_);


    #ifdef DEBUG_SHOW
    std::vector<std::string> msgs;
    std::vector<cv::Mat> imgs;
    if(HoleFusionParameters::debug_show_find_holes) // Debug
    {
      std::string msg = LPATH( STR(__FILE__)) + STR(" ") + TOSTR(__LINE__);
      msg += STR(" : Final keypoints");
      msgs.push_back(msg);
      imgs.push_back(
        Visualization::showKeypoints(
          msg,
          interpolatedDepthImage_,
          -1,
          depthHolesConveyor_.keyPoints)
        );
    }
    if(HoleFusionParameters::debug_show_find_holes)
    {
      Visualization::multipleShow("depthCandidateHolesCallback function",
        imgs, msgs, HoleFusionParameters::debug_show_find_holes_size,1);
    }
    #endif
    //!< Processing complete.

    #ifdef DEBUG_TIME
    Timer::tick("processCandidateHoles");
    Timer::printAllMeansTree();
    #endif
  }



  /**
    @brief Unpacks the the HoleFilters::HolesConveyor struct for the
    candidate holes, the interpolated depth image and the point cloud
    from the vision_communications::DepthCandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::DepthCandidateHolesVectorMsg&] The input
    candidate holes message obtained through the depth node
    @param[out] conveyor [HoleFilters::HolesConveyor*] The output conveyor
    struct
    @param[out] pointCloudXYZ [PointCloudXYZPtr*] The output point cloud
    @param[out] interpolatedDepthImage [cv::Mat*] The output interpolated
    depth image
    @return void
   **/
  void HoleFusion::unpackDepthMessage(
    const vision_communications::DepthCandidateHolesVectorMsg& holesMsg,
    HoleFilters::HolesConveyor* conveyor, PointCloudXYZPtr* pointCloudXYZ,
    cv::Mat* interpolatedDepthImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackDepthMessage", "depthCandidateHolesCallback");
    #endif

    //!< Recreate the conveyor
    fromCandidateHoleMsgToConveyor(holesMsg.candidateHoles, conveyor);

    //!< Unpack the interpolated depth image
    MessageConversions::extractDepthImageFromMessageContainer(
      holesMsg,
      interpolatedDepthImage,
      sensor_msgs::image_encodings::TYPE_32FC1);

    //!< Unpack the point cloud
    MessageConversions::extractPointCloudXYZFromMessageContainer(holesMsg,
      pointCloudXYZ);

    #ifdef DEBUG_TIME
    Timer::tick("unpackDepthMessage");
    #endif
  }



  /**
    @brief Unpacks the the HoleFilters::HolesConveyor struct for the
    candidate holes, the RGB image
    from the vision_communications::DepthCandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::RgbCandidateHolesVectorMsg&] The input
    candidate holes message obtained throught the RGB node
    @param[out] conveyor [HoleFilters::HolesConveyor*] The output conveyor
    struct
    @param[out] rgbImage [cv::Mat*] The output RGB image
    @return void
   **/
  void HoleFusion::unpackRgbMessage(
    const vision_communications::RgbCandidateHolesVectorMsg& holesMsg,
    HoleFilters::HolesConveyor* conveyor, cv::Mat* rgbImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("unpackRgbMessage", "rgbCandidateHolesCallback");
    #endif

    //!< Recreate the conveyor
    fromCandidateHoleMsgToConveyor(holesMsg.candidateHoles, conveyor);

    //!< Unpack the RGB image
    MessageConversions::extractRgbImageFromMessageContainer(
      holesMsg,
      rgbImage,
      sensor_msgs::image_encodings::TYPE_32FC3);

    #ifdef DEBUG_TIME
    Timer::tick("unpackRgbMessage");
    #endif
  }



  /**
    @brief Requests from the synchronizer to process a new point cloud
    @return void
   **/
  void HoleFusion::unlockSynchronizer()
  {
    #ifdef DEBUG_SHOW
    ROS_INFO("Sending unlock message");
    #endif

    std_msgs::Empty unlockMsg;
    unlockPublisher_.publish(unlockMsg);
  }
}
