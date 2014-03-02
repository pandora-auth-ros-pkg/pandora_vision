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

namespace vision
{
  /**
    @brief The HoleFusion constructor
   **/
  HoleFusion::HoleFusion(void)
  {
    //!< Advertise the topic that the rgb_depth_synchronizer will be
    //!< subscribed to in order for the hole_fusion_node to unlock it
    unlockPublisher_ = nodeHandle_.advertise <std_msgs::Empty>
      ("/vision/hole_fusion/unlock_rgb_depth_synchronizer", 1000, true);

    //!< Subscribe to the topic where the depth node publishes
    //!< candidate holes
    depthCandidateHolesSubscriber_= nodeHandle_.subscribe(
      "/synchronized/camera/depth/candidate_holes", 1,
      &HoleFusion::depthCandidateHolesCallback, this);

    ROS_INFO("HoleFusion node initiated");
  }



  /**
    @brief The HoleFusion deconstructor
   **/
  HoleFusion::~HoleFusion(void)
  {
    ROS_INFO("HoleFusion node terminated");
  }



  /**
    @brief Requests from the synchronizer to process a new point cloud
    @return void
   **/
  void HoleFusion::unlockSynchronizer()
  {
    std_msgs::Empty unlockMsg;
    unlockPublisher_.publish(unlockMsg);
  }



  /**
    @brief Callback for the candidate holes via the depth node
    @param depthCandidateHolesVector
    [const vision_communications::DepthCandidateHolesVectorMsg&]
    The message containing the necessary information to filter hole
    candidates acquired through the depth node
    @return void
   **/
  void HoleFusion::depthCandidateHolesCallback(
    const vision_communications::DepthCandidateHolesVectorMsg&
    depthCandidateHolesVector)
  {
    //!< Recreate the conveyor
    HoleFilters::HolesConveyor conveyor;
    fromDepthMessageToConveyor(depthCandidateHolesVector, conveyor);

    //!< Unpack the interpolated depth image
    cv::Mat interpolatedDepthImage;
    MessageConversions::extractImageFromMessageContainer(
      depthCandidateHolesVector, interpolatedDepthImage,
      sensor_msgs::image_encodings::TYPE_32FC1);

    //!< Unpack the point cloud
    PointCloudXYZPtr pointCloudXYZ(new PointCloudXYZ);
    MessageConversions::extractPointCloudXYZFromMessageContainer(
      depthCandidateHolesVector, pointCloudXYZ);

    //!< check holes for debugging purposes
    HoleFilters::checkHoles(
      interpolatedDepthImage,
      pointCloudXYZ,
      conveyor);


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
          interpolatedDepthImage,
          -1,
          conveyor.keyPoints)
        );
    }
    if(HoleFusionParameters::debug_show_find_holes)
    {
      Visualization::multipleShow("depthCandidateHolesCallback function",
        imgs, msgs, HoleFusionParameters::debug_show_find_holes_size,1);
    }
    #endif
  }



  /**
    @brief Recreates the HoleFilters::HolesConveyor struct for the
    candidate holes from the
    vision_communications::DepthCandidateHolesVectorMsg message
    @param[in] holesMsg
    [vision_communications::DepthCandidateHolesVectorMsg&] The input
    depth candidate holes
    @param[out] conveyor [HoleFilters::HolesConveyor&] The output conveyor
    struct
    @return void
   **/
  void HoleFusion::fromDepthMessageToConveyor(
    const vision_communications::DepthCandidateHolesVectorMsg& holesMsg,
    HoleFilters::HolesConveyor& conveyor)
  {
    for (unsigned int i = 0; i < holesMsg.candidateHoles.size(); i++)
    {
      //!< Recreate conveyor.keypoints
      cv::KeyPoint holeKeypoint;
      holeKeypoint.pt.x = holesMsg.candidateHoles[i].keypointX;
      holeKeypoint.pt.y = holesMsg.candidateHoles[i].keypointY;
      conveyor.keyPoints.push_back(holeKeypoint);

      //!< Recreate conveyor.rectangles
      std::vector<cv::Point2f> renctangleVertices;
      for (unsigned int v = 0;
        v < holesMsg.candidateHoles[i].verticesX.size(); v++)
      {
        cv::Point2f vertex;
        vertex.x = holesMsg.candidateHoles[i].verticesX[v];
        vertex.y = holesMsg.candidateHoles[i].verticesY[v];
        renctangleVertices.push_back(vertex);
      }
      conveyor.rectangles.push_back(renctangleVertices);

      //!< Recreate conveyor.outlines
      std::vector<cv::Point> outlinePoints;
      for (unsigned int o = 0;
        o < holesMsg.candidateHoles[i].outlineX.size(); o++)
      {
        cv::Point outlinePoint;
        outlinePoint.x = holesMsg.candidateHoles[i].outlineX[o];
        outlinePoint.y = holesMsg.candidateHoles[i].outlineY[o];
        outlinePoints.push_back(outlinePoint);
      }
      conveyor.outlines.push_back(outlinePoints);
    }
  }
}
