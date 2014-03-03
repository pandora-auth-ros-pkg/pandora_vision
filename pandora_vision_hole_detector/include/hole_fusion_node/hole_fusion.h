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

#ifndef HOLE_FUSION_H
#define HOLE_FUSION_H

#include <std_msgs/Empty.h>
#include "vision_communications/DepthCandidateHolesVectorMsg.h"
#include "message_conversions/message_conversions.h"
#include <depth_node/defines.h>
#include "depth_node/depth_parameters.h"
#include "hole_fusion_node/depth_filters.h"
#include "hole_fusion_node/hole_fusion_parameters.h"
/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace vision
{
  class HoleFusion
  {
    private:

      //!< The ROS node handle
      ros::NodeHandle nodeHandle_;

      //!< The parameters needed for the hole fusion node
      HoleFusionParameters params;

      //!< The ROS publisher that will be used for unlocking the
      //!< synchronizer_node
      ros::Publisher unlockPublisher_;

      //!< The ROS subscriber for acquisition of candidate holes through the
      //!< depth node
      ros::Subscriber depthCandidateHolesSubscriber_;

      /**
        @brief Requests from the synchronizer to process a new point cloud
        @return void
       **/
      void unlockSynchronizer();

      /**
        @brief Callback for the candidate holes via the depth node
        @param depthCandidateHolesVector
        [const vision_communications::DepthCandidateHolesVectorMsg&]
        The message containing the necessary information to filter hole
        candidates acquired through the depth node
        @return void
       **/
      void depthCandidateHolesCallback(
        const vision_communications::DepthCandidateHolesVectorMsg&
        depthCandidateHolesVector);

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
      void fromDepthMessageToConveyor(
        const vision_communications::DepthCandidateHolesVectorMsg& holesMsg,
        HoleFilters::HolesConveyor& conveyor);

      /**
       @brief Unpacks the the HoleFilters::HolesConveyor struct for the
       candidate holes, the interpolated depth image and the point cloud
       from the vision_communications::DepthCandidateHolesVectorMsg message
       @param[in] holesMsg
       [vision_communications::DepthCandidateHolesVectorMsg&] The input
       depth candidate holes message
       @param[out] conveyor [HoleFilters::HolesConveyor&] The output conveyor
       struct
       @param[out] pointCloudXYZ [PointCloudXYZPtr&] The output point cloud
       @param[out] interpolatedDepthImage [cv::Mat&] The output interpolated
       depth image
       @return void
       **/
      void unpackDepthMessage(
        const vision_communications::DepthCandidateHolesVectorMsg& holesMsg,
        HoleFilters::HolesConveyor& conveyor, PointCloudXYZPtr& pointCloudXYZ,
        cv::Mat& interpolatedDepthImage);

    public:

      /**
        @brief The HoleFusion constructor
       **/
      HoleFusion(void);

      /**
        @brief The HoleFusion deconstructor
       **/
      ~HoleFusion(void);
  };
}
#endif
