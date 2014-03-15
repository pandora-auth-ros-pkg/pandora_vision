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

#include "depth_node/depth.h"
#include "message_conversions/message_conversions.h"

namespace pandora_vision
{
  /**
    @brief Default constructor. Initiates communications, loads parameters.
    @return void
   **/
  PandoraKinect::PandoraKinect(void)
  {
    #ifdef DEBUG_TIME
    Timer::start("PandoraKinect");
    #endif

    ros::Duration(0.5).sleep();

    //!< Subscribe to the point cloud published by the
    //!< rgb_depth_synchronizer node
    _inputCloudSubscriber = _nodeHandle.subscribe(
      "/synchronized/camera/depth/points", 1,
      &PandoraKinect::inputCloudCallback, this);

    //!< Advertise the candidate holes found by the depth node
    _candidateHolesPublisher = _nodeHandle.advertise
      <vision_communications::DepthCandidateHolesVectorMsg>(
      "/synchronized/camera/depth/candidate_holes", 1000);

    //!< Advertise the point cloud's planes
    _planePublisher = _nodeHandle.advertise<PointCloudXYZ>
      ("/vision/kinect/planes", 1000);

    ROS_INFO("Depth node initiated");

    #ifdef DEBUG_TIME
    Timer::tick("PandoraKinect");
    #endif
  }



  /**
    @brief Default destructor
    @return void
   **/
  PandoraKinect::~PandoraKinect(void)
  {
    ROS_INFO("Depth node terminated");
  }



  /**
    @brief Callback for the point cloud
    @param msg [const sensor_msgs::PointCloud2ConstPtr&] The point cloud
    message
    @return void
   **/
  void PandoraKinect::inputCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    #ifdef DEBUG_TIME
    Timer::start("inputCloudCallback", "", true);
    #endif

    #ifdef DEBUG_SHOW
    ROS_INFO("Depth node callback");
    #endif

    //!< Extract a PointCloudXYZPtr from the point cloud message
    PointCloudXYZPtr pointCloudXYZ (new PointCloudXYZ);
    MessageConversions::extractPointCloudXYZFromMessage(msg, &pointCloudXYZ);

    //!< Extract the depth image from the PointCloudXYZPtr
    cv::Mat depthImage(pointCloudXYZ->height, pointCloudXYZ->width, CV_32FC1);
    extractDepthImageFromPointCloud(pointCloudXYZ, &depthImage);

    //!< Finds possible holes
    cv::Mat interpolatedDepthImage;
    HoleFilters::HolesConveyor holes = HoleDetector::findHoles(depthImage,
      interpolatedDepthImage);

    //!< Create the candidate holes message
    vision_communications::DepthCandidateHolesVectorMsg depthCandidateHolesMsg;

    createCandidateHolesMessage(holes,
      interpolatedDepthImage,
      pointCloudXYZ,
      &depthCandidateHolesMsg,
      sensor_msgs::image_encodings::TYPE_32FC1);

    //!< Publish the candidate holes message
    _candidateHolesPublisher.publish(depthCandidateHolesMsg);

    #ifdef DEBUG_TIME
    Timer::tick("inputCloudCallback");
    #endif

    return;
  }



  /**
    @brief Constructs a vision_communications/DepthCandidateHolesVectorMsg
    message
    @param[in] conveyor [HoleFilters::HolesConveyor&] A struct containing
    vectors of the holes' keypoints, bounding rectangles' vertices
    and blobs' outlines
    @param[in] interpolatedDepthImage [cv::Mat&] The denoised depth image
    @param[in] pointCloudXYZPtr [PointCloudXYZPtr&] The undistorted point
    cloud
    @param[out] depthCandidateHolesMsg
    [vision_communications::DepthCandidateHolesVectorMsg*] The output message
    @param[in] encoding [std::string&] The interpoladedDepth image's encoding
    @return void
   **/
  void PandoraKinect::createCandidateHolesMessage(
    const HoleFilters::HolesConveyor& conveyor,
    const cv::Mat& interpolatedDepthImage,
    const PointCloudXYZPtr& pointCloudXYZPtr,
    vision_communications::DepthCandidateHolesVectorMsg* depthCandidateHolesMsg,
    const std::string& encoding)
  {
    #ifdef DEBUG_TIME
    Timer::start("createCandidateHolesMessage", "inputCloudCallback");
    #endif

    //!< Fill the vision_communications::DepthCandidateHolesVectorMsg's
    //!< candidateHoles vector
    for (unsigned int i = 0; i < conveyor.keyPoints.size(); i++)
    {
      vision_communications::CandidateHoleMsg holeMsg;

      //!< Push back the keypoint
      holeMsg.keypointX = conveyor.keyPoints[i].pt.x;
      holeMsg.keypointY = conveyor.keyPoints[i].pt.y;

      //!< Push back the bounding rectangle's vertices
      for (int v = 0; v < conveyor.rectangles[i].size(); v++)
      {
        holeMsg.verticesX.push_back(conveyor.rectangles[i][v].x);
        holeMsg.verticesY.push_back(conveyor.rectangles[i][v].y);
      }

      //!< Push back the blob's outline points
      for (int o = 0; o < conveyor.outlines[i].size(); o++)
      {
        holeMsg.outlineX.push_back(conveyor.outlines[i][o].x);
        holeMsg.outlineY.push_back(conveyor.outlines[i][o].y);
      }

      //!< Push back one hole to the holes vector message
      depthCandidateHolesMsg->candidateHoles.push_back(holeMsg);
    }


    //!< Fill vision_communications::DepthCandidateHolesVectorMsg's
    //!< sensor_msgs/PointCloud2 pointCloud
    sensor_msgs::PointCloud2 pointCloudMessage;
    MessageConversions::convertPointCloudXYZToMessage(pointCloudXYZPtr,
      &pointCloudMessage);
    depthCandidateHolesMsg->pointCloud = pointCloudMessage;


    //!< Convert the cv::Mat image to a ROS message
    cv_bridge::CvImagePtr imageMessagePtr(new cv_bridge::CvImage());

    imageMessagePtr->header = pointCloudMessage.header;
    imageMessagePtr->encoding = encoding;
    imageMessagePtr->image = interpolatedDepthImage;

    //!< Fill vision_communications::DepthCandidateHolesVectorMsg's
    //!< sensor_msgs/Image interpolatedDepthImage
    depthCandidateHolesMsg->interpolatedDepthImage =
      *imageMessagePtr->toImageMsg();

    #ifdef DEBUG_TIME
    Timer::tick("createCandidateHolesMessage");
    #endif
  }



  /**
    @brief Extracts a CV_32FC1 depth image from a PointCloudXYZPtr
    point cloud
    @param pointCloudXYZ [const PointCloudXYZPtr&] The point cloud
    @param depthImage [cv::Mat*] The extracted depth image
    @return [cv::Mat] The depth image
   **/
  void PandoraKinect::extractDepthImageFromPointCloud(
    const PointCloudXYZPtr& pointCloudXYZ, cv::Mat* depthImage)
  {
    #ifdef DEBUG_TIME
    Timer::start("extractDepthImageFromPointCloud", "inputCloudCallback");
    #endif

    for (unsigned int row = 0; row < pointCloudXYZ->height; ++row)
    {
      for (unsigned int col = 0; col < pointCloudXYZ->width; ++col)
      {
        depthImage->at<float>(row, col) =
          pointCloudXYZ->points[col + pointCloudXYZ->width * row].z;

        //!< if element is nan make it a zero
        if (depthImage->at<float>(row, col) != depthImage->at<float>(row, col))
        {
          depthImage->at<float>(row, col) = 0.0;
        }
      }
    }

    #ifdef DEBUG_TIME
    Timer::tick("extractDepthImageFromPointCloud");
    #endif
  }



  /**
    @brief Stores a ensemble of point clouds in pcd images
    @param in_cloud [const std::vector<PointCloudXYZPtr>&] The point clouds
    @return void
   **/
  void PandoraKinect::storePointCloudVectorToImages(
    const std::vector<PointCloudXYZPtr>& in_vector)
  {
    #ifdef DEBUG_TIME
    Timer::start("storePointCloudVectorToImages");
    #endif

    for (int i = 0; i < in_vector.size(); i++)
    {
      pcl::io::savePCDFileASCII
        (boost::to_string(i) + "_.pcd", *in_vector[i]);
    }

    #ifdef DEBUG_TIME
    Timer::tick("storePointCloudVectorToImages");
    #endif
  }



  /**
  @brief Publishes the planes to /vision/kinect/planes topic
  @param cloudVector [const std::vector<PointCloudXYZPtr>&] The point clouds\
  containing the planes
  @return void
   **/
  void PandoraKinect::publishPlanes(
    const std::vector<PointCloudXYZPtr>& cloudVector)
  {
    #ifdef DEBUG_TIME
    Timer::start("publishPlanes");
    #endif

    PointCloudXYZPtr aggregatedPlanes (new PointCloudXYZ);

    for (unsigned int i = 0; i < cloudVector.size(); i++)
    {
      *aggregatedPlanes += *cloudVector[i];
    }

    aggregatedPlanes->header.frame_id = cloudVector[0]->header.frame_id;
    aggregatedPlanes->header.stamp = cloudVector[0]->header.stamp;
    _planePublisher.publish(aggregatedPlanes);

    #ifdef DEBUG_TIME
    Timer::tick("publishPlanes");
    #endif
  }

} // namespace pandora_vision
