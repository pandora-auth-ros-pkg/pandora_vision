/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 *  THIS HARDWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS HARDWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *  Choutas Vassilis <vasilis4ch@gmail.com>
 *********************************************************************/


#include <string>
#include <limits>
#include <cmath>
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/transforms.h"
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_preprocessor.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  HardObstaclePreProcessor::
  HardObstaclePreProcessor() {}

  void
  HardObstaclePreProcessor::initialize(const std::string& ns,
      sensor_processor::Handler* handler)
  {
    sensor_processor::PreProcessor<sensor_msgs::PointCloud2, CVMatStamped>::
      initialize(ns, handler);

    reconfServerPtr_.reset( new dynamic_reconfigure::Server< ::pandora_vision_obstacle::elevation_mapConfig >(
          this->getProcessorNodeHandle()) );
    reconfServerPtr_->setCallback(boost::bind(&HardObstaclePreProcessor::reconfCallback,
          this, _1, _2));

    if (!this->getProcessorNodeHandle().getParam("ElevationMap/base_foot_print_id", baseFootPrintFrameId_))
    {
      ROS_ERROR_STREAM("[" + this->getName() + "] preprocessor nh processor : Could not "
          << "retrieve the name of the base Footprint Link!");
      ROS_BREAK();
    }
    if (!this->getProcessorNodeHandle().getParam("ElevationMap/kinect_frame_id", pclSensorFrameId_))
    {
      ROS_ERROR_STREAM("[" + this->getName() + "] preprocessor nh processor : Could not "
          << "retrieve the name of the PCL sensor Link!");
      ROS_BREAK();
    }

    tf::StampedTransform tfTransform;
    tfListener_.waitForTransform("/world", "/map", ros::Time(0), ros::Duration(1));
    tfListener_.lookupTransform("/world", "/map", ros::Time(0), tfTransform);
  }

  void HardObstaclePreProcessor::reconfCallback(const ::pandora_vision_obstacle::elevation_mapConfig params,
          const uint32_t& level)
  {
    maxAllowedDist_ = params.maxDist;
    minElevation_ = params.minElevation;
    maxElevation_ = params.maxElevation;
    elevationMapWidth_ = params.elevationMapWidth;
    elevationMapHeight_ = params.elevationMapHeight;
    visualisationFlag_ = params.visualisationFlag;
    gridResolution_ = params.gridResolution;
  }

  bool HardObstaclePreProcessor::preProcess(const PointCloud2ConstPtr& input,
      const CVMatStampedPtr& output)
  {
    NODELET_INFO("[%s] In preprocess", this->getName().c_str());

    // Convert the input Point Cloud to a local elevation map
    // in the form of a occupancy Grid Map
    if (!PointCloudToCvMat(input, output))
    {
      ROS_ERROR_STREAM("[" + this->getName() + "]: Could not convert the "
          "Point Cloud to an OpenCV matrix!");
      return false;
    }

    if (visualisationFlag_)
      viewElevationMap(output);

    return true;
  }

  void HardObstaclePreProcessor::viewElevationMap(const CVMatStampedPtr& elevationMapStamped)
  {
    if (elevationMapStamped->image.empty())
    {
      ROS_ERROR_STREAM("[" + this->getName() + "]: The elevation map cannot be displayed,"
          << " the input image is empty!");
      return;
    }
    cv::Mat elevationMapImg;
    cv::Mat colorMapImg;

    cv::Mat mask = elevationMapStamped->image != -std::numeric_limits<double>::max();
    // Normalize only the map elements that correspond to known cells.
    cv::normalize(elevationMapStamped->image, elevationMapImg, 0, 1, cv::NORM_MINMAX, -1, mask);
    // Set all unknown areas to 0 so that they appear as black.
    elevationMapImg.convertTo(elevationMapImg, CV_8UC3, 255);
    cv::applyColorMap(elevationMapImg, colorMapImg, cv::COLORMAP_JET);
    cv::bitwise_not(mask, mask);
    elevationMapImg.setTo(0.0, mask);


    cv::imshow("Elevation Map Image", colorMapImg);
    cv::waitKey(5);
    return;
  }

  /**
   * @brief Converts an Point Cloud to a local elevation map in OpenCV matrix
   * format.
   * @param inputPointCloud[const boost::shared_ptr<sensor_msgs::PointCloud2>&] The Point Cloud received
   * from the RGBD sensor.
   * @param outputImgPtr[const CVMatStampedPtr&] The resulting elevation
   * map as an OpenCV matrix.
   * @return bool True if the conversion was successful, false otherwise.
  */
  bool HardObstaclePreProcessor::PointCloudToCvMat(
      const PointCloud2ConstPtr& inputPointCloud,
      const CVMatStampedPtr& outputImgPtr)
  {
    ROS_DEBUG_STREAM("[" + this->getName() + "]: Received a new Point Cloud Message");
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensorPointCloudPtr(new pcl::PointCloud<pcl::PointXYZ> ());
    tf::StampedTransform baseFootPrintTf;

    // converting PointCloud2 msg format to pcl pointcloud format in order to read the 3d data
    pcl::fromROSMsg(*inputPointCloud, *sensorPointCloudPtr);

    try
    {
      tfListener_.waitForTransform(baseFootPrintFrameId_,
          inputPointCloud->header.frame_id, inputPointCloud->header.stamp, ros::Duration(0.2));
      pcl_ros::transformPointCloud(baseFootPrintFrameId_, *sensorPointCloudPtr, *mapPointCloudPtr, tfListener_);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR_STREAM("[" << this->getName() << "]:" << ex.what());
      ROS_DEBUG_STREAM("[" << this->getName() << "]: PointCloud Transform failed");
      return false;
    }

    // Create the output Elevation Map
    outputImgPtr->image = cv::Mat(elevationMapHeight_, elevationMapWidth_, CV_64FC1);
    outputImgPtr->image.setTo(-std::numeric_limits<double>::max());
    outputImgPtr->header = inputPointCloud->header;

    for (int ii = 0; ii < sensorPointCloudPtr->size(); ++ii)
    {
      pcl::PointXYZ& currentPoint = mapPointCloudPtr->points[ii];
      double measuredDist = sensorPointCloudPtr->points[ii].z;

      // Check if the current point has any NaN value.
      if (isnan(currentPoint.x) || isnan(currentPoint.y) || isnan(currentPoint.z))
        continue;

      // Check if the point is in the allowed distance range.
      if (measuredDist > maxAllowedDist_)
        continue;

      // Check if the z coordinate is within the specified range.
      if (currentPoint.z < minElevation_ || currentPoint.z > maxElevation_)
        continue;

      // If the current point is located higher that the higher point of the corresponding cell
      // then substitute it.
      if (currentPoint.z >
          outputImgPtr->image.at<double>(convertToYCoord(currentPoint.y), convertToXCoord(currentPoint.x)))
        outputImgPtr->image.at<double>(convertToYCoord(currentPoint.y),
            convertToXCoord(currentPoint.x)) = currentPoint.z;
    }
    return true;
  }

  int
  HardObstaclePreProcessor::
  convertToXCoord(double meters)
  {
    return static_cast<int>(floor(meters / gridResolution_ + static_cast<double>(elevationMapWidth_) / 2));
  }

  int
  HardObstaclePreProcessor::
  convertToYCoord(double meters)
  {
    return static_cast<int>(floor(static_cast<double>(elevationMapHeight_) / 2 - meters / gridResolution_));
  }

}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
