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
#include "nav_msgs/OccupancyGrid.h"
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_preprocessor.h"

namespace pandora_vision
{
  HardObstaclePreProcessor::HardObstaclePreProcessor(const std::string& ns,
      sensor_processor::Handler* handler) : sensor_processor::PreProcessor<sensor_msgs::PointCloud2,
  CVMatStamped>(ns, handler)
  {
    ROS_INFO_STREAM("[" + this->getName() + "] preprocessor nh processor : " +
        this->accessProcessorNh()->getNamespace());
  }

  HardObstaclePreProcessor::~HardObstaclePreProcessor()
  {
  }

  bool HardObstaclePreProcessor::preProcess(const PointCloud2ConstPtr& input,
      const CVMatStampedPtr& output)
  {
    ROS_DEBUG_STREAM("["+this->accessPublicNh()->getNamespace()+"] In preprocessor!");

    // Convert the input Point Cloud to a local elevation map
    // in the form of a occupancy Grid Map
    if (!PointCloudToCvMat(input, output))
    {
      ROS_ERROR_STREAM("[" + this->accessPublicNh()->getNamespace() + "]: Could not convert the "
          "Point Cloud to an OpenCV matrix!");
      return false;
    }

    return true;
  }

  /**
   * @brief Converts an Point Cloud to a local elevation map in OpenCV matrix
   * format.
   * @param inputPointCloud[const PointCloud2ConstPtr&] The Point Cloud received
   * from the RGBD sensor.
   * @param outputImgPtr[const CVMatStampedPtr&] The resulting elevation
   * map as an OpenCV matrix.
   * @return bool True if the conversion was successful, false otherwise.
  */
  bool HardObstaclePreProcessor::PointCloudToCvMat(
      const PointCloud2ConstPtr& inputPointCloud,
      const CVMatStampedPtr& outputImgPtr)
  {
    return true;
  }

}  // namespace pandora_vision
