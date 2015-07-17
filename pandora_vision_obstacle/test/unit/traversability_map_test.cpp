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
 * Authors:
 *  Choutas Vassilis <vasilis4ch@gmail.com>
 *  Kofinas Miltiadis <mkofinas@gmail.com>
 *********************************************************************/

#include <vector>
#include <limits>
#include <gtest/gtest.h>
#include <time.h>
#include "pandora_vision_obstacle/hard_obstacle_detection/hard_obstacle_detector.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class TraversabilityMapTest: public ::testing::Test
  {
    public:
      TraversabilityMapTest(){}

      virtual void SetUp()
      {
        // Define the description of the robot.
        boost::shared_ptr<RobotGeometryMaskDescription> descriptionPtr;
        descriptionPtr.reset(new RobotGeometryMaskDescription);
        descriptionPtr->wheelH = 0;
        descriptionPtr->barrelH = 0.06;
        descriptionPtr->robotH = 0.08;
        descriptionPtr->wheelD = 0.07;
        descriptionPtr->barrelD = 0.07;
        descriptionPtr->robotD = 0.09;
        descriptionPtr->totalD = descriptionPtr->wheelD + 2 * descriptionPtr->barrelD
          + descriptionPtr->robotD;
          descriptionPtr->eps = 0.01;
        descriptionPtr->maxPossibleAngle = 20;
        descriptionPtr->RESOLUTION = 0.02;

        hardObstacleDetectorPtr_.reset(new HardObstacleDetector(descriptionPtr));
        ROS_INFO("Traversability Map Test] :Finished Setting up test instance!");
      }

      void createUniformElevationMap(const TraversabilityMask::MatPtr& elevationMapPtr,
          int width, int height, double elevation)
      {
        elevationMapPtr->create(height, width, CV_64FC1);
        elevationMapPtr->setTo(elevation);
        return;
      }

      void createLinearHorizontalElMap(const TraversabilityMask::MatPtr& elevationMapPtr,
          int width, int height,
          double minElevation, double maxElevation)
      {
        elevationMapPtr->create(height, width, CV_64FC1);
        double slope = tan((maxElevation - minElevation) / width);
        for (int j = 0; j < width; ++j)
        {
          double val = slope * j + minElevation;
          for (int i = 0; i < height; ++i)
          {
            elevationMapPtr->at<double>(i, j) = val;
          }
        }
        return;
      }

      void createLinearVerticalElMap(const TraversabilityMask::MatPtr& elevationMapPtr,
          int width, int height, double minElevation, double maxElevation)
      {
        elevationMapPtr->create(height, width, CV_64FC1);
        double slope = tan((maxElevation - minElevation) / height);

        for (int i = 0; i < height; ++i)
        {
          double val = slope * i + minElevation;

          for (int j = 0; j < width; ++j)
          {
            elevationMapPtr->at<double>(i, j) = val;
          }
        }
        return;
      }

      virtual ~TraversabilityMapTest()
      {}

    protected:
      boost::shared_ptr<HardObstacleDetector> hardObstacleDetectorPtr_;
  };

TEST_F(TraversabilityMapTest, TestUniformElevationMap)
{
  cv::Mat traversabilityMap;
  TraversabilityMask::MatPtr elevationMapPtr(new cv::Mat);
  int width = 300;
  int height = 300;

  // createUniformElevationMap(elevationMapPtr, width, height, 0.5);
  createLinearVerticalElMap(elevationMapPtr, width, height, 0, 10);
  hardObstacleDetectorPtr_->updateElevationMap(elevationMapPtr);
  struct timeval startwtime, endwtime;
  gettimeofday(&startwtime , NULL);

  hardObstacleDetectorPtr_->createTraversabilityMap(*elevationMapPtr, &traversabilityMap);
  gettimeofday(&endwtime, NULL);
  double execTime = static_cast<double>((endwtime.tv_usec -
        startwtime.tv_usec) / 1.0e6 + endwtime.tv_sec -
      startwtime.tv_sec);
  ROS_INFO_STREAM("[Traversability Map Test]: Traversability Map creation time = " << execTime);

  hardObstacleDetectorPtr_->displayTraversabilityMap(traversabilityMap, *elevationMapPtr, 0);
}

} // namespace pandora_vision_obstacle
} // namespace pandora_vision
