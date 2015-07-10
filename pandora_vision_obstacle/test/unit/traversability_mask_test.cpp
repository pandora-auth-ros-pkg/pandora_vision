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
 *********************************************************************/

#include <vector>
#include <gtest/gtest.h>
#include "pandora_vision_obstacle/hard_obstacle_detection/RobotGeometryMaskDescription.h"
#include "pandora_vision_obstacle/hard_obstacle_detection/traversability_mask.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  class TraversabilityMaskTest: public ::testing::Test
  {
  public:
    TraversabilityMaskTest()    {}

    typedef TraversabilityMask::MatPtr MatPtr;

    virtual void SetUp()
    {
      // Set the robot dimensions.
      boost::shared_ptr<RobotGeometryMaskDescription> descriptionPtr(new RobotGeometryMaskDescription);
      setRobotDescription(descriptionPtr);

      traversabilityMaskPtr_.reset(new TraversabilityMask(descriptionPtr));

      int wheelSize = traversabilityMaskPtr_->metersToSteps(descriptionPtr->wheelD);
      int robotSize = traversabilityMaskPtr_->metersToSteps(descriptionPtr->robotD);
      int barrelSize = traversabilityMaskPtr_->metersToSteps(descriptionPtr->barrelD);
      int totalSize = traversabilityMaskPtr_->metersToSteps(descriptionPtr->totalD);

      updatedElevationMaskPtr_.reset(new cv::Mat(traversabilityMaskPtr_->robotGeometryMask_->size(), CV_64FC1));
    }

    void setRobotDescription(const boost::shared_ptr<RobotGeometryMaskDescription>& descriptionPtr)
    {
      descriptionPtr->wheelH = 0.0;
      descriptionPtr->barrelH = 0.067;
      descriptionPtr->robotH = 0.134;
      descriptionPtr->wheelD = 0.0742;
      descriptionPtr->barrelD = 0.075;
      descriptionPtr->robotD = 0.296;
      descriptionPtr->totalD = 2 * descriptionPtr->wheelD + 2 * descriptionPtr->barrelD
        + descriptionPtr->robotD;
      descriptionPtr->RESOLUTION = 0.01;
    }

    void createUniformElevationMap(const MatPtr& elevationMapPtr, int width, int height, double elevation)
    {
      elevationMapPtr->create(height, width, CV_64FC1);
      elevationMapPtr->setTo(elevation);
      return;
    }

    void createLinearHorizontalElMap(const MatPtr& elevationMapPtr, int width, int height,
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

    void createLinearVerticalElMap(const MatPtr& elevationMapPtr, int width, int height,
        double minElevation, double maxElevation)
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

    void findElevatedLeft(MatPtr al, double hForward, double hBack, double d)
    {
      traversabilityMaskPtr_->findElevatedLeft(al, hForward, hBack, d);
    }

    inline double getMaskValue(int i, int j)
    {
      return traversabilityMaskPtr_->robotGeometryMask_->at<double>(i,j);
    }


    virtual ~TraversabilityMaskTest ()
    {}

  protected:
    boost::shared_ptr<TraversabilityMask> traversabilityMaskPtr_;
    MatPtr updatedElevationMaskPtr_;
  };


  TEST_F(TraversabilityMaskTest, SideElevationTransform)
  {
    MatPtr elevationMapPtr;
    int width = 300;
    int height = 300;

    createUniformElevationMap(elevationMapPtr, width, height, 0);

    for (int i = 0; i < elevationMapPtr->rows; ++i)
    {
      for (int j = 0; j < elevationMapPtr->cols; ++j)
      {
        ASSERT_NEAR(elevationMapPtr->at<double>(i, j), getMaskValue(i, j),
            0.01);
      }
    }
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision

