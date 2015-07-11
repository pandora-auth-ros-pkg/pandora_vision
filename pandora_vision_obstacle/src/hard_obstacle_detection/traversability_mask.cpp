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
 *   Tsirigotis Christos <tsirif@gmail.com>
 *********************************************************************/

#include <limits>
#include "pandora_vision_obstacle/hard_obstacle_detection/traversability_mask.h"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  TraversabilityMask::
    TraversabilityMask()
    {

    }

  TraversabilityMask::TraversabilityMask(const RobotGeometryMaskDescriptionPtr& descriptionPtr)
  {
    ROS_INFO("[Traversability Mask]: Creating Traversability Mask object!");
    description_ = descriptionPtr;

    // Create the robot height mask.
    createMaskFromDesc(robotGeometryMask_, description_);

    ROS_INFO("[Traversability Mask]: Finished constructing Traversability Mask Object.");
  }

  TraversabilityMask::~TraversabilityMask() {}

  /**
   * @brief Creates the Robot Height Mask
   * @description Creates the height mask for the robot according to the description file.
   * @param inputOutputMap[const MatPtr&] A pointer to the matrix that contains the mask.
   * @param description[RobotGeometryMaskDescriptionPtr] A pointer to the parameter structure
   * that approximately describes the robot.
   * @return void
   */
  void TraversabilityMask::createMaskFromDesc(const MatPtr& inputOutputMask,
      const RobotGeometryMaskDescriptionPtr& description)
  {
    // Initialize the Mask dimensions
    // int totalSize = metersToSteps(description_->totalD);
    // Calculate the values for the size of the robot parts.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    int barrelSize = metersToSteps(description_->barrelD);
    int totalSize = 2 * wheelSize + 2 * barrelSize + robotSize;
    // Initialize the mask of the robot.
    robotGeometryMask_.reset(new cv::Mat(totalSize, totalSize, CV_64FC1));
    robotGeometryMask_->setTo(0.0);

    // Assign the values for the motors of the robots.
    for (int i = 0; i < barrelSize; ++i)
    {
      for (int j = 0; j < barrelSize; ++j)
      {
        // Top Left Barrel Height
        robotGeometryMask_->at<double>(i, j + wheelSize) = description_->barrelH;
        robotGeometryMask_->at<double>(i + wheelSize, j) = description_->barrelH;
        // Top Right Barrel Height
        robotGeometryMask_->at<double>(i + wheelSize, j + totalSize - wheelSize) =
          description_->barrelH;
        robotGeometryMask_->at<double>(i, j + totalSize - wheelSize - barrelSize) =
          description_->barrelH;
        // Bottom Left Barrel Height
        robotGeometryMask_->at<double>(i + totalSize - barrelSize - wheelSize, j) =
          description_->barrelH;
        robotGeometryMask_->at<double>(i + totalSize  - wheelSize, j + wheelSize) =
          description_->barrelH;
        // Bottom Right Barrel Height
        robotGeometryMask_->at<double>(i + totalSize - wheelSize - barrelSize,
            j + totalSize - wheelSize) = description_->barrelH;
        robotGeometryMask_->at<double>(i + totalSize - wheelSize, j + totalSize - wheelSize - barrelSize)
          = description_->barrelH;
      }
    }

    int robotBoxSize = totalSize - 2 * wheelSize - 2 * barrelSize;
    // std::cout << totalSize << std::endl;
    // std::cout << robotBoxSize << std::endl;
    // std::cout << barrelSize << std::endl;
    // std::cout << wheelSize << std::endl;
    // Fill in the values for the height for the body of the robot.
    for (int i = wheelSize; i < totalSize - wheelSize; ++i)
    {
      for (int j = wheelSize ; j < totalSize - wheelSize; ++j)
      {
        // Assign the height of the robot to the corresponding positions.
        robotGeometryMask_->at<double>(i, j) = description_->robotH;
      }
    }
    for (int i = 0; i < wheelSize; ++i)
    {
      for (int j = wheelSize + barrelSize; j < totalSize - wheelSize - barrelSize; ++j)
      {
        robotGeometryMask_->at<double>(i, j) = description_->robotH;
        robotGeometryMask_->at<double>(i + totalSize - wheelSize, j) = description_->robotH;
      }
    }
    for (int i = wheelSize + barrelSize; i < totalSize - wheelSize - barrelSize; ++i)
    {
      for (int j = 0; j < wheelSize; ++j)
      {
        robotGeometryMask_->at<double>(i, j) = description_->robotH;
        robotGeometryMask_->at<double>(i, j + totalSize - wheelSize) = description_->robotH;
      }
    }
    // cv::imshow("mask", *robotGeometryMask_);
    // cv::waitKey(0);
    return;
  }


  /**
   * @brief Check if the provided point is traversible by the robot or not
  */
  int8_t
  TraversabilityMask::findTraversability(const cv::Point& center)
  {
    center_ = center;

    // Calculate the current position of the Upper Left Wheel.
    cv::Point upperLeftWheelPos(
        metersToSteps(center_.x - description_->robotD - 2 * description_->barrelD - description_->wheelD),
        metersToSteps(center_.y - description_->robotD - 2 * description_->barrelD - description_->wheelD));
    // Calculate the current position of the Lower Left Wheel.
    cv::Point lowerLeftWheelPos(
        metersToSteps(center_.x - description_->robotD - description_->barrelD - description_->wheelD),
        metersToSteps(center_.y + description_->robotD + description_->barrelD + description_->wheelD));
    cv::Point upperRightWheelPos(
        metersToSteps(center_.x + description_->robotD + 2 * description_->barrelD - description_->wheelD),
        metersToSteps(center_.y + description_->robotD + 2 * description_->barrelD - description_->wheelD));
    // Calculate the current position of the Lower Right Wheel.
    cv::Point lowerRightWheelPos(
        metersToSteps(center_.x + description_->robotD + description_->barrelD - description_->wheelD),
        metersToSteps(center_.y + description_->robotD + description_->barrelD + description_->wheelD));

    double upperLeftWheelMeanHeight, upperLeftWheelStdDev;
    bool upperLeftWheelValid = findHeightOnWheel(upperLeftWheelPos, &upperLeftWheelMeanHeight,
        &upperLeftWheelStdDev);

    double lowerLeftWheelMeanHeight, lowerLeftWheelStdDev;
    bool lowerLeftWheelValid = findHeightOnWheel(lowerLeftWheelPos, &lowerLeftWheelMeanHeight,
        &lowerLeftWheelStdDev);

    double upperRightWheelMeanHeight, upperRightWheelStdDev;
    bool upperRightWheelValid = findHeightOnWheel(upperRightWheelPos, &upperRightWheelMeanHeight,
        &upperRightWheelStdDev);

    double lowerRightWheelMeanHeight, lowerRightWheelStdDev;
    bool lowerRightWheelValid = findHeightOnWheel(lowerRightWheelPos, &lowerRightWheelMeanHeight,
        &lowerRightWheelStdDev);

    // TODO(Vassilis Choutas): Check return values
    if (!upperLeftWheelValid)
    {
      return -1;
    }
    if (!lowerLeftWheelValid)
    {
      return -1;
    }

    if (!lowerRightWheelValid)
    {
      return -1;
    }
    if (!upperLeftWheelValid)
    {
      return -1;
    }

    int wheelSize = metersToSteps(description_->wheelD);
    double robotSize = metersToSteps(description_->totalD);

    double wheelCenterDist = description_->robotD + 2 * description_->barrelD + description_->wheelD;
    // Get the mask for the left side of the robot
    MatPtr updatedMaskPtr(new cv::Mat(robotGeometryMask_->size(), CV_64FC1));
    // Initialize the transformed map by creating a deep copy of the original.
    robotGeometryMask_->copyTo(*updatedMaskPtr);

    MatPtr tempMapPtr(new cv::Mat(*updatedMaskPtr,
        cv::Rect(0, 0, wheelSize, updatedMaskPtr->rows)));
    // Calculate the mask for the left side of the robot.
    findElevatedLeftRight(tempMapPtr, upperLeftWheelMeanHeight, lowerLeftWheelMeanHeight, wheelCenterDist);

    tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
        cv::Rect(updatedMaskPtr->cols - wheelSize, 0, wheelSize, updatedMaskPtr->rows)));
    // Get the mask for the right side of the robot.
    findElevatedLeftRight(tempMapPtr, upperRightWheelMeanHeight, lowerRightWheelMeanHeight, wheelCenterDist);

    tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
        cv::Rect(0, 0, updatedMaskPtr->cols, wheelSize)));
    // Get the mask for the top side of the robot.
    findElevatedTopBottom(tempMapPtr, upperLeftWheelMeanHeight, upperRightWheelMeanHeight, wheelCenterDist);

    tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
        cv::Rect(updatedMaskPtr->rows - wheelSize, 0, updatedMaskPtr->cols, wheelSize)));
    // Get the mask for the bottom side of the robot.
    findElevatedTopBottom(tempMapPtr, lowerLeftWheelMeanHeight, lowerRightWheelMeanHeight,
        wheelCenterDist);

    interpolateElevationMap(updatedMaskPtr, upperLeftWheelMeanHeight, upperRightWheelMeanHeight,
        lowerRightWheelMeanHeight, lowerLeftWheelMeanHeight);
    // Decide about binary traversability
  }  // End of findTraversability

  double TraversabilityMask::bilinearInterpolation(const cv::Point& P, const cv::Point& Q11, const cv::Point& Q21,
      const cv::Point& Q22, const cv::Point& Q12, double fQ11, double fQ21, double fQ22, double fQ12)
  {
    double R1 = (Q22.x - P.x) / (Q22.x - Q11.x) * fQ11 + (P.x - Q11.x) / (Q22.x - Q11.x) * fQ21;
    double R2 = (Q22.x - P.x) / (Q22.x - Q11.x) * fQ12 + (P.x - Q11.x) / (Q22.x - Q11.x) * fQ22;

    return (Q22.y - P.y) / (Q22.y - Q21.y) * R1 + (P.y - Q11.y) / (Q22.y - Q11.y) * R2;
  }

  /**
   * @brief Performs interpolation on the elevation Map
   * @description Applies bilinear interpolation on the elevation map to fill the
   * unknown values in the middle of the robot's mask.
   * @param inputOutputMap[const MatPtr&] The input mask that will be updated to
   * produce the filled mask.
   * @param upperLeftWheelMeanHeight[double] The mean estimated height of the upper left wheel.
   * @param upperRightWheelMeanHeight[double] The mean estimated height of the upper right wheel.
   * @param lowerRightWheelMeanHeight[double] The mean estimated height of the lower right wheel.
   * @param lowerLeftWheelMeanHeight[double] The mean estimated height of the lower left wheel.
   */
  void TraversabilityMask::interpolateElevationMap(const MatPtr& inputOutputMap, double upperLeftWheelMeanHeight,
      double upperRightWheelMeanHeight, double lowerRightWheelMeanHeight,
      double lowerLeftWheelMeanHeight)
  {
    int wheelSize = metersToSteps(description_->wheelD);
    double robotSize = metersToSteps(description_->totalD);

    // Interpolate the mask values to get the robot's local estimated elevation.
    for (int i = wheelSize + 1; i < robotSize - wheelSize; ++i)
    {
      for (int j = wheelSize + 1; j < robotSize - wheelSize; ++j)
      {
        inputOutputMap->at<double>(i, j) +=
          bilinearInterpolation(cv::Point(j, i),
              cv::Point(0, 0), cv::Point(inputOutputMap->cols - 1, 0),
              cv::Point(inputOutputMap->cols - 1, inputOutputMap->rows - 1), cv::Point(0, inputOutputMap->rows - 1),
              upperLeftWheelMeanHeight, upperRightWheelMeanHeight, lowerRightWheelMeanHeight,
              lowerLeftWheelMeanHeight);
      }
    }
  }

  void
  TraversabilityMask::setElevationMap(const boost::shared_ptr<cv::Mat const>& map)
  {
    elevationMapPtr_ = map;
  }

  void
  TraversabilityMask::loadGeometryMask(const ros::NodeHandle& nh)
  {
  }

  void
  TraversabilityMask::findElevatedLeftRight(MatPtr aLeftRight, double hForward, double hBack, double d)
  {
    // Convert the size of the wheel from distance units to the number of cells
    // it corresponds using the current elevation map resolution.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    double angle;
    double slope;
    // Find the wheel that is located higher
    if (hForward > hBack)
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hForward - hBack) / d);
      slope =  tan(angle);
      for (int j = wheelSize + 1; j < aLeftRight->rows - wheelSize; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
          + hBack;
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(robotSize - j, i) += val;
        }
      }
    }
    else
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hBack - hForward) / d);
      slope =  tan(angle);
      for (int j = wheelSize + 1; j < aLeftRight->rows - wheelSize; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
          + hForward;
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(j, i) += val;
        }
      }
    }

    return;
  }

  void
  TraversabilityMask::findElevatedTopBottom(MatPtr aTopBottom, double hLeft, double hRight, double d)
  {
    // Convert the size of the wheel from distance units to the number of cells
    // it corresponds using the current elevation map resolution.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    double angle;
    double slope;
    // Find the wheel that is located higher
    if (hLeft < hRight)
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hRight - hLeft) / d);
      slope =  tan(angle);
      for (int j = wheelSize + 1; j < aTopBottom->cols - wheelSize; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
            + hLeft;
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, j) += val;
        }
      }
    }
    else
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hLeft - hRight) / d);
      slope =  tan(angle);
      for (int j = wheelSize + 1; j < aTopBottom->cols - wheelSize; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
            + hRight;
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, robotSize - j) += val;
        }
      }
    }

    return;
  }

  bool
  TraversabilityMask::findHeightOnWheel(const cv::Point& wheelPos, double* meanHeight, double* stdDevHeight)
  {
    // MatPtr wheel( new cv::Mat(elevationMapPtr_->rows, elevationMapPtr_->cols, CV_64FC1, cv::Scalar(0)));
    MatPtr wheelElevation;
    bool known = cropToWheel(wheelPos, wheelElevation);
    if (!known)
      return false;

    cv::Mat validityMask = *wheelElevation == - std::numeric_limits<double>::max();
    if (cv::countNonZero(validityMask) > 0)
      return false;

    cv::Scalar mean, std_dev;
    cv::meanStdDev(*elevationMapPtr_, mean, std_dev, *wheelElevation);
    *meanHeight = mean[0];
    *stdDevHeight = std_dev[0];

    wheelElevation->setTo(*meanHeight, *wheelElevation);

    return true;
  }

  /**
   * @brief Creates a mask for the given wheel on the elevation map.
  */
  bool
  TraversabilityMask::cropToWheel(const cv::Point& wheelPos, const MatPtr& wheel)
  {
    int wheelSize = metersToSteps(description_->wheelD);
    // Copy the region of the elevation map that corresponds to the current wheel.
    cv::Mat roi = (*elevationMapPtr_)(cv::Rect(wheelPos.x, wheelPos.x, wheelSize, wheelSize));
    roi.copyTo(*wheel);
    return true;
  }

  TraversabilityMask::MatPtr TraversabilityMask::cropToRight()
  {
  }

  TraversabilityMask::MatPtr
  TraversabilityMask::cropToLeft()
  {
  }

  TraversabilityMask::MatPtr
  TraversabilityMask::cropToTop()
  {
  }

  TraversabilityMask::MatPtr
  TraversabilityMask::cropToBottom()
  {
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
