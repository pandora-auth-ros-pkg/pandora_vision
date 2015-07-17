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
 *   Vassilis Choutas <vasilis4ch@gmail.com>
 *   Kofinas Miltiadis <mkofinas@gmail.com>
 *   *********************************************************************/

#include <limits>
#include "pandora_vision_obstacle/hard_obstacle_detection/traversability_mask.h"
#include "opencv2/imgproc/imgproc.hpp"

namespace pandora_vision
{
namespace pandora_vision_obstacle
{
  TraversabilityMask::
    TraversabilityMask()
    {
      displayWheelPositionFlag_ = false;
    }

  TraversabilityMask::TraversabilityMask(const RobotGeometryMaskDescriptionPtr& descriptionPtr)
  {
    ROS_INFO("[Traversability Mask]: Creating Traversability Mask object!");
    description_ = descriptionPtr;
    displayWheelPositionFlag_ = false;

    // Create the robot height mask.
    createMaskFromDesc(description_);


    ROS_INFO("[Traversability Mask]: Finished constructing Traversability Mask Object.");
  }

  TraversabilityMask::~TraversabilityMask() {}

  void TraversabilityMask::createMaskFromDesc()
  {
    // Calculate the values for the size of the robot parts.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    int barrelSize = metersToSteps(description_->barrelD);
    int totalSize = robotSize + 2 * (barrelSize + wheelSize);

    // Initialize the mask of the robot.
    robotGeometryMask_.reset(new cv::Mat(totalSize, totalSize, CV_64FC1, cv::Scalar(0)));

    // Assign the values for the motors of the robots.
    // Top Left Barrels
    (*robotGeometryMask_)(cv::Rect(0, wheelSize, wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize, 0, wheelSize, wheelSize)) = description_->barrelH;

    // Top Right Barrels
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize + robotSize, 0,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize, wheelSize,
          wheelSize, wheelSize)) = description_->barrelH;
    // Bottom Left Barrels.
    (*robotGeometryMask_)(cv::Rect(0, wheelSize + barrelSize + robotSize,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize,
          wheelSize + 2 * barrelSize + robotSize, wheelSize, wheelSize)) = description_->barrelH;
    // Bottom Right Barrels.
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize + robotSize, wheelSize + 2 * barrelSize + robotSize,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize,
          wheelSize + barrelSize + robotSize, wheelSize, wheelSize)) = description_->barrelH;

    // Robot sidelines.
    (*robotGeometryMask_)(cv::Rect(0, wheelSize + barrelSize, wheelSize, robotSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize, wheelSize + barrelSize,
          wheelSize, robotSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize, 0, robotSize, wheelSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize, wheelSize + 2 * barrelSize + robotSize,
          robotSize, wheelSize)) = description_->robotH;

    // Robot Main body.
    (*robotGeometryMask_)(cv::Rect(wheelSize, wheelSize, 2 * barrelSize + robotSize,
          2 * barrelSize + robotSize)) = description_->robotH;
    return;
  }

  /**
   * @brief Creates the Robot Height Mask
   * @description Creates the height mask for the robot according to the description file.
   * @param inputOutputMap[const MatPtr&] A pointer to the matrix that contains the mask.
   * @param description[RobotGeometryMaskDescriptionPtr] A pointer to the parameter structure
   * that approximately describes the robot.
   * @return void
   */
  void TraversabilityMask::createMaskFromDesc(const RobotGeometryMaskDescriptionPtr& description)
  {
    // Calculate the values for the size of the robot parts.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->robotD);
    int barrelSize = metersToSteps(description_->barrelD);
    int totalSize = robotSize + 2 * (barrelSize + wheelSize);

    // Initialize the mask of the robot.
    robotGeometryMask_.reset(new cv::Mat(totalSize, totalSize, CV_64FC1, cv::Scalar(0)));

    // Assign the values for the motors of the robots.
    // Top Left Barrels
    (*robotGeometryMask_)(cv::Rect(0, wheelSize, wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize, 0, wheelSize, wheelSize)) = description_->barrelH;

    // Top Right Barrels
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize + robotSize, 0,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize, wheelSize,
          wheelSize, wheelSize)) = description_->barrelH;
    // Bottom Left Barrels.
    (*robotGeometryMask_)(cv::Rect(0, wheelSize + barrelSize + robotSize,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize,
          wheelSize + 2 * barrelSize + robotSize, wheelSize, wheelSize)) = description_->barrelH;
    // Bottom Right Barrels.
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize + robotSize, wheelSize + 2 * barrelSize + robotSize,
          wheelSize, wheelSize)) = description_->barrelH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize,
          wheelSize + barrelSize + robotSize, wheelSize, wheelSize)) = description_->barrelH;

    // Robot sidelines.
    (*robotGeometryMask_)(cv::Rect(0, wheelSize + barrelSize, wheelSize, robotSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + 2 * barrelSize + robotSize, wheelSize + barrelSize,
          wheelSize, robotSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize, 0, robotSize, wheelSize)) = description_->robotH;
    (*robotGeometryMask_)(cv::Rect(wheelSize + barrelSize, wheelSize + 2 * barrelSize + robotSize,
          robotSize, wheelSize)) = description_->robotH;

    // Robot Main body.
    (*robotGeometryMask_)(cv::Rect(wheelSize, wheelSize, 2 * barrelSize + robotSize,
          2 * barrelSize + robotSize)) = description_->robotH;
  }

  /**
   * @brief Check if the provided point is traversible by the robot or not
  */
  int8_t
  TraversabilityMask::findTraversability(const cv::Point& center)
  {
    int8_t returnValue;
    center_ = center;
    // if (elevationMapPtr_->at<double>(center_.y, center_.x) == - std::numeric_limits<double>::max())
      // return unknownArea;

    int maskSize = robotGeometryMask_->rows;
    int wheelSize = metersToSteps(description_->wheelD);
    int barrelSize = metersToSteps(description_->barrelD);
    int robotSize = metersToSteps(description_->totalD);
    int robotDSize = metersToSteps(description_->robotD);

    // Calculate the current position of the Upper Left Wheel.

    cv::Point2d upperLeftWheelPos(
        center_.x - maskSize / 2,
        center_.y - maskSize / 2);
    // Calculate the current position of the Lower Left Wheel.
    cv::Point2d lowerLeftWheelPos(
        center_.x - maskSize / 2,
        center_.y + maskSize / 2 - wheelSize);
    cv::Point2d upperRightWheelPos(
        center_.x + maskSize / 2  - wheelSize,
        center_.y - maskSize / 2);
    // Calculate the current position of the Lower Right Wheel.
    cv::Point2d lowerRightWheelPos(
        center_.x + maskSize / 2  - wheelSize,
        center_.y + maskSize / 2  - wheelSize);

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

    // Calculate the number of valid wheels.
    int validWheelNum = upperLeftWheelValid + upperRightWheelValid + lowerLeftWheelValid
      + lowerRightWheelValid;

    double wheelCenterDist = description_->robotD + 2 * description_->barrelD + description_->wheelD;
    // Get the mask for the left side of the robot
    MatPtr updatedMaskPtr(new cv::Mat(robotGeometryMask_->size(), CV_64FC1));
    // Initialize the transformed map by creating a deep copy of the original.
    robotGeometryMask_->copyTo(*updatedMaskPtr);

    int validAreaTopLeftX = 0, validAreaTopLeftY = 0;
    int validAreaWidth = 0, validAreaHeight = 0;

    cv::Mat tempImg, displayImage;
<<<<<<< Updated upstream
<<<<<<< Updated upstream

    // Create a mask for the wheels.
    cv::Mat wheelMask(updatedMaskPtr->size(), CV_8UC1, cv::Scalar(1));
    // Update the value of the mask on the wheels.
    // Set the height of the upper left wheel.
    wheelMask(cv::Rect(0, 0, wheelSize, wheelSize)) = cv::Scalar(0);
    // Set the height of the lower left wheel.
    wheelMask(cv::Rect(0, updatedMaskPtr->rows - wheelSize, wheelSize, wheelSize)) = cv::Scalar(0);
    // Set the height of the upper right wheel.
    wheelMask(cv::Rect(updatedMaskPtr->cols - wheelSize, 0, wheelSize, wheelSize)) = cv::Scalar(0);
    // Set the height of the lower right wheel.
    wheelMask(cv::Rect(updatedMaskPtr->cols - wheelSize, updatedMaskPtr->rows - wheelSize,
          wheelSize, wheelSize)) = cv::Scalar(0);

    bool calcElevationFlag;

    if (displayWheelPositionFlag_)
    {
      elevationMapPtr_->convertTo(tempImg, CV_8UC1, 255.0);
      cv::normalize(tempImg, tempImg, 0, 255, cv::NORM_MINMAX);
      cv::cvtColor(tempImg, displayImage, CV_GRAY2BGR);
      // Paint the area that the top left wheel occupies red.
      cv::rectangle(displayImage, cv::Rect(center_.x - robotDSize / 2 - 2 * wheelSize,
            center_.y - robotDSize / 2 - 2 * wheelSize, wheelSize, wheelSize), cv::Scalar(255, 0, 0), -1);
      // Paint the area that the top right wheel occupies blue.
      cv::rectangle(displayImage, cv::Rect(center_.x + robotDSize / 2 + wheelSize,
            center_.y - robotDSize / 2 - 2 * wheelSize, wheelSize, wheelSize), cv::Scalar(0, 0, 255), -1);
      // Paint the area that the bottom left wheel occupies green.
      cv::rectangle(displayImage, cv::Rect(center_.x - robotDSize / 2 - 2 * wheelSize,
            center_.y + robotDSize / 2 + wheelSize, wheelSize, wheelSize), cv::Scalar(0, 255, 0), -1);
      // Paint the area that the bottom right wheel occupies magenta.
      cv::rectangle(displayImage, cv::Rect(center_.x + robotDSize / 2 + wheelSize,
            center_.y + robotDSize / 2 + wheelSize, wheelSize, wheelSize), cv::Scalar(255, 0, 255), -1);
    }
    // TODO(Vassilis Choutas): Check return values
    // If no wheel position is know then mark the current point as unknown.
    if (validWheelNum == 0 || validWheelNum == 1)
    {
        returnValue = unknownArea;
    }
    else if (validWheelNum == 2 || validWheelNum == 3)
    {
      MatPtr tempMapPtr;
      cv::Mat validElevationMapOverlap;

      // If the diagonal wheels are valid the ignore this cell.
      // if ((upperLeftWheelValid && lowerRightWheelValid) || (upperRightWheelValid && lowerLeftWheelValid))
        // return unknownArea;
      // The top Part of the robot Wheelpart is valid
      // else if (upperLeftWheelValid && upperRightWheelValid)
      if (upperLeftWheelValid && upperRightWheelValid)
      {
        validAreaTopLeftX = wheelSize;
        validAreaTopLeftY = 0;
        validAreaWidth = updatedMaskPtr->cols - 2 * wheelSize;
        validAreaHeight = wheelSize;
        tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
              cv::Rect(wheelSize, 0, validAreaWidth, wheelSize)));
        // Get the mask for the top side of the robot.
        calcElevationFlag = findElevatedTopBottom(tempMapPtr, upperLeftWheelMeanHeight, upperRightWheelMeanHeight,
            wheelCenterDist);
        validElevationMapOverlap = (*elevationMapPtr_)(cv::Rect(center_.x - robotDSize / 2 - wheelSize,
              center_.y - robotDSize / 2 - 2 * wheelSize , validAreaWidth, validAreaHeight));
        // Paint yellow the area between the top wheels if they are valid.
        if (displayWheelPositionFlag_)
          cv::rectangle(displayImage, cv::Rect(center_.x - robotDSize / 2 - wheelSize,
                center_.y - robotDSize / 2 - 2 * wheelSize , validAreaWidth, validAreaHeight),
              cv::Scalar(0, 255, 255), -1);
      }
      // The bottom part of the robot is valid.
      else if (lowerLeftWheelValid && lowerRightWheelValid)
      {
        validAreaTopLeftX = wheelSize;
        validAreaTopLeftY = updatedMaskPtr->rows - wheelSize;
        validAreaWidth = updatedMaskPtr->cols - 2 * wheelSize;
        validAreaHeight = wheelSize;
        tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
              cv::Rect(wheelSize, validAreaTopLeftY, validAreaWidth, wheelSize)));
        // Get the mask for the bottom side of the robot.
        calcElevationFlag = findElevatedTopBottom(tempMapPtr, lowerLeftWheelMeanHeight, lowerRightWheelMeanHeight,
            wheelCenterDist);
        validElevationMapOverlap = (*elevationMapPtr_)(cv::Rect(center_.x - robotDSize / 2 - wheelSize,
              center_.y + robotDSize / 2 + wheelSize, validAreaWidth, validAreaHeight));

        // Paint yellow the area between the bottom wheels if they are valid.
        if (displayWheelPositionFlag_)
          cv::rectangle(displayImage, cv::Rect(center_.x - robotDSize / 2 - wheelSize,
                center_.y + robotDSize / 2 + wheelSize, validAreaWidth, validAreaHeight),
              cv::Scalar(0, 255, 255), -1);
      }
      // The left part of the robot is valid.
      else if (upperLeftWheelValid && lowerLeftWheelValid)
      {
        validAreaTopLeftX = 0;
        validAreaTopLeftY = wheelSize;
        validAreaWidth = wheelSize;
        validAreaHeight = updatedMaskPtr->rows - 2 * wheelSize;
        tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
              cv::Rect(0, wheelSize, wheelSize, validAreaHeight)));
        // Calculate the mask for the left side of the robot.
        calcElevationFlag = findElevatedLeftRight(tempMapPtr, upperLeftWheelMeanHeight, lowerLeftWheelMeanHeight,
            wheelCenterDist);
        validElevationMapOverlap = (*elevationMapPtr_)(cv::Rect(center_.x - robotDSize / 2 - 2 * wheelSize,
              center_.y - robotDSize / 2 - wheelSize, validAreaWidth, validAreaHeight));
        // Paint yellow the area between the left wheels if they are valid.
        if (displayWheelPositionFlag_)
          cv::rectangle(displayImage, cv::Rect(center_.x - robotDSize / 2 - 2 * wheelSize,
                center_.y - robotDSize / 2 - wheelSize, validAreaWidth, validAreaHeight),
              cv::Scalar(0, 255, 255), -1);
      }
      // The right part of the robot is valid.
      else if (upperRightWheelValid && lowerRightWheelValid)
      {
        validAreaTopLeftX = updatedMaskPtr->cols - wheelSize;
        validAreaTopLeftY = 0;
        validAreaWidth = wheelSize;
        validAreaHeight = updatedMaskPtr->rows - 2 * wheelSize;
        tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
              cv::Rect(validAreaTopLeftX, wheelSize, wheelSize, validAreaHeight)));
        // Get the mask for the right side of the robot.
        calcElevationFlag = findElevatedLeftRight(tempMapPtr, upperRightWheelMeanHeight, lowerRightWheelMeanHeight,
            wheelCenterDist);
        validElevationMapOverlap = (*elevationMapPtr_)(cv::Rect(center_.x + robotDSize / 2 + wheelSize,
              center_.y - robotDSize / 2 - wheelSize, validAreaWidth, validAreaHeight));
        // Paint yellow the area between the right wheels if they are valid.
        if (displayWheelPositionFlag_)
          cv::rectangle(displayImage, cv::Rect(center_.x + robotDSize / 2 + wheelSize,
                center_.y - robotDSize / 2 - wheelSize, validAreaWidth, validAreaHeight),
              cv::Scalar(0, 255, 255), -1);
      }

      // if (upperLeftWheelValid && upperRightWheelValid)
      // {
        // MatPtr tempMapPtr(new cv::Mat(*updatedMaskPtr,
              // cv::Rect(0, 0, updatedMaskPtr->cols - 1, wheelSize)));
        // // Get the mask for the top side of the robot.
        // bool traversabilityFlag = findElevatedTopBottom(tempMapPtr, upperLeftWheelMeanHeight,
            // upperRightWheelMeanHeight, wheelCenterDist);
        // // If the return value is false then it that point is not traversible.
        // if (!traversabilityFlag)
          // return occupiedArea;
        // validAreaWidth += updatedMaskPtr->cols - 1;
        // validAreaHeight += wheelSize;
        // // Get the Position of the the barrels that are located below the forward wheels.
        // cv::Point2d leftBarrelPos(
            // metersToSteps(center_.x - description_->robotD / 2 - description_->barrelD
              // - description_->wheelD),
            // metersToSteps(center_.y - description_->robotD / 2 - description_->barrelD));
        // cv::Point2d rightBarrelPos(
            // metersToSteps(center_.x + description_->robotD / 2 + description_->barrelD),
            // metersToSteps(center_.y - description_->robotD / 2 - description_->barrelD));

        // double upperLeftBarrelMeanHeight, upperLeftBarrelStdDev;
        // bool upperLeftBarrelValid = findHeightOnWheel(leftBarrelPos, &upperLeftBarrelMeanHeight,
            // &upperLeftBarrelStdDev);

        // double upperRightBarrelMeanHeight, upperRightBarrelStdDev;
        // bool upperRightBarrelValid = findHeightOnWheel(rightBarrelPos, &upperRightBarrelMeanHeight,
            // &upperRightBarrelStdDev);
        // if (upperLeftBarrelValid && upperRightBarrelValid)
        // {
          // // Update the values for the elevated mask for the barrel area.
          // tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
                // cv::Rect(0, wheelSize, updatedMaskPtr->cols - 1, barrelSize)));
          // traversabilityFlag = findElevatedTopBottomBody(tempMapPtr, upperLeftBarrelMeanHeight,
              // upperRightBarrelMeanHeight, wheelCenterDist);
          // if (!traversabilityFlag)
            // return occupiedArea;
          // validAreaHeight += barrelSize;
          // // Get the Position of the upper half of the robot body located below .
          // cv::Point2d leftBodyPartPos(
              // metersToSteps(center_.x - description_->robotD / 2 - description_->barrelD
                // - description_->wheelD),
              // metersToSteps(center_.y - description_->robotD / 2));
          // cv::Point2d rightBodyPartPos(
              // metersToSteps(center_.x + description_->robotD / 2 + description_->barrelD),
              // metersToSteps(center_.y - description_->robotD));

          // double upperLeftBodyPartMeanHeight, upperLeftBodyPartStdDev;
          // bool upperLeftBodyPartValid = findHeightOnWheel(leftBodyPartPos, &upperLeftBodyPartMeanHeight,
              // &upperLeftBodyPartStdDev);

          // double upperRightBodyPartMeanHeight, upperRightBodyPartStdDev;
          // bool upperRightBodyPartValid = findHeightOnWheel(rightBodyPartPos, &upperRightBodyPartMeanHeight,
              // &upperRightBodyPartStdDev);
          // // Check that the upper half parts of the robot are within a known area of the elevation map
          // if (upperLeftBodyPartValid && upperRightBodyPartValid)
          // {
            // tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
                  // cv::Rect(0, wheelSize + barrelSize, updatedMaskPtr->cols - 1, robotDSize / 2)));
            // traversabilityFlag = findElevatedTopBottomBody(tempMapPtr, upperRightWheelMeanHeight,
                // lowerRightWheelMeanHeight, wheelCenterDist);
            // // If the upper half of the main robot body is not located in a valid area
            // // then mark the point as occupied.
            // if (!traversabilityFlag)
              // return occupiedArea;
            // validAreaHeight += robotDSize / 2;
          // }
        // }
        // // Extract the elevation map area that corresponds to the valid part of the mask.
        // cv::Mat validElevationMapOverlap =
          // (*elevationMapPtr_)(cv::Rect(center_.x - robotDSize / 2 - barrelSize - wheelSize,
                // center_.y -  - robotDSize / 2 - barrelSize - wheelSize, validAreaWidth, validAreaHeight));
        // // Create a shallow copy of the valid region of the transformed robot height mask.
        // cv::Mat validMask = (*updatedMaskPtr)(cv::Rect(validAreaTopLeftX, validAreaTopLeftY,
              // validAreaWidth, validAreaHeight));
        // cv::Mat diff = validMask - validElevationMapOverlap;
        // cv::Mat result = diff <= 0 & validElevationMapOverlap != unknownArea;
        // if (cv::countNonZero(result) > 0)
          // return occupiedArea;
        // else
        // {
          // if (cv::countNonZero(validElevationMapOverlap == - std::numeric_limits<double>::max()) == 0)
            // return freeArea;
          // else
            // return unknownArea;
        // }
      // }  // End_if : Only top wheels on known elevation.
      // if (lowerLeftWheelValid && lowerRightWheelValid)
      // {
      // }
      // if (upperLeftWheelValid && lowerLeftWheelValid)
      // {
      // }
      // if (upperRightWheelValid && lowerRightWheelValid)
      // {
      // }
      // Create a shallow copy of the valid region of the transformed robot height mask.
      cv::Mat validMask = (*updatedMaskPtr)(cv::Rect(validAreaTopLeftX, validAreaTopLeftY,
            validAreaWidth, validAreaHeight));

      if (validMask.empty())
        return unknownArea;
      cv::Mat diff = validMask - validElevationMapOverlap;
      if (diff.empty())
        returnValue = unknownArea;
      cv::Mat result = (diff <= 0) & (validElevationMapOverlap != unknownArea) &
        wheelMask(cv::Rect(validAreaTopLeftX, validAreaTopLeftY, validAreaWidth, validAreaHeight));
      if (cv::countNonZero(result) > 0)
        returnValue = occupiedArea;
      else
        returnValue = freeArea;
    }
    // else if (validWheelNum == 3)
    // {
    /* } */
    // If all the wheels are on a known area of the elevation Map.
    else if (validWheelNum == 4)
    {
      MatPtr tempMapPtr(new cv::Mat(*updatedMaskPtr,
            cv::Rect(0, 0, wheelSize, updatedMaskPtr->rows)));
      // Calculate the mask for the left side of the robot.
      bool calcElevationFlag = findElevatedLeftRight(tempMapPtr, upperLeftWheelMeanHeight, lowerLeftWheelMeanHeight,
          wheelCenterDist);
      tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
            cv::Rect(updatedMaskPtr->cols - wheelSize, 0, wheelSize, updatedMaskPtr->rows)));
      // Get the mask for the right side of the robot.
      calcElevationFlag = findElevatedLeftRight(tempMapPtr, upperRightWheelMeanHeight, lowerRightWheelMeanHeight,
          wheelCenterDist);

      tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
            cv::Rect(0, 0, updatedMaskPtr->cols, wheelSize)));
      // Get the mask for the top side of the robot.
      calcElevationFlag = findElevatedTopBottom(tempMapPtr, upperLeftWheelMeanHeight,
          upperRightWheelMeanHeight, wheelCenterDist);

      tempMapPtr.reset(new cv::Mat(*updatedMaskPtr,
            cv::Rect(0, updatedMaskPtr->rows - wheelSize, updatedMaskPtr->cols, wheelSize)));
      // Get the mask for the bottom side of the robot.
      calcElevationFlag = findElevatedTopBottom(tempMapPtr, lowerLeftWheelMeanHeight, lowerRightWheelMeanHeight,
          wheelCenterDist);

      // Paint the area between all the wheels.
      if (displayWheelPositionFlag_)
      {
        // Paint the area between the top pair of wheels.
        cv::rectangle(displayImage, cv::Rect(center_.x - robotDSize / 2 - wheelSize,
                center_.y - robotDSize / 2 - 2 * wheelSize , updatedMaskPtr->cols - 2 * wheelSize, wheelSize),
              cv::Scalar(0, 255, 255), -1);
        // Paint the area between the bottom pair of wheels.
        cv::rectangle(displayImage, cv::Rect(center_.x - robotDSize / 2 - wheelSize,
              center_.y + robotDSize / 2 + wheelSize, updatedMaskPtr->cols - 2 * wheelSize, wheelSize),
            cv::Scalar(0, 255, 255), -1);
        cv::rectangle(displayImage, cv::Rect(center_.x - robotDSize / 2 - 2 * wheelSize,
              center_.y - robotDSize / 2 - wheelSize, wheelSize, updatedMaskPtr->rows - 2 * wheelSize),
            cv::Scalar(0, 255, 255), -1);
        cv::rectangle(displayImage, cv::Rect(center_.x + robotDSize / 2 + wheelSize,
              center_.y - robotDSize / 2 - wheelSize, wheelSize, updatedMaskPtr->rows - 2 * wheelSize),
            cv::Scalar(0, 255, 255), -1);
      }

      interpolateElevationMap(updatedMaskPtr, upperLeftWheelMeanHeight, upperRightWheelMeanHeight,
          lowerRightWheelMeanHeight, lowerLeftWheelMeanHeight);
      validAreaTopLeftX = 0;
      validAreaTopLeftY = 0;
      validAreaWidth = updatedMaskPtr->cols;
      validAreaHeight = updatedMaskPtr->rows;
      // Decide about binary traversability
      // Extract the elevation map area that corresponds to the valid part of the mask.
      cv::Mat validElevationMapOverlap =
        (*elevationMapPtr_)(cv::Rect(center_.x - robotDSize / 2 - 2 * wheelSize,
              center_.y - robotDSize / 2 - 2 * wheelSize, validAreaWidth, validAreaHeight));
      // Create a shallow copy of the valid region of the transformed robot height mask.
      cv::Mat validMask = (*updatedMaskPtr)(cv::Rect(validAreaTopLeftX, validAreaTopLeftY,
            validAreaWidth, validAreaHeight));
      cv::Mat diff = validMask - validElevationMapOverlap;

      cv::Mat result = (diff <= 0) & (validElevationMapOverlap != unknownArea) &
        wheelMask;
      if (cv::countNonZero(result) > 0)
        returnValue = occupiedArea;
      else
        returnValue = freeArea;
    }
    // Display the position of the robot on the map.
    if (displayWheelPositionFlag_)
    {
      cv::imshow("Display Image", displayImage);
      cv::waitKey(1);
    }

    if (validWheelNum >= 2)
      cv::waitKey(0);

    // Display the position of the robot on the map.
    if (displayWheelPositionFlag_)
    {
      cv::imshow("Display Image", displayImage);
      cv::waitKey(1);
    }
    return returnValue;
  }  // End of findTraversability

  /**
   * @brief Implements the bilinear interpolation method
   * @description This method takes as input a set of four known and one unknown point and
   * applies bilinear interpolation to find the function value on the unknown Point2d.
  */
  double TraversabilityMask::bilinearInterpolation(const cv::Point2d& P, const cv::Point2d& Q11, const cv::Point2d& Q21,
      const cv::Point2d& Q22, const cv::Point2d& Q12, double fQ11, double fQ21, double fQ22, double fQ12)
  {
    double R1 = (Q22.x - P.x) / (Q22.x - Q11.x) * fQ11 + (P.x - Q11.x) / (Q22.x - Q11.x) * fQ21;
    double R2 = (Q22.x - P.x) / (Q22.x - Q11.x) * fQ12 + (P.x - Q11.x) / (Q22.x - Q11.x) * fQ22;

    return (Q22.y - P.y) / (Q22.y - Q21.y) * R1 + (P.y - Q11.y) / (Q22.y - Q11.y) * R2;
  }

  /**
   * @brief Applies barycentric Interpolation to calculate unknown values in a triangle.
   * @description Given the three vertices of the triangles and their corresponding values
   * we calculate the barycentric coordinates of the query point and using them the approximate value
   * of the function at this point
   * @param P[const cv::Point2d&] The query point where the interpolation will be performed.
   * @param A[const cv::Point2d&] A vertice of the triangle
   * @param B[const cv::Point2d&] The second vertice of the triangle.
   * @param C[const cv::Point2d&] The third vertice of the triangle.
   * @param fA[double] The value of the function at the first vertice.
   * @param fB[double] The value of the function at the second vertice.
   * @param fC[double] The value of the function at the final vertice.
   * @return double The interpolated value at the query point.
   */
  double TraversabilityMask::barycentricInterpolation(const cv::Point2d& P, const cv::Point2d& A,
      const cv::Point2d& B, const cv::Point2d&C, double fA, double fB, double fC)
  {
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
  void TraversabilityMask::interpolateElevationMap(const MatPtr& inputOutputMap, double upperLeftCornerMeanHeight,
      double upperRightCornerMeanHeight, double lowerRightCornerMeanHeight,
      double lowerLeftCornerMeanHeight)
  {
    int wheelSize = metersToSteps(description_->wheelD);

    // Interpolate the mask values to get the robot's local estimated elevation.
    for (int i = wheelSize; i < inputOutputMap->rows - wheelSize; ++i)
    {
      for (int j = wheelSize; j < inputOutputMap->cols - wheelSize; ++j)
      {
        inputOutputMap->at<double>(i, j) +=
          bilinearInterpolation(cv::Point2d(j, i),
              cv::Point2d(0, 0), cv::Point2d(inputOutputMap->cols - 1, 0),
              cv::Point2d(inputOutputMap->cols - 1, inputOutputMap->rows - 1), cv::Point2d(0, inputOutputMap->rows - 1),
              upperLeftCornerMeanHeight, upperRightCornerMeanHeight, lowerRightCornerMeanHeight,
              lowerLeftCornerMeanHeight);
      }
    }
  }

  void
  TraversabilityMask::setElevationMap(const boost::shared_ptr<cv::Mat const>& map)
  {
    elevationMapPtr_ = map;
   }

  /**
   * @brief Loads a robot description and create a 2D mask to describe it.
   * @description The robot's dimensions are loaded using the provided nodehandle and
   * then an approximate 2D elevation mask is created.
   * @param nh[const ros::NodeHandle& nh] A nodehandle used to access the paramater server
   * @return void
  */
  void
  TraversabilityMask::loadGeometryMask(const ros::NodeHandle& nh)
  {
    description_.reset(new RobotGeometryMaskDescription);

    if (!nh.getParam("robot_description/wheel_height", description_->wheelH))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the height of the wheels!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->wheelH = 0;
    }
    if (!nh.getParam("robot_description/barrel_height", description_->barrelH))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the height of the motor cylinder!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->barrelH = 0.06;
    }
    if (!nh.getParam("robot_description/robot_height", description_->robotH))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the height of the body of the robot!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->robotH = 0.08;
    }
    if (!nh.getParam("robot_description/wheel_width", description_->wheelD))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the width of the robot's wheel!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->wheelD = 0.07;
    }
    if (!nh.getParam("robot_description/barrel_width", description_->barrelD))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the width of the robot's motor cylinder!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->barrelD = 0.07;
    }
    if (!nh.getParam("robot_description/robot_width", description_->robotD))
    {
      ROS_WARN("[Traversability Mask]: Could not retrieve the width of the robot's body!");
      ROS_INFO("[Traversability Mask]: Using the default value!");
      description_->robotD = 0.09;
    }

    description_->totalD = description_->robotD + 2 * (description_->barrelD + description_->wheelD);

    if (!nh.getParam("robot_description/epsilon", description_->eps))
    {
      ROS_WARN("[Traversability Mask]: Could not read the tolerance parameter!");
      ROS_INFO("[Traversability Mask]: Setting it to 1 cm!");
      description_->eps = 0.01;
    }
    if (!nh.getParam("robot_description/max_slope", description_->eps))
    {
      ROS_WARN("[Traversability Mask]: Could not read the maximum possible slope!");
      ROS_INFO("[Traversability Mask]: Setting it to 20 degrees!");
      description_->maxPossibleAngle = 20;
    }
    if (!nh.getParam("robot_description/cellResolution", description_->RESOLUTION))
    {
      ROS_WARN("[Traversability Mask]: Could not read the mask resolution!");
      ROS_INFO("[Traversability Mask]: Setting it to 2 cm per cell!");
      description_->RESOLUTION = 0.02;
    }

    ROS_INFO("[Traversability Mask]: Mask Loading finished successfully!");
    return;
  }

  bool
  TraversabilityMask::findElevatedLeftRight(MatPtr aLeftRight, double hForward, double hBack, double d)
  {
    // Convert the size of the wheel from distance units to the number of cells
    // it corresponds using the current elevation map resolution.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->totalD);

    // Iterate over the section of the mask for the current pair of wheels
    // and update their corresponding values.
    double slope = fabs(hForward - hBack) / d;
    double angle = asin(slope);
    // Reject slopes greater than the maximum possible angle.
    if (angle * 180 / CV_PI > description_->maxPossibleAngle)
      return false;
    // Find the wheel that is located higher
    if (hForward > hBack)
    {
      double slopeResolution = hBack;
      for (int j = wheelSize; j < aLeftRight->rows - wheelSize; ++j)
      {
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(aLeftRight->rows - j - 1, i) += j * fabs(hForward - hBack) / aLeftRight->rows
            + hBack;
        }
      }
    }
    else
    {
      double slopeResolution = hForward;
      for (int j = wheelSize; j < aLeftRight->rows - wheelSize; ++j)
      {
        slopeResolution += slope;
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(j, i) += j * fabs(hForward - hBack) / aLeftRight->rows + hForward;
        }
      }
    }
    return true;
  }

  bool
  TraversabilityMask::findElevatedLeftRightBody(MatPtr aLeftRight, double hForward, double hBack, double d)
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
      // Reject slopes greater than the maximum possible angle.
      if (angle * 180 / CV_PI > description_->maxPossibleAngle)
        return false;

      slope =  tan(angle);
      for (int j = 0; j < aLeftRight->rows; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
          + hBack;
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(robotSize - j, i) += val;
        }
      }
      return true;
    }
    else
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hBack - hForward) / d);
      // Reject slopes greater than the maximum possible angle.
      if (angle * 180 / CV_PI > description_->maxPossibleAngle)
        return false;
      slope =  tan(angle);
      for (int j = 0; j < aLeftRight->rows; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
          + hForward;
        for (int i = 0; i < aLeftRight->cols; ++i)
        {
          aLeftRight->at<double>(j, i) += val;
        }
      }
      return true;
    }
    return true;
  }

  bool
  TraversabilityMask::findElevatedTopBottom(MatPtr aTopBottom, double hLeft, double hRight, double d)
  {
    // Convert the size of the wheel from distance units to the number of cells
    // it corresponds using the current elevation map resolution.
    int wheelSize = metersToSteps(description_->wheelD);
    int robotSize = metersToSteps(description_->totalD);

    // Iterate over the section of the mask for the current pair of wheels
    // and update their corresponding values.
    double slope = fabs(hRight - hLeft) / d;
    double angle = asin(slope);

    // Reject slopes greater than the maximum possible angle.
    if (angle * 180 / CV_PI > description_->maxPossibleAngle)
      return false;
    // Find the wheel that is located higher
    if (hLeft < hRight)
    {
      double slopeResolution = hLeft;
      for (int j = wheelSize; j < aTopBottom->cols - wheelSize; ++j)
      {
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, j) += j * fabs(hRight - hLeft) / aTopBottom->cols + hLeft;
        }
      }
    }
    else
    {
      double slopeResolution = hRight;
      for (int j = wheelSize; j < aTopBottom->cols - wheelSize; ++j)
      {
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, aTopBottom->cols - j - 1) += j * fabs(hRight - hLeft) / aTopBottom->cols
            + hRight;
        }
      }
    }
    return true;
  }

  bool
  TraversabilityMask::findElevatedTopBottomBody(MatPtr aTopBottom, double hLeft, double hRight, double d)
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
      // Reject slopes greater than the maximum possible angle.
      if (angle * 180 / CV_PI > description_->maxPossibleAngle)
        return false;
      slope =  tan(angle);
      for (int j = 0; j < aTopBottom->cols; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
            + hLeft;
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, j) += val;
        }
      }
      return true;
    }
    else
    {
      // Iterate over the section of the mask for the current pair of wheels
      // and update their corresponding values.
      angle = asin((hLeft - hRight) / d);
      // Reject slopes greater than the maximum possible angle.
      if (angle * 180 / CV_PI > description_->maxPossibleAngle)
        return false;
      slope =  tan(angle);
      for (int j = 0; j < aTopBottom->cols; ++j)
      {
        double val = slope * (j * description_->RESOLUTION - description_->wheelD / 2)
            + hRight;
        for (int i = 0; i < aTopBottom->rows; ++i)
        {
          aTopBottom->at<double>(i, robotSize - j) += val;
        }
      }
      return true;
    }

    return true;
  }

  bool
  TraversabilityMask::findHeightOnWheel(const cv::Point2d& wheelPos, double* meanHeight, double* stdDevHeight)
  {
    int wheelSize = metersToSteps(description_->wheelD);
    MatPtr wheelElevation( new cv::Mat(wheelSize, wheelSize, CV_64FC1, cv::Scalar(0)));

    bool known = cropToWheel(wheelPos, wheelElevation);
    if (!known)
      return false;

    cv::Mat validityMask;

    cv::compare(*wheelElevation, - std::numeric_limits<double>::max(), validityMask, cv::CMP_EQ);

    if (cv::countNonZero(validityMask) > 0)
      return false;

    cv::Scalar mean, std_dev;
    cv::meanStdDev(*wheelElevation, mean, std_dev);
    *meanHeight = mean[0];
    *stdDevHeight = std_dev[0];

    return true;
  }

  /**
   * @brief Creates a mask for the given wheel on the elevation map.
  */
  bool
  TraversabilityMask::cropToWheel(const cv::Point2d& wheelPos, const MatPtr& wheel)
  {
    int wheelSize = metersToSteps(description_->wheelD);
    // Copy the region of the elevation map that corresponds to the current wheel.
    cv::Mat roi = (*elevationMapPtr_)(cv::Rect(wheelPos.x, wheelPos.y, wheelSize, wheelSize));

    roi.copyTo(*wheel);
    return true;
  }
}  // namespace pandora_vision_obstacle
}  // namespace pandora_vision
