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
* Author:  Despoina Paschalidou
*          Miltiadis Kofinas, <mkofinas@gmail.com>
*********************************************************************/

#include <vector>
#include "pandora_vision_motion/motion_detector.h"

namespace pandora_vision
{
  /**
    @brief Class Constructor
    Initializes all varialbes for thresholding
  */

  MotionDetector::MotionDetector(void)
  {
    setUpMotionDetector();
    ROS_INFO("Creating MotionDetector instance");
  }


  /**
    @brief Class Destructor
    Deallocates memory used for storing images
  */
  MotionDetector::~MotionDetector()
  {
    ROS_INFO("Destroying MotionDetector instance");
  }

  std::vector<BBoxPOIPtr> MotionDetector::getMotionPosition(void)
  {
    return bounding_boxes_;
  }

  void MotionDetector::setMaxDeviation(int max_deviation)
  {
    max_deviation_ = max_deviation;
  }

  void MotionDetector::setUpMotionDetector(void)
  {
    kernel_erode_ = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    history = 10;
    varThreshold = 16;
    bShadowDetection = false;
    nmixtures = 3;
    diff_threshold = 45;
    motion_high_thres = 7500;
    motion_low_thres = 200;
    visualization = false;
    show_image = false;
    show_background = false;
    show_diff_image = false;
    show_moving_objects_contours = false;
    dbscanEnable_ = false;
    max_deviation_ = 25;
    bg_ = cv::BackgroundSubtractorMOG2(history, varThreshold, bShadowDetection);
    ROS_INFO("Created MotionDetector instance");
  }

   /**
    @brief Function that detects motion, according to substraction
    between background image and current frame_. According to predifined
    thresholds motion is detected. According to the type of motion
    the suitable value is returned.
    @param frame_ [&cv::Mat] current frame to be processed
    @return [int] Index of evaluation of Motion in current frame_.
  */
  void MotionDetector::detectMotion(const cv::Mat& frame)
  {
    frame_ = frame.clone();
    /// Check that frame_ has data and that image has 3 channels
    if (frame_.data && frame_.channels() == 3)
    {
      movingObjects_ = frame_.clone();
      /// Update the background model and create
      /// binary mask for foreground objects
      bg_.operator()(frame_, foreground_);
      bg_.getBackgroundImage(background_);

      cv::subtract(frame_, background_, thresholdedDifference_);
      cv::cvtColor(thresholdedDifference_, thresholdedDifference_, CV_BGR2GRAY);

      cv::threshold(thresholdedDifference_, thresholdedDifference_,
        diff_threshold, 255, cv::THRESH_BINARY);

      cv::Mat kernel = getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3),
        cv::Point(-1, -1));

      cv::morphologyEx(thresholdedDifference_, thresholdedDifference_,
       cv::MORPH_BLACKHAT, kernel, cv::Point(-1, -1), 8);

      detectMotionPosition(thresholdedDifference_);

      if (visualization || show_image ||
        show_background || show_diff_image ||
        show_moving_objects_contours)
      {
        debugShow();
      }
    }
  }

  /**
    @brief Function that calculates motion's position
    @param diff: [&cv::Mat] frame_ that represents
      the thresholded difference between current frame_ and computed
      background.
    @return void
  */
  void MotionDetector::detectMotionPosition(const cv::Mat& diff)
  {
    /// Check that the thresholded difference image has data
    finalBoxes.clear();
    bounding_boxes_.clear();
    if (diff.data)
    {
      /// Calculate the standard deviation
      cv::Scalar mean, stddev;
      meanStdDev(diff, mean, stddev);
      ROS_INFO_STREAM("Motion stdev=" << stddev[0] << " max_deviation= " << max_deviation_);
      /// If not to much changes then the motion is real
      if (stddev[0] < max_deviation_)
      {
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::erode(foreground_, foreground_, cv::Mat());
        cv::dilate(foreground_, foreground_, cv::Mat());

        /// find motion Contours
        cv::findContours(diff.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        cv::Mat mask = cv::Mat::zeros(diff.size(), CV_8UC1);
        cv::drawContours(mask, contours, -1, cv::Scalar(255, 255, 255), CV_FILLED);
        ROS_INFO_STREAM("Contours found=" << contours.size());

        /// find motion Points
        std::vector<cv::Point> motionPoints;
        for(int i = 0; i < diff.cols; i++)
        for (int j = 0; j < diff.rows; j++)
        {
          if(static_cast<int>(mask.at<uchar>(j, i)) == 255)
          {
            motionPoints.push_back(cv::Point(i, j));
          }
        }
        ROS_INFO_STREAM("motion points=" << motionPoints.size());

        /// Start dbscan to cluster motion Points for localization
        std::vector<std::vector<cv::Point> > clusters;

        if(contours.size() > 1 && dbscanEnable_ )
        {
          DBSCAN dbscan(motionPoints, 50, 1);
          dbscan.dbscan_cluster();
          clusters = dbscan.getGroups();
          ROS_INFO_STREAM("CLUSTERS FOUND="<<clusters.size());
          if(clusters.size() > 0)
            // cohesion = dbscan.getCohesion(clusters);
          for (int i = 0; i < cohesion.size(); i++)
          std::cout << cohesion[i] << std::endl;

          /// bound clusters into a box
          for (int i = 0; i < clusters.size(); i++)
          {
            finalBoxes.push_back(mergeBoundingBoxes(clusters[i]));
          }
        }
        else if (motionPoints.size() != 0)
        {
           finalBoxes.push_back(mergeBoundingBoxes(motionPoints));
        }

        float probability;

        for (int i = 0; i < finalBoxes.size(); i++)
        {
          probability = calculateMotionProbability(finalBoxes[i], mask);
          if(probability == 0.0)
            continue;
          bounding_box_.reset( new BBoxPOI() );
          rectangle(movingObjects_, finalBoxes[i].tl(), finalBoxes[i].br(), cv::Scalar(0, 255, 255), 1);
          bounding_box_->setWidth(finalBoxes[i].width);
          bounding_box_->setHeight(finalBoxes[i].height);
          bounding_box_->setPoint(cv::Point(finalBoxes[i].tl().x + finalBoxes[i].width / 2,
                                  finalBoxes[i].tl().y + finalBoxes[i].height / 2));
          bounding_box_->setProbability(probability);
          bounding_boxes_.push_back(bounding_box_);
        }
      }
    }
  }

  float MotionDetector::calculateMotionProbability(const cv::Rect& bbox, const cv::Mat& img)
  {
    int points = 0;
    float prob = 0;
    for(int i = bbox.x; i < bbox.x + bbox.width; i++)
      for(int j = bbox.y; j < bbox.y + bbox.height; j++)
      {
        if(static_cast<int>(img.at<uchar>(j, i)) == 255)
        {
          points++;
        }
      }
    if (points <= motion_high_thres && points >= motion_low_thres)
      prob = 1.0; //points / static_cast<float>(bbox.width * bbox.height);
    if (points > motion_high_thres)
      prob = 0.51;
    ROS_INFO_STREAM("PROB=" << prob << " points=" << points);
    return prob;
  }


  /**
    @brief Function that defines the type of movement
    according to the number of pixels, that differ from current
    frame_ and background. In case insignificant motion 0 is detected
    0 is returned. If there is slight motion 1 is returned and last
    bust not least in case extensive motion is detected 2 is returned
    @param thresholdedDifference_: [&cv::Mat] frame_ that represents
      the thresholded difference between current frame_ and computed
      background
    @return typeOfMovement [int], where 2 corresponds to moving objects
    with greater probability whereas 0 corresponds to stationary objects
  */
  int MotionDetector::motionIdentification(const cv::Mat& thresholdedDifference_)
  {
    //!< Counts value of non zero pixels in binary image in order
    //!< to find the exact number of pixels, that differ from current
    //!< frame_ and background
    int countDiff = countNonZero(thresholdedDifference_);
    if (countDiff > motion_high_thres)
      return 2;
    else if (countDiff > motion_low_thres)
      return 1;
    else
      return 0;
  }


  /**
    @brief Function used for debug reasons, that shows background
    foreground and contours of motion trajectories in current frame_
    @return void
  */
  void MotionDetector::debugShow()
  {
    std::vector<cv::Scalar> colors;
    cv::RNG rng(3);

    for(int i = 0; i <= finalBoxes.size(); i++)
    {
        colors.push_back(HSVtoRGBcvScalar(rng(255),255,255));
    }
    for(int i = 0; i < finalBoxes.size(); i++)
    {
      cv::Scalar color;
      int label = i ;
      color = colors[label];
      putText(thresholdedDifference_, to_string(i), finalBoxes[i].tl(), cv::FONT_HERSHEY_COMPLEX, .5, color, 1);
      cv::rectangle(thresholdedDifference_, finalBoxes[i].tl(), finalBoxes[i].br(), color, 2, CV_AA);
    }
    cv::Mat frame, background, temp, movingObjects;
    cv::resize(frame_, frame, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);
    cv::resize(background_, background, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);
    cv::resize(thresholdedDifference_, temp, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);
    cv::resize(movingObjects_, movingObjects, cv::Size(0, 0), 0.75, 0.75, cv::INTER_AREA);

    cv::Mat temp1[] = {temp, temp, temp};
    cv::Mat diff;
    cv::merge(temp1, 3, diff);

    cv::Mat displayImage = cv::Mat(frame.rows * 2, frame.cols * 2, frame.type());
    frame.copyTo(displayImage(cv::Rect(0, 0, frame.cols,frame.rows)));
    background.copyTo(displayImage(cv::Rect(background.cols, 0, background.cols, background.rows)));
    diff.copyTo(displayImage(cv::Rect(0, diff.rows, diff.cols, diff.rows)));
    movingObjects.copyTo(displayImage(cv::Rect(movingObjects.cols, movingObjects.rows,
                                     movingObjects.cols, movingObjects.rows)));

    if (visualization)
      cv::imshow("Original/Background/ThresholdedDiff/MovingObjects", displayImage);
    if (show_image)
      cv::imshow("Frame", frame);
    if (show_background)
      cv::imshow("Background", background);
    if (show_diff_image)
      cv::imshow("Thresholded difference between background and current frame_", diff);
    if (show_moving_objects_contours)
      cv::imshow("Moving objects in current frame", movingObjects);
    cv::waitKey(10);
  }

  cv::Rect MotionDetector::mergeBoundingBoxes(std::vector<cv::Point>& bbox)
  {
    int min_x = movingObjects_.cols;
    int max_x = 0;
    int min_y = movingObjects_.rows;
    int max_y = 0;
    for(int i = 0; i < bbox.size(); i++)
    {
      if(min_x > bbox[i].x)
        min_x = bbox[i].x;
      if(min_y > bbox[i].y)
        min_y = bbox[i].y;
      if(max_x < bbox[i].x)
        max_x = bbox[i].x;
      if(max_y < bbox[i].y)
        max_y = bbox[i].y;
    }

    cv::Rect r = cv::Rect(min_x, min_y, max_x-min_x+1, max_y-min_y+1);
    // ROS_INFO_STREAM("final RECT="<<r.tl() << " " << r.br());
    return r;
  }

  cv::Scalar MotionDetector::HSVtoRGBcvScalar(int H, int S, int V)
  {

    int bH = H; // H component
    int bS = S; // S component
    int bV = V; // V component
    double fH, fS, fV;
    double fR, fG, fB;
    const double double_TO_BYTE = 255.0f;
    const double BYTE_TO_double = 1.0f / double_TO_BYTE;

    // Convert from 8-bit integers to doubles
    fH = (double)bH * BYTE_TO_double;
    fS = (double)bS * BYTE_TO_double;
    fV = (double)bV * BYTE_TO_double;

    // Convert from HSV to RGB, using double ranges 0.0 to 1.0
    int iI;
    double fI, fF, p, q, t;

    if( bS == 0 )
    {
      // achromatic (grey)
      fR = fG = fB = fV;
    }
    else
    {
        // If Hue == 1.0, then wrap it around the circle to 0.0
        if (fH>= 1.0f)
            fH = 0.0f;

        fH *= 6.0; // sector 0 to 5
        fI = floor( fH ); // integer part of h (0,1,2,3,4,5 or 6)
        iI = (int) fH; // " " " "
        fF = fH - fI; // factorial part of h (0 to 1)

        p = fV * ( 1.0f - fS );
        q = fV * ( 1.0f - fS * fF );
        t = fV * ( 1.0f - fS * ( 1.0f - fF ) );

        switch( iI )
        {
        case 0:
            fR = fV;
            fG = t;
            fB = p;
            break;
        case 1:
            fR = q;
            fG = fV;
            fB = p;
            break;
        case 2:
            fR = p;
            fG = fV;
            fB = t;
            break;
        case 3:
            fR = p;
            fG = q;
            fB = fV;
            break;
        case 4:
            fR = t;
            fG = p;
            fB = fV;
            break;
        default: // case 5 (or 6):
            fR = fV;
            fG = p;
            fB = q;
            break;
        }
    }

    // Convert from doubles to 8-bit integers
    int bR = (int)(fR * double_TO_BYTE);
    int bG = (int)(fG * double_TO_BYTE);
    int bB = (int)(fB * double_TO_BYTE);

    // Clip the values to make sure it fits within the 8bits.
    if (bR > 255)
        bR = 255;
    if (bR < 0)
        bR = 0;
    if (bG >255)
        bG = 255;
    if (bG < 0)
        bG = 0;
    if (bB > 255)
        bB = 255;
    if (bB < 0)
        bB = 0;

    // Set the RGB cvScalar with G B R, you can use this values as you want too..
    return cv::Scalar(bB,bG,bR); // R component
  }
}  // namespace pandora_vision
