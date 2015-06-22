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

  BBoxPOIPtr MotionDetector::getMotionPosition(void)
  {
    return bounding_box_;
  }

  void MotionDetector::setMaxDeviation(int max_deviation)
  {
    max_deviation_ = max_deviation;
  }

  void MotionDetector::setUpMotionDetector(void)
  {
    bounding_box_.reset( new BBoxPOI() );
    kernel_erode_ = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));

    bounding_box_->setPoint(cv::Point(0, 0));
    bounding_box_->setWidth(0);
    bounding_box_->setHeight(0);
    bounding_box_->setProbability(0.0f);
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
    indexFrame = 0;

    max_deviation_ = 16;
    bg_ = cv::BackgroundSubtractorMOG2(history, varThreshold, bShadowDetection);
    ROS_INFO("Created MotionDetector instance");
  }

   /**
    @brief Function that detects motion, according to substraction
    between background image and current frame. According to predifined
    thresholds motion is detected. According to the type of motion
    the suitable value is returned.
    @param frame [&cv::Mat] current frame to be processed
    @return [int] Index of evaluation of Motion in current frame.
  */
  int MotionDetector::detectMotion(const cv::Mat& frame)
  {
    /// Check that frame has data and that image has 3 channels
    if (frame.data && frame.channels() == 3)
    {
      indexFrame++;
      movingObjects_ = frame.clone();
      /// Update the background model and create
      /// binary mask for foreground objects
      bg_.operator()(frame, foreground_);
      bg_.getBackgroundImage(background_);

      cv::Mat thresholdedDifference;
      cv::subtract(frame, background_, thresholdedDifference);
      cv::cvtColor(thresholdedDifference, thresholdedDifference, CV_BGR2GRAY);

      cv::threshold(thresholdedDifference, thresholdedDifference,
        diff_threshold, 255, cv::THRESH_BINARY);


      cv::Mat kernel = getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3),
        cv::Point(-1, -1));

      cv::morphologyEx(thresholdedDifference, thresholdedDifference,
        cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 8);


      detectMotionPosition(thresholdedDifference);

      cv::Point boxCenter(floor(bounding_box_->getPoint().x + bounding_box_->getWidth() * 0.5),
        floor(bounding_box_->getPoint().y + bounding_box_->getHeight() * 0.5));
      bounding_box_->setPoint(boxCenter);

      if (visualization || show_image ||
        show_background || show_diff_image ||
        show_moving_objects_contours)
      {
        debugShow(thresholdedDifference, frame);
      }
      return typeOfMovement_;
    }
    else
    {
      return 0;
    }
  }

  /**
   * @brief
   **/

  void MotionDetector::findMotionParameters(const cv::Mat& frame)
  {
    switch (detectMotion(frame))
    {
      case 0:
        bounding_box_->setProbability(0);
        break;
      case 1:
        bounding_box_->setProbability(0.51);
        break;
      case 2:
        bounding_box_->setProbability(1);
        break;
      default:
        bounding_box_->setProbability(-1);
        break;
    }
  }

  BBoxPOIPtr MotionDetector::getBoundingBox()
  {
    return bounding_box_;
  }

  /**
    @brief Function that calculates motion's position
    @param diff: [&cv::Mat] frame that represents
      the thresholded difference between current frame and computed
      background.
    @return void
  */
  void MotionDetector::detectMotionPosition(const cv::Mat& diff)
  {
    /// Check that the thresholded difference image has data
    if (diff.data)
    {
      /// Calculate the standard deviation
      cv::Scalar mean, stddev;
      meanStdDev(diff, mean, stddev);
      ROS_INFO_STREAM("Motion stdev=" << stddev[0] << " max_deviation= " << max_deviation_);
      /// If not to much changes then the motion is real
      if (stddev[0] < max_deviation_)
      {
        typeOfMovement_ = motionIdentification(diff);

        int number_of_changes = 0;
        int min_x = diff.cols, max_x = 0;
        int min_y = diff.rows, max_y = 0;
        /// Loop over image and detect changes
        for (int i = 0; i < diff.rows; i++)
        {
          for (int j = 0; j < diff.cols; j++)
          {
            if (static_cast<int>(diff.at<uchar>(i, j)) == 255)
            {
              number_of_changes++;
              if (min_y > i)
                min_y = i;
              if (max_y < i)
                max_y = i;
              if (min_x > j)
                min_x = j;
              if (max_x < j)
                max_x = j;
            }
          }
        }
        if (number_of_changes)
        {
          cv::Point _tlcorner(min_x, min_y);
          cv::Point _brcorner(max_x, max_y);
          rectangle(movingObjects_, _tlcorner, _brcorner, cv::Scalar(0, 255, 255), 1);
          bounding_box_->setWidth(max_x - min_x + 1);
          bounding_box_->setHeight(max_y - min_y + 1);
          bounding_box_->setPoint(_tlcorner);
        }
      }
    }
  }

  /**
    @brief Function that defines the type of movement
    according to the number of pixels, that differ from current
    frame and background. In case insignificant motion 0 is detected
    0 is returned. If there is slight motion 1 is returned and last
    bust not least in case extensive motion is detected 2 is returned
    @param thresholdedDifference: [&cv::Mat] frame that represents
      the thresholded difference between current frame and computed
      background
    @return typeOfMovement [int], where 2 corresponds to moving objects
    with greater probability whereas 0 corresponds to stationary objects
  */
  int MotionDetector::motionIdentification(const cv::Mat& thresholdedDifference)
  {
    //!< Counts value of non zero pixels in binary image in order
    //!< to find the exact number of pixels, that differ from current
    //!< frame and background
    int countDiff = countNonZero(thresholdedDifference);
    if (countDiff > motion_high_thres)
      return 2;
    else if (countDiff > motion_low_thres)
      return 1;
    else
      return 0;
  }


  /**
    @brief Function used for debug reasons, that shows background
    foreground and contours of motion trajectories in current frame
    @param thresholdedDifference: [&cv::Mat] frame that represents
      the thresholded difference between current frame and computed
      background.
    @param frame: [&cv::Mat] current frame, captured from camera
    @return void
  */
  void MotionDetector::debugShow(
    const cv::Mat& thresholdedDifference,
    const cv::Mat& frame
  )
  {
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::erode(foreground_, foreground_, cv::Mat());
    cv::dilate(foreground_, foreground_, cv::Mat());
    cv::findContours(thresholdedDifference.clone(), contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 2);
    ROS_INFO_STREAM("Contours found=" << contours.size());
    std::vector<double> areas(contours.size());
    //!< find largest contour area
 
    // for (int i = 0; i < contours.size(); i++) 
    // {
            // areas[i] = cv::contourArea(cv::Mat(contours[i]));
            // ROS_INFO_STREAM("Area of contour=" << areas[i]);
    /* }  */

    std::vector<cv::Rect> boxes, finalBoxes, noiseBoxes;
    std::vector<std::vector<cv::Rect> > clusters;

    for(int i = 0; i < contours.size(); i++)
    {
        cv::Rect r = cv::boundingRect(contours[i]);
        boxes.push_back(r);
        ROS_INFO_STREAM("RECT[" << i << "]=" << r.x <<" " <<r.y
                        << " "<< r.width<< " " << r.height);
    }

    DBSCAN dbscan(boxes, 20, 1);
    dbscan.dbscan_cluster();
    clusters = dbscan.getGroups(&noiseBoxes);
    std::vector<cv::Rect> temp;
    // cv::groupRectangles(clusters[i],1,100000);
    for (int i = 0; i < clusters.size(); i++)
    {

      finalBoxes.push_back(mergeBoundingBoxes(clusters[i]));
    }

        cv::Mat grouped = cv::Mat::zeros(frame.size(),CV_8UC3);
    std::vector<cv::Scalar> colors;
    cv::RNG rng(3);

    for(int i = 0; i <= dbscan._cluster_id; i++)
    {
        colors.push_back(HSVtoRGBcvScalar(rng(255),255,255));
    }
    for(int i = 0; i < dbscan._data.size(); i++)
    {
      cv::Scalar color;
        if(dbscan._labels[i] == -1)
        {
            color = cv::Scalar(128,128,128);
        }
        else
        {
            int label = dbscan._labels[i];
            color = colors[label];
        }
        putText(grouped, to_string(dbscan._labels[i]), dbscan._data[i].tl(), cv::FONT_HERSHEY_COMPLEX, .5, color, 1);
        drawContours(grouped, contours, i, color, -1);
    }
    for(int i = 0; i < finalBoxes.size(); i++)
    {
      cv::rectangle(grouped,finalBoxes[i].tl(),finalBoxes[i].br(), cv::Scalar(100,100,200), 2, CV_AA);
    }
    
    for(int i = 0; i < noiseBoxes.size(); i++)
    {
      cv::rectangle(grouped,noiseBoxes[i].tl(),noiseBoxes[i].br(), cv::Scalar(100,100,200), 2, CV_AA);
    }


    imshow("grouped", grouped);
    //!< get index of largest contour
    /*double max;
    cv::Point maxPosition;*/
    /*cv::minMaxLoc(cv::Mat(areas),0,&max,0,&maxPosition);*/

    if (visualization || show_image)
      cv::imshow("Frame", frame);
    if (visualization || show_background)
      cv::imshow("Background", background_);
    if (visualization || show_diff_image)
      cv::imshow("Thresholded difference between background and current frame", thresholdedDifference);
    if (visualization || show_moving_objects_contours)
      cv::imshow("Moving objects in current frame", movingObjects_);
    cv::waitKey(10);

    contours.clear();
  }

  cv::Rect MotionDetector::mergeBoundingBoxes(std::vector<cv::Rect>& bbox)
  {
    int min_x = movingObjects_.cols;
    int max_x = 0;
    int min_y = movingObjects_.rows;
    int max_y = 0;
    for(int i = 0; i < bbox.size(); i++)
    {
      ROS_INFO_STREAM("RECT="<<bbox[i].tl() << " " << bbox[i].br());
      if(min_x > bbox[i].tl().x)
        min_x = bbox[i].tl().x;
      if(min_y > bbox[i].tl().y)
        min_y = bbox[i].tl().y;
      if(max_x < bbox[i].br().x)
        max_x = bbox[i].br().x;
      if(max_y < bbox[i].br().y)
        max_y = bbox[i].br().y;
    }

    cv::Rect r=cv::Rect(min_x, min_y, max_x-min_x, max_y-min_y);

    ROS_INFO_STREAM("final RECT="<<r.tl() << " " << r.br());
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
