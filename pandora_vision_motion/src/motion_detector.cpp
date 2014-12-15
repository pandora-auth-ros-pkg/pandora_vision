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
*********************************************************************/

#include "pandora_vision_motion/motion_detector.h"

namespace pandora_vision 
{
  
  /**
    @brief Class Constructor
    Initializes all varialbes for thresholding
  */
  MotionDetector::MotionDetector()
  { 
    kernel_erode_ = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    bg_ = cv::BackgroundSubtractorMOG2(
            MotionParameters::history,
            MotionParameters::varThreshold, 
            MotionParameters::bShadowDetection
        );
    bounding_box_.x = 0;
    bounding_box_.y = 0;
    bounding_box_.width = 0;
    bounding_box_.height = 0;
    
    max_deviation_ = 50;
    ROS_INFO("Created MotionDetector instance");
  }

  /**
    @brief Class Destructor
    Deallocates memory used for storing images
  */
  MotionDetector::~MotionDetector()
  {
    ROS_INFO("Destroying MotionDetector instance");
  }

  /**
    @brief Function that detects motion, according to substraction
    between background image and current frame. According to predifined 
    thresholds motion is detected. According to the type of motion
    the suitable value is returned.
    @param frame [&cv::Mat] current frame to be processed 
    @return [int] Index of evaluation of Motion in current frame.
  */
  int MotionDetector::detectMotion(cv::Mat &frame)
  {
    movingObjects_ = frame.clone(); 
    /// Upadate the background model and create 
    /// binary mask for foreground objects
    bg_.operator()(frame, foreground_);
    bg_.getBackgroundImage(background_);
        
    cv::Mat thresholdedDifference;
    cv::subtract(frame, background_, thresholdedDifference);
    cv::cvtColor(thresholdedDifference, thresholdedDifference, CV_BGR2GRAY);
    cv::threshold(
        thresholdedDifference, 
        thresholdedDifference, 
        MotionParameters::diff_threshold, 
        255, 
        cv::THRESH_BINARY
    );
      
     
    int typeOfMovement = motionIdentification(thresholdedDifference);
    cv::Mat kernel = getStructuringElement(
      cv::MORPH_CROSS , 
      cv::Size(3, 3), 
      cv::Point( -1, -1 )
    );
    
    cv::morphologyEx(
      thresholdedDifference, 
      thresholdedDifference, 
      cv::MORPH_CLOSE, 
      kernel, 
      cv::Point(-1, -1), 
      8
    );
 
    detectMotionPosition(thresholdedDifference);
    
    if(MotionParameters::visualization ||
        MotionParameters::show_image ||
        MotionParameters::show_background ||
        MotionParameters::show_diff_image ||
        MotionParameters::show_moving_objects_contours 
    )
    debugShow(thresholdedDifference, frame);
    
    return typeOfMovement;
  }
  
  /**
    @brief Function that calculates motion's postion
    @param diff: [cv::Mat] frame that represents
      the thresholded difference between current frame and computed 
      background.
    @return void 
  */
  void MotionDetector::detectMotionPosition(cv::Mat diff)
  {
    /// Calculate the standard deviation
    cv::Scalar mean, stddev;
    meanStdDev(diff, mean, stddev);
    /// If not to much changes then the motion is real 
    if(stddev[0] < max_deviation_)
    {
      int number_of_changes = 0;
      int min_x = diff.cols, max_x = 0;
      int min_y = diff.rows, max_y = 0;
      /// Loop over image and detect changes
      for(int j = 1; j < diff.cols-11; j+=2){ 
        for(int i = 1; i < diff.rows-11; i+=2){
            if(static_cast<int>(diff.at<uchar>(j, i)) == 255){
                number_of_changes++;
                if(min_x > i) 
                  min_x = i;
                if(max_x < i) 
                  max_x = i;
                if(min_y > j) 
                  min_y = j;
                if(max_y < j) 
                  max_y = j;
                }
            }
        }
      if(number_of_changes){
        cv::Point _tlcorner(min_x, min_y);
        cv::Point _brcorner(max_x, max_y);
        
        rectangle(movingObjects_, _tlcorner, _brcorner, cv::Scalar(0, 255, 255), 1);
        bounding_box_.x = _tlcorner.x;
        bounding_box_.y = _tlcorner.y;
        bounding_box_.width = max_x - min_x;
        bounding_box_.height = max_y - min_y;
      }  
    }
  }
  
  /**
    @brief Creates the continuous table of motion in current frame
    @return [int[]] table of motion position and size
  */
   int* MotionDetector::getMotionPosition()
  {
    int *motion_position = new int[4];
    //!< Center_x
    motion_position[0] = round( 
      bounding_box_.x + bounding_box_.width * 0.5 
    );
    //!< Center_y
    motion_position[1] = round( 
      bounding_box_.y + bounding_box_.height * 0.5 
    );
    //!< Rectangle width
    motion_position[2] = bounding_box_.width;
    //!< Rectangle height
    motion_position[3] = bounding_box_.height;
    
    return motion_position;
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
  int MotionDetector::motionIdentification(
    const cv::Mat &thresholdedDifference
  )
  {
    //!< Counts value of non zero pixels in binary image in order 
    //!< to find the exact number of pixels, that differ from current 
    //!< frame and background 
    int countDiff = countNonZero(thresholdedDifference);
    if (countDiff > MotionParameters::motion_high_thres)
      return 2;
    else if (countDiff > MotionParameters::motion_low_thres)
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
    const cv::Mat &thresholdedDifference, 
    const cv::Mat &frame
  )
  {
    std::vector<std::vector<cv::Point> > contours;
    cv::erode(foreground_, foreground_, cv::Mat());
    cv::dilate(foreground_, foreground_, cv::Mat());
    cv::findContours(
      foreground_, 
      contours, 
      CV_RETR_EXTERNAL, 
      CV_CHAIN_APPROX_NONE
    );
    cv::drawContours(frame, contours, -1, cv::Scalar(0, 0, 255), 2);
    if(MotionParameters::visualization || MotionParameters::show_image)
      cv::imshow("Frame", frame);
    if(MotionParameters::visualization || MotionParameters::show_background)
      cv::imshow("Background", background_);
    if(MotionParameters::visualization || MotionParameters::show_diff_image)
      cv::imshow("Thresholded difference between background and current frame", thresholdedDifference);
    if(MotionParameters::visualization || MotionParameters::show_moving_objects_contours)
      cv::imshow("Moving objects in current frame", movingObjects_);
    cv::waitKey(10);
    
    contours.clear();
  }
     
}// namespace pandora_vision
