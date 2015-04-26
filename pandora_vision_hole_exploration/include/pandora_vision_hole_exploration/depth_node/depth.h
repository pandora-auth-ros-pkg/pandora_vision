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
 * Authors: Vasilis Bosdelekidis, Alexandros Philotheou
 *********************************************************************/

#ifndef DEPTH_NODE_DEPTH_H
#define DEPTH_NODE_DEPTH_H

#include "pandora_vision_msgs/CandidateHolesVectorMsg.h"
#include "utils/message_conversions.h"
#include "utils/holes_conveyor.h"
#include "utils/parameters.h"
#include "utils/visualization.h"

/**
  @namespace pandora_vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class Depth
    @brief Provides functionalities for locating holes via
    analysis of a Depth image
   **/
  class Depth
  {
    private:

      // The NodeHandle
      ros::NodeHandle nodeHandle_;

      // Subscriber of Kinect point cloud
      ros::Subscriber depthImageSubscriber_;

      // The name of the topic where the depth image is acquired from
      std::string depthImageTopic_;

      // The ROS publisher ofcandidate holes
      ros::Publisher candidateHolesPublisher_;

      // The name of the topic where the candidate holes that the depth node
      // locates are published to
      std::string candidateHolesTopic_;

      // The dynamic reconfigure (depth) parameters' server
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::depth_cfgConfig> server;

      // The dynamic reconfigure (depth) parameters' callback
      dynamic_reconfigure::Server<pandora_vision_hole_exploration::depth_cfgConfig>::CallbackType f;

      /**
        @brief Callback for the depth image received by the synchronizer node.

        The depth image message received by the synchronizer node is unpacked
        in a cv::Mat image. After basic filtering, holes are located inside this        image and information about them, along with the depth image, is then se        nt to the hole fusion node;
        @param msg [const sensor_msgs::Image&] The depth image message
        @return void
       **/
      void inputDepthImageCallback(const sensor_msgs::Image& inImage);

      /**
        @brief Acquires topics' names needed to be subscribed to and advertise
        to by the depth node
        @param void
        @return void
       **/
      void getTopicNames();

      /**
        @brief The function called when a parameter is changed
        @param[in] config [const pandora_vision_hole::depth_cfgConfig&]
        @param[in] level [const uint32_t]
        @return void
       **/
      void parametersCallback(
          const pandora_vision_hole_exploration::depth_cfgConfig& config,
          const uint32_t& level);


    public:

      // The constructor
      Depth(void);

      /**
        @brief The function called to extract holes from Depth image
        @param[in] depthImage [const cv::Mat&] The Depth image to be processed,
        in CV_32FC1 format
        @return [HolesConveyor] A struct with useful info about each hole.
       **/
      static HolesConveyor findHoles(const cv::Mat& depthImage);

      /**
        @brief The function called to filter the depth image, by applying simple thresholding at 0, simple morhology transformations and to eliminate (make dark) the region at borders.
        @param[in] depthImage [const cv::Mat&] The Depth image to be processed,
        in CV_8UC3 format
        @param[in] filteredImage [cv::Mat* filteredImage] The output filtered binary image.
        @return void
       **/
      static void filterImage(const cv::Mat& depthImage, cv::Mat* filteredImage);


      /**
        @brief The function called to extract contours that were not ignored by the initial filtering held in filterImage function.
        @param[in] filteredImage [cv::Mat&] The filtered image represented as white edges where there was a small depth. Remember that there were only edges left, which were black at the original depth image and were not random black pixels.
        @param[in] contours [std::vector<std::vector<cv::Point>>*] The contours found.
        @return void
       **/
      static void detectContours(const cv::Mat& filteredImage, std::vector<std::vector<cv::Point> >* contours);


      /**
        @brief The function called to estimate center of mass for each contour found and bounding boxes.
        @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
        @param[in] mc [std::vector<cv::Point2f>*] Center of mass of each contour as x, y coordinates..
        @return void
       **/
      static void getContourInfo(std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Point2f>* mc, std::vector<cv::Rect>* boundRect);

      /**
        @brief The function called to make validation of found contours
        @param[in] image [cv::Mat&] The original image. Used to extract some features to evaluate contour size, similarity etc.
        @param[in] contours [std::vector<std::vector<cv::Point>>&] The contours found before.
        @param[in] mc [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
        @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
        @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
        @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
        @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
        @return void
       **/
      static void validateContours(const cv::Mat& image, std::vector<std::vector<cv::Point> >& contours, std::vector<cv::Point2f>* mc, std::vector<int>* contourHeight, std::vector<int>* contourWidth, std::vector<bool>* realContours, std::vector<cv::Rect>& boundRect);

      /**
        @brief The function called by validateContours to make validation of a single contour
        @param[in] image [const cv::Mat&] The depth image 
        @param[in] ci [int] Current contour index in contours vector 
        @param[in] mcv [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
        @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
        @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
        @param[in] contourLabel [std::map<std::pair<int, int>, float>*] A map of contours relationship represented as current contour & some other contour as key and a probability of shame contour calculated via the values of some features between the two contours.
        @param[in] numLabels [std::vector<int>*] For each contour a counter of how many times it appears in the abovementioned map. In the map are strored only pairs of contours whose merging probability is above some threshold.
        @param[in] boundRect [std::vector<cv::Rect>&] A vector containing the bounding rectangles for each contour 
        @param[in] contours [std::vector<std::vector<cv::Point> >&] All contours found. 
        @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. 
        @return void
       **/
      static bool validateContour(const cv::Mat& image, int ci, std::vector<cv::Point2f>* mcv, std::vector<int>* contourHeight, std::vector<int>* contourWidth, std::map<std::pair<int, int>, float>* contourLabel, std::vector<int>* numLabels, std::vector<cv::Rect>& boundRect, std::vector<std::vector<cv::Point> >& contours, std::vector<bool>* realContours);

      /**
        @brief The function called by validateContours to do the final merging after the probabilities for each pair were found in validateContour. New contourwidths and heights are calculated for merged contours. New coordinates for the merged contour as the average of all the contours that consist it are calculated.
        @param[in] ci [int] Current contour index in contours vector 
        @param[in] mcv [std::vector<Point2f>*] A vector containing coordinates of the centers of mass of all the contours found 
        @param[in] contourHeight [std::vector<int>*] A vector containing the overall contours' heights 
        @param[in] contourWidth [std::vector<int>*] A vector containing the overall contours' widths 
        @param[in] contourLabel [std::map<std::pair<int, int>, float>*] A map of contours relationship represented as current contour & some other contour as key and a probability of shame contour calculated via the values of some features between the two contours.
        @param[in] numLabels [std::vector<int>*] For each contour a counter of how many times it appears in the abovementioned map. In the map are strored only pairs of contours whose merging probability is above some threshold.
        @param[in] realContours [std::vector<bool>*] Contains flags if a contour is valid or not. Calculated inside this function.
        @param[in] contours [std::vector<std::vector<cv::Point> >&] All contours found. 
        @return void
       **/
      static void mergeContours(int ci, std::vector<cv::Point2f>* mcv, std::vector<int>* contourHeight, std::vector<int>* contourWidth, std::map<std::pair<int, int>, float>* contourLabel, std::vector<int>* numLabels, std::vector<bool>* realContours, std::vector<std::vector<cv::Point> >& contours);

        // The destructor
        ~Depth(void);
  };

} //namespace pandora_vision

#endif  // DEPTH_NODE_DEPTH_H

