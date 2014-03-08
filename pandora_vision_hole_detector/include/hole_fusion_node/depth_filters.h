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
 * Authors: Alexandros Filotheou, Manos Tsardoulias
 *********************************************************************/

#ifndef HOLE_FUSION_NODE_DEPTH_FILTERS_H
#define HOLE_FUSION_NODE_DEPTH_FILTERS_H

#include "depth_node/hole_filters.h"
#include "hole_fusion_node/planes_detection.h"
#include <math.h>

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class HoleDetector
    @brief Provides the functionalities for detecting holes [functional]
   **/
  class DepthFilters
  {
    public:

      /**
        @brief Checks for valid holes just by depth difference between the
        center of the blob and the edges of the bounding box
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
        The bounding boxes' vertices
        @param[in][out] msgs [std::vector<std::string>*] Messages for debug
        reasons
        @param[in] inflationSize [const int&] The number of pixels by which the
        bounding rectange will be inflated
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesDepthDiff(
        const cv::Mat& depthImage,
        const std::vector<cv::KeyPoint>& inKeyPoints,
        const std::vector<std::vector<cv::Point2f> >& inRectangles,
        std::vector<std::string>* msgs,
        const int& inflationSize);

      /**
        @brief Checks for valid holes by area / depth comparison
        @param[in] depthImage [const cv::Mat&] The depth image
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
        The bounding boxes' vertices
        @param[in][out] msgs [std::vector<std::string>*] Messages for debug
        reasons
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesDepthArea(
        const cv::Mat& depthImage,
        const std::vector<cv::KeyPoint>& inKeyPoints,
        const std::vector<std::vector<cv::Point2f> >& inRectangles,
        std::vector<std::string>* msgs);

      /**
        @brief Brushfire from a blobs's outline to its bounding box
        with an inflation size (inflates the rectangle by inflationSize pixels).
        If the points between the blob's outline and the inflated rectangle
        lie on one plane, this blob is a hole.
        @param[in] inImage [const cv::Mat&] The input depth image
        @param[in] initialPointCloud
        [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
        The original point cloud acquired from the depth sensor
        @param[in] keyPoints [const std::vector<cv::KeyPoint>&] The keypoints of
        blobs
        @param[in] outlines [const std::vector<std::vector<cv::point> >&] The
        points the outline consists of
        @param[in] rectangles [const std::vector<std::vector<cv::point2f> >&]
        The bounding boxes' vertices
        @param[in] inflationsize [const int&] Grow the rectangle by
        inflationsize as to acquire more points to check for plane existence.
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesBrushfireOutlineToRectangle(
        const cv::Mat& inImage,
        const PointCloudXYZPtr& initialPointCloud,
        const std::vector<cv::KeyPoint>& keyPoints,
        const std::vector<std::vector<cv::Point> >& outlines,
        const std::vector<std::vector<cv::Point2f> >& rectangles,
        const int& inflationSize);

      /**
        @brief Given the bounding box of a blob, inflate it.
        All the points that lie on the (edges of the) rectangle should
        also lie on exactly one plane for the blob to be a hole.
        @param[in] inImage [const cv::Mat&] The input depth image
        @param[in] initialPointCloud
        [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
        The original point cloud,  uninterpolated, undistorted.
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
        of blobs
        @param[in] rectangles [const std::vector<std::vector<cv::point2f> >&]
        The bounding boxes' vertices
        @param[in] inflationsize [cosnt int&] grow the rectangle by
        inflationsize as to acquire more points to check for plane existence.
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesRectangleOutline(
        const cv::Mat& inImage,
        const PointCloudXYZPtr& initialPointCloud,
        const std::vector<cv::KeyPoint>& inKeyPoints,
        const std::vector<std::vector<cv::Point2f> >& rectangles,
        const int& inflationSize);

      /**
        @brief Checks the homogenity of the gradient of depth in an area
        enclosed by @param inOutlines
        @param[in] interpolatedDepthImage [const cv::Mat&] The input depth image
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The keypoints
        of blobs
        @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
        @param[out] msgs [std::vector<std::string>*] Debug messages
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree which with the
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesDepthHomogenity(
        const cv::Mat& interpolatedDepthImage,
        const std::vector<cv::KeyPoint>& inKeyPoints,
        const std::vector<std::vector<cv::Point> >& inOutlines,
        std::vector<std::string>* msgs,
        std::vector<float>* probabilitiesVector);

        /**
          @brief Apply a cascade-like hole checker. Each filter applied is
          attached to an order which relates to the sequence of the overall
          filter execution.
          @param[in] interpolatedDepthImage [cv::Mat&] The denoised depth image
          @param[in] initialPointCloud
          [const pcl::PointCloud<pcl::PointXYZ>::Ptr&]
          The undistorted input point cloud
          @param[in][out] conveyor [HoleFilters::HolesConveyor*] A struct that
          contains the final valid holes
          @return void
         **/
        static void checkHoles(
          const cv::Mat& interpolatedDepthImage,
          const pcl::PointCloud<pcl::PointXYZ>::Ptr& initialPointCloud,
          HoleFilters::HolesConveyor* conveyor);

      /**
        @brief Apply a cascade-like hole checker. Each filter applied
        is attached to an order which relates to the sequence of the overall
        filter execution.
        @param[in] method [const unsigned int&] The filter identifier to execute
        @param[in] img [const cv::Mat&] The input depth image
        @param[in] pointCloud [const pcl::PointCloud<pcl::PointXYZ>::Ptr&] The
        original point cloud that corresponds to the input depth image
        @param[in][out] conveyor [HoleFilters::HolesConveyor*] The structure
        that holds the final holes' data
        @param[in] inflationSize [const int&] The amount of pixels by which each
        bounding box is inflated
        @param[in][out] imgs [std::vector<cv::Mat>*] A vector of images which
        shows the holes that are considered valid by each filter
        @param[in][out] msgs [std::vector<std::string>*] Debug messages
        @return void
       **/
      static void applyFilter(
        const unsigned int& method,
        const cv::Mat& img,
        const PointCloudXYZPtr& pointCloud,
        HoleFilters::HolesConveyor* conveyor,
        const int& inflationSize,
        std::vector<cv::Mat>* imgs,
        std::vector<std::string>* msgs);
  };

}

#endif
