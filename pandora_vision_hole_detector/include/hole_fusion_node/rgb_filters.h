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
#ifndef RGB_FILTERS_H
#define RGB_FILTERS_H

#include <math.h>
#include "depth_node/hole_filters.h"
#include "hole_fusion_node/hole_fusion_parameters.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class RgbFilters
    @brief Provides functionalities of hole filters relevant to RGB images
   **/
  class RgbFilters
  {
    public:

      /**
        @brief Checks for difference of mean value of luminosity between the
        pixels that comprise the blob's bounding box edges and the points
        inside the blob's outline.
        @param[in] inImage [const cv::Mat&] The RGB image
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The vector
        of the candidate holes's keypoints
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
        The vector of the candidate holes's bounding boxes
        @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
        The vector of the candidate holes's outline points
        @param[in] inflationSize [cosnt int&] grow the rectangle by
        inflationSize as to acquire more points to check for plane existence.
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree which with the
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesLuminosityDiff(
        const cv::Mat& inImage,
        const std::vector<cv::KeyPoint>& inKeyPoints,
        const std::vector<std::vector<cv::Point2f> >& inRectangles,
        const std::vector<std::vector<cv::Point> >& inOutlines,
        const int& inflationSize,
        std::vector<float>* probabilitiesVector);


      /**
        @brief Given a set of keypoints and their respective outline and
        bounding box points, and a model histogram, this filter looks for near
        equation between the histograms of the points that consist the bounding
        box and the model histogram, and for major difference between the
        histograms of the bounding box and the points inside the outline of the
        blob.
        @param[in] inImage [const cv::Mat&] The input RGB image
        @param[in] inHistogram [const cv::MatND&]
        The model histogram's H and S component
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The vector
        of the candidate holes's keypoints
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
        The vector of the candidate holes's bounding boxes
        @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
        The vector of the candidate holes's outline points
        @param[in] inflationSize [cosnt int&] grow the rectangle by
        inflationSize as to acquire more points to check for plane existence.
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree which with the
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesTextureDiff(
        const cv::Mat& inImage,
        const cv::MatND& inHistogram,
        const std::vector<cv::KeyPoint>& inKeyPoints,
        const std::vector<std::vector<cv::Point2f> >& inRectangles,
        const std::vector<std::vector<cv::Point> >& inOutlines,
        const int& inflationSize,
        std::vector<float>* probabilitiesVector);

      /**
        @brief Given a set of keypoints and their respective outline and
        bounding box points, and a model histogram, this filter creates the
        back project of the @param inImage based on @param inHistogram and
        exports a vector of probabilities, that is a vector of how probable it
        is for a candidate hole's bounding box points to have a high probability
        in the back project image, and for the points inside the candidate
        hole's outline to have a low probability in the back project image
        @param[in] inImage [const cv::Mat&] The input RGB image
        @param[in] inHistogram [const cv::MatND&]
        The model histogram's H and S component
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The vector
        of the candidate holes's keypoints
        @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&]
        The vector of the candidate holes's bounding boxes
        @param[in] inOutlines [const std::vector<std::vector<cv::Point> >&]
        The vector of the candidate holes's outline points
        @param[in] inflationSize [cosnt int&] grow the rectangle by
        inflationSize as to acquire more points to check for plane existence.
        @param[out] probabilitiesVector [std::vector<float>*] A vector
        of probabilities hinting to the certainty degree which with the
        candidate hole is associated. While the returned set may be reduced in
        size, the size of this vector is the same throughout and equal to the
        number of keypoints found and published by the rgb node
        @return std::set<unsigned int> The indices of valid (by this filter)
        blobs
       **/
      static std::set<unsigned int> checkHolesTextureBackProject(
        const cv::Mat& inImage,
        const cv::MatND& inHistogram,
        const std::vector<cv::KeyPoint>& inKeyPoints,
        const std::vector<std::vector<cv::Point2f> >& inRectangles,
        const std::vector<std::vector<cv::Point> >& inOutlines,
        const int& inflationSize,
        std::vector<float>* probabilitiesVector);
  };
}
#endif

