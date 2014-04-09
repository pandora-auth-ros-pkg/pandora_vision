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

#ifndef KINECT_BLOB_DETECTION
#define KINECT_BLOB_DETECTION

#include "depth_node/morphological_operators.h"

/**
  @namespace vision
  @brief The main namespace for PANDORA vision
 **/
namespace pandora_vision
{
  /**
    @class BlobDetection
    @brief Provides methods for blob detection
   **/
  class BlobDetection
  {
    public:

      /**
        @brief Implements the brushfire algorithm for all blob keypoints in
        order to find a blob limits
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>] The keypoints
        @param[in] edgesImage [const cv::Mat] The input image
        @param[out] blobsOutlineVector [std::vector<std::vector<cv::Point> >&]
        The output vector containing the blobs' outline
        @param[out] blobsArea [std::vector<float>&] The area of each blob
        @return void
       **/
      static void brushfireKeypoint (
          const std::vector<cv::KeyPoint> inKeyPoints,
          cv::Mat edgesImage,
          std::vector<std::vector<cv::Point> >& blobsOutlineVector,
          std::vector<float>& blobsArea);

      /**
        @brief Implements the brushfire algorithm. Its specific purpose is
        to find the points between a blob's outline and its bounding box
        (not necessarily one of least area).
        @param[in] inPoint [const cv::Point] The input point
        @param[in] inImage [cv::Mat] The input image
        @param[out] pointsCovered [std::set<unsigned int>&] The points between
        two areas of non-zero value pixels.
        @return void
       **/
      static void brushfirePoint(const cv::Point inPoint,
          cv::Mat inImage,
          std::set<unsigned int>& pointsCovered);


      /**
        @brief Detects blobs in an image
        @param[in] inImage [const cv::Mat] The input image
        @param[out] keyPointsOut [std::vector<cv::KeyPoint>&] The ouput
        @return void
       **/
      static void detectBlobs(const cv::Mat inImage,
          std::vector<cv::KeyPoint>& keyPointsOut);

      /**
        @brief Implements a raycast algorithm for all blob keypoints in order
        to find the blob limits
        @param[in] inKeyPoints [const std::vector<cv::KeyPoint>] The keypoints
        @param[in] edgesImage [cv::Mat] The input image
        @param[in] partitions [const int] The number of directions towards which
        the outline of the blob will be sought, or the number of partitions in
        which the blob will be divided by the rays. Same deal.
        @param[out] blobsOutlineVector [std::vector<std::vector<cv::Point> >&]
        The output vector containing the blobs' (rough approximate) outline
        @param[out] blobsArea [std::vector<float>&] The area of each blob
        @return void
       **/
      static void raycastKeypoint(
          const std::vector<cv::KeyPoint> inKeyPoints,
          cv::Mat edgesImage,
          const int partitions,
          std::vector<std::vector<cv::Point> >& blobsOutlineVector,
          std::vector<float>& blobsArea);

      /**
        @brief Takes as input a binary image and stores in @outlines the
        outlines of closed curves. (Assumes that the input image comprises
        entirely of closed curves.)
        @param[in] inImage [cv::Mat&] The input binary image
        @param[out] outlines [std::vector<std::vector<cv::Point> >&] The points
        that each detected closed curve consists of
        @return void
       **/
      static void getClosedCurves(cv::Mat& inImage,
          std::vector<std::vector<cv::Point> >& outlines);

  };

}

#endif
