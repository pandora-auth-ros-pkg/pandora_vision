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
 * Authors: Alexandros Philotheou, Manos Tsardoulias
 *********************************************************************/

#include "utils/hole_filters.h"

namespace pandora_vision
{
  /**
    @brief Given a set of keypoints and an edges image, this function
    returns the valid keypoints and for each one, its respective, least
    area, rotated bounding box and the points of its outline.
    @param[in,out] keyPoints [std::vector<cv::KeyPoint>*]
    The original keypoints found.
    @param[in] denoisedDepthImageEdges [cv::Mat*] The original denoised
    depth edges image
    @param[in] detectionMethod [const int&] The method by which the outline of a
    blob is obtained. 0 means by means of brushfire, 1 by means of raycasting
    @param[in,out] conveyor [HolesConveyor*] A struct that contains the final
    valid holes
    @return void
   **/
  void HoleFilters::validateBlobs(
    std::vector<cv::KeyPoint>* keyPoints,
    cv::Mat* denoisedDepthImageEdges,
    const int& detectionMethod,
    HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("validateBlobs", "findHoles");
    #endif

    switch(detectionMethod)
    {
      case 0:
        {
          std::vector<std::vector<cv::Point2f> > blobsOutlineVector;
          std::vector<float> blobsArea;

          BlobDetection::brushfireKeypoint(*keyPoints,
            denoisedDepthImageEdges,
            &blobsOutlineVector,
            &blobsArea);

          //!< For each outline found, find the rotated rectangle
          //!< with the least area that encloses it.
          cv::Mat inputDenoisedDepthImageEdges;
          denoisedDepthImageEdges->copyTo(inputDenoisedDepthImageEdges);

          cv::Mat rectanglesImage;
          std::vector< std::vector<cv::Point2f> > rectangles;

          //!< Given the outline of the blob, find the least area
          //!< rotated bounding box that encloses it
          BoundingBoxDetection::findRotatedBoundingBoxesFromOutline(
            inputDenoisedDepthImageEdges,
            blobsOutlineVector,
            blobsArea,
            &rectanglesImage,
            &rectangles);

          //!< Correlate each keypoint with each rectangle found.
          //!< Keep in mind that for a blob to be a potential hole, its area
          //!< must be greater than Parameters::bounding_box_min_area_threshold
          validateKeypointsToRectangles(
            *keyPoints,
            rectangles,
            blobsArea,
            blobsOutlineVector,
            conveyor);

          break;
        }
      case 1:
        {
          std::vector<std::vector<cv::Point2f> > blobsOutlineVector;
          std::vector<float> blobsArea;

          BlobDetection::raycastKeypoint(keyPoints,
            denoisedDepthImageEdges,
            Parameters::raycast_keypoint_partitions,
            &blobsOutlineVector,
            &blobsArea);

          //!< For each outline found, find the rotated rectangle
          //!< with the least area that encloses it.
          cv::Mat inputDenoisedDepthImageEdges;
          denoisedDepthImageEdges->copyTo(inputDenoisedDepthImageEdges);

          cv::Mat rectanglesImage;
          std::vector< std::vector<cv::Point2f> > rectangles;

          //!< Given the outline of the blob, find the least area
          //!< rotated bounding box that encloses it
          BoundingBoxDetection::findRotatedBoundingBoxesFromOutline(
            inputDenoisedDepthImageEdges,
            blobsOutlineVector,
            blobsArea,
            &rectanglesImage,
            &rectangles);

          //!< Correlate each keypoint with each rectangle found.
          //!< Keep in mind that for a blob to be a potential hole, its area
          //!< must be greater than Parameters::bounding_box_min_area_threshold
          validateKeypointsToRectangles(
            *keyPoints,
            rectangles,
            blobsArea,
            blobsOutlineVector,
            conveyor);

          break;
        }
    }
    /* The end product here is a struct (conveyor) of keypoints,
     * a set of rectangles that enclose them  and the outline of
     * each blob found.
     */
    #ifdef DEBUG_TIME
    Timer::tick("validateBlobs");
    #endif
  }



  /**
    @brief This functions takes as input arguments a keypoints vector of
    size N, a rectangles vector of size M (the rectangle is represented
    by its 4 vertices so that the input can be either a Rectangle or a
    Rotated Rectangle) and a vector with the area of each rectangle
    with purpose to identify the least area rectangle in which a
    keypoint resides. It outputs a vector of keypoints (each keypoint must
    reside in at least one rectangle) and a vector of rectangles (again
    represented by its 4 vertices). There is a one-to-one association
    between the keypoints and the rectangles.
    @param[in] inKeyPoints [const std::vector<cv::KeyPoint>&] The key points
    @param[in] inRectangles [const std::vector<std::vector<cv::Point2f> >&] The
    rectangles found
    @param[in] inRectanglesArea [const std::vector<float>&]
    The area of each rectangle
    @param[in] inContours [const std::vector<std::vector<cv::Point2f> >&]
    The outline of each blob found
    @param[out] conveyor [HolesConveyor*] The container of vector of blobs'
    keypoints, outlines and areas
    @return void
   **/
  void HoleFilters::validateKeypointsToRectangles (
    const std::vector<cv::KeyPoint>& inKeyPoints,
    const std::vector<std::vector<cv::Point2f> >& inRectangles,
    const std::vector<float>& inRectanglesArea,
    const std::vector<std::vector<cv::Point2f> >& inContours,
    HolesConveyor* conveyor)
  {
    #ifdef DEBUG_TIME
    Timer::start("validateKeypointsToRectangles", "validateBlobs");
    #endif
    for (unsigned int keypointId = 0;
      keypointId < inKeyPoints.size(); keypointId++)
    {
      std::vector<int> keypointResidesInRectIds;

      //!< Test to see where (in which rectangle(s)) the keypoint resides.
      for (unsigned int rectId = 0; rectId < inRectangles.size(); rectId++)
      {
        if (cv::pointPolygonTest(
            inRectangles[rectId], inKeyPoints[keypointId].pt, false) > 0)
        {
          keypointResidesInRectIds.push_back(rectId);
        }
      }

      /*
       * If the keypoint resides in exactly one rectangle.
       */
      if (keypointResidesInRectIds.size() == 1)
      {
        conveyor->keyPoints.push_back(inKeyPoints[keypointId]);
        conveyor->rectangles.push_back(
          inRectangles[keypointResidesInRectIds[0]]);
        conveyor->outlines.push_back(inContours[keypointId]);
      }

      /*
       * If the keypoint resides in multiple rectangles choose the one
       * with the least area.
       */
      if (keypointResidesInRectIds.size() > 1)
      {
        float minRectArea = 1000000.0;
        int minAreaRectId;
        for (unsigned int i = 0; i < keypointResidesInRectIds.size(); i++)
        {
          if (inRectanglesArea[keypointResidesInRectIds[i]] < minRectArea)
          {
            minRectArea = inRectanglesArea[keypointResidesInRectIds[i]];
            minAreaRectId = keypointResidesInRectIds[i];
          }
        }

        conveyor->keyPoints.push_back(inKeyPoints[keypointId]);
        conveyor->rectangles.push_back(inRectangles[minAreaRectId]);
        conveyor->outlines.push_back(inContours[keypointId]);
      }
      /*
       * If the keypoint has no rectangle attached to it
       * (if, for example the blob's area was smaller
       * than blob_min_area_threshold), do not insert the keypoint in the
       * valid keypoints vector etc.
       */
    }
    #ifdef DEBUG_TIME
    Timer::tick("validateKeypointsToRectangles");
    #endif
  }

} // namespace pandora_vision
