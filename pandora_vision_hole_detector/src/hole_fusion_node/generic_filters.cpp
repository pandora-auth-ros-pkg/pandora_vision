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

#include "hole_fusion_node/generic_filters.h"

namespace pandora_vision
{
  /**
    @brief Assimilates fragmented holes into existing whole ones
    from either source (RGB or Depth). It checks whether a set of keypoints
    reside in another source's set of bounding boxes with greater area.
    If so, the latter keypoint etc are kept and the former one is deleted.
    @param[in][out] depthHolesConveyor [HoleFilters::HolesConveyor*]
    The candidate holes conveyor originated from the depth node
    @param[in][out] rgbHolesConveyor [HoleFilters::HolesConveyor*]
    The candidate holes conveyor originated from the rgb node
    @return void
   **/
  void GenericFilters::mergeHoles(
    HoleFilters::HolesConveyor* depthHolesConveyor,
    HoleFilters::HolesConveyor* rgbHolesConveyor)
  {
    //!< Merging holes has a meaning only if both nodes have published
    //!< candidate holes
    if (depthHolesConveyor->keyPoints.size() > 0 &&
      rgbHolesConveyor->keyPoints.size() > 0)
    {
      //!< Validate depth's holes against the rgb ones
      for (int r = 0; r < rgbHolesConveyor->keyPoints.size(); r++)
      {
        for (int d = depthHolesConveyor->keyPoints.size() - 1; d >= 0; d--)
        {
          //!< If a depth keypoint resides in a rgb bounding box
          //!< delete the keypoint along with its associated conveyor entries
          if (cv::pointPolygonTest(rgbHolesConveyor->rectangles[r],
              depthHolesConveyor->keyPoints[d].pt, false) > 0)
          {
            //!< Calculate the rgb's bounding box area
            float rgbRectangleWidthX = rgbHolesConveyor->rectangles[r][0].x
              - rgbHolesConveyor->rectangles[r][1].x;
            float rgbRectangleWidthY = rgbHolesConveyor->rectangles[r][0].y
              - rgbHolesConveyor->rectangles[r][1].y;
            float rgbRectangleHeightX = rgbHolesConveyor->rectangles[r][1].x
              - rgbHolesConveyor->rectangles[r][2].x;
            float rgbRectangleHeightY = rgbHolesConveyor->rectangles[r][1].y
              - rgbHolesConveyor->rectangles[r][2].y;

            float rgbBoxArea = sqrt(pow(rgbRectangleWidthX, 2)
              + pow(rgbRectangleWidthY, 2)) * sqrt(pow(rgbRectangleHeightX, 2)
              + pow(rgbRectangleHeightY, 2));

            //!< Calculate the depth's bounding box area
            float depthRectangleWidthX = depthHolesConveyor->rectangles[d][0].x
              - depthHolesConveyor->rectangles[d][1].x;
            float depthRectangleWidthY = depthHolesConveyor->rectangles[d][0].y
              - depthHolesConveyor->rectangles[d][1].y;
            float depthRectangleHeightX = depthHolesConveyor->rectangles[d][1].x
              - depthHolesConveyor->rectangles[d][2].x;
            float depthRectangleHeightY = depthHolesConveyor->rectangles[d][1].y
              - depthHolesConveyor->rectangles[d][2].y;

            float depthBoxArea = sqrt(pow(depthRectangleWidthX, 2) +
              pow(depthRectangleWidthY, 2)) * sqrt(pow(depthRectangleHeightX, 2)
              + pow(depthRectangleHeightY, 2));

            if (depthBoxArea < rgbBoxArea)
            {
              depthHolesConveyor->keyPoints.erase(
                depthHolesConveyor->keyPoints.begin() + d);
              depthHolesConveyor->outlines.erase(
                depthHolesConveyor->outlines.begin() + d);
              depthHolesConveyor->rectangles.erase(
                depthHolesConveyor->rectangles.begin() + d);
            }
          }
        }
      }


      //!< Validate RGB's holes against the depth ones
      for (int d = 0; d < depthHolesConveyor->keyPoints.size(); d++)
      {
        for (int r = rgbHolesConveyor->keyPoints.size() - 1; r >= 0; r--)
        {
          //!< If a rgb keypoint resides in a depth's bounding box
          //!< delete the keypoint along with its associated conveyor entries
          if (cv::pointPolygonTest(depthHolesConveyor->rectangles[d],
              rgbHolesConveyor->keyPoints[r].pt, false) > 0)
          {
            //!< Calculate the depth's bounding box area
            float depthRectangleWidthX = depthHolesConveyor->rectangles[d][0].x
              - depthHolesConveyor->rectangles[d][1].x;
            float depthRectangleWidthY = depthHolesConveyor->rectangles[d][0].y
              - depthHolesConveyor->rectangles[d][1].y;
            float depthRectangleHeightX = depthHolesConveyor->rectangles[d][1].x
              - depthHolesConveyor->rectangles[d][2].x;
            float depthRectangleHeightY = depthHolesConveyor->rectangles[d][1].y
              - depthHolesConveyor->rectangles[d][2].y;

            float depthBoxArea = sqrt(pow(depthRectangleWidthX, 2) +
              pow(depthRectangleWidthY, 2)) * sqrt(pow(depthRectangleHeightX, 2)
              + pow(depthRectangleHeightY, 2));

            //!< Calculate the rgb's bounding box area
            float rgbRectangleWidthX = rgbHolesConveyor->rectangles[r][0].x
              - rgbHolesConveyor->rectangles[r][1].x;
            float rgbRectangleWidthY = rgbHolesConveyor->rectangles[r][0].y
              - rgbHolesConveyor->rectangles[r][1].y;
            float rgbRectangleHeightX = rgbHolesConveyor->rectangles[r][1].x
              - rgbHolesConveyor->rectangles[r][2].x;
            float rgbRectangleHeightY = rgbHolesConveyor->rectangles[r][1].y
              - rgbHolesConveyor->rectangles[r][2].y;

            float rgbBoxArea = sqrt(pow(rgbRectangleWidthX, 2)
              + pow(rgbRectangleWidthY, 2)) * sqrt(pow(rgbRectangleHeightX, 2)
              + pow(rgbRectangleHeightY, 2));


            if (rgbBoxArea < depthBoxArea)
            {
              rgbHolesConveyor->keyPoints.erase(
                rgbHolesConveyor->keyPoints.begin() + r);
              rgbHolesConveyor->outlines.erase(
                rgbHolesConveyor->outlines.begin() + r);
              rgbHolesConveyor->rectangles.erase(
                rgbHolesConveyor->rectangles.begin() + r);
            }
          }
        }
      }
    }
  }
}
