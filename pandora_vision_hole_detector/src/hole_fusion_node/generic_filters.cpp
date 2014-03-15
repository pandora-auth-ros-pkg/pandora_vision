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
      assimilate(*rgbHolesConveyor, depthHolesConveyor);

      assimilate(*depthHolesConveyor, rgbHolesConveyor);
    }

  }



  /**
    @brief Assimilates the fragmented holes of @param assimilable into the
    existing whole ones of @param assimilator. It checks whether the set of
    assimilable's keypoints reside in the assimilator's set of bounding
    boxes with greater area. If so, the latter keypoint etc are kept and
    the former one is deleted.
    @param[in][out] assimilator [const HoleFilters::HolesConveyor&]
    The candidate holes conveyor that will potentially assimilate the
    assimimable's holes
    @param[in][out] assimilable [HoleFilters::HolesConveyor*]
    The candidate holes conveyor whose holes will potentially by assimilated
    by the assimilator
    @return void
   **/
  void GenericFilters::assimilate(const HoleFilters::HolesConveyor& assimilator,
    HoleFilters::HolesConveyor* assimilable)
  {
    //!< Validate depth's holes against the rgb ones
    for (int i = 0; i < assimilator.keyPoints.size(); i++)
    {
      for (int j = assimilable->keyPoints.size() - 1; j >= 0; j--)
      {
        //!< If an assimilable keypoint resides in an assimilator bounding box
        //!< delete the keypoint along with its associated conveyor entries
        if (cv::pointPolygonTest(assimilator.rectangles[i],
            assimilable->keyPoints[j].pt, false) > 0)
        {
          //!< Calculate the assimilator's bounding box area
          float assimilatorRectangleWidthX = assimilator.rectangles[i][0].x
            - assimilator.rectangles[i][1].x;
          float assimilatorRectangleWidthY = assimilator.rectangles[i][0].y
            - assimilator.rectangles[i][1].y;
          float assimilatorRectangleHeightX = assimilator.rectangles[i][1].x
            - assimilator.rectangles[i][2].x;
          float assimilatorRectangleHeightY = assimilator.rectangles[i][1].y
            - assimilator.rectangles[i][2].y;

          float assimilatorBoxArea = sqrt(pow(assimilatorRectangleWidthX, 2)
            + pow(assimilatorRectangleWidthY, 2)) *
            sqrt(pow(assimilatorRectangleHeightX, 2)
            + pow(assimilatorRectangleHeightY, 2));

          //!< Calculate the assimilable's bounding box area
          float assimilableRectangleWidthX = assimilable->rectangles[j][0].x
            - assimilable->rectangles[j][1].x;
          float assimilableRectangleWidthY = assimilable->rectangles[j][0].y
            - assimilable->rectangles[j][1].y;
          float assimilableRectangleHeightX = assimilable->rectangles[j][1].x
            - assimilable->rectangles[j][2].x;
          float assimilableRectangleHeightY = assimilable->rectangles[j][1].y
            - assimilable->rectangles[j][2].y;

          float assimilableBoxArea = sqrt(pow(assimilableRectangleWidthX, 2) +
            pow(assimilableRectangleWidthY, 2)) *
            sqrt(pow(assimilableRectangleHeightX, 2)
            + pow(assimilableRectangleHeightY, 2));

          if (assimilableBoxArea < assimilatorBoxArea)
          {
            assimilable->keyPoints.erase(assimilable->keyPoints.begin() + j);
            assimilable->outlines.erase(assimilable->outlines.begin() + j);
            assimilable->rectangles.erase(assimilable->rectangles.begin() + j);
          }
        }
      }
    }
  }

} // namespace pandora_vision
