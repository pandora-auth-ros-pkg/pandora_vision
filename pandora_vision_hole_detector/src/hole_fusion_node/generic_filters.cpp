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
    from either source (RGB or Depth).
    @param[in][out] depthHolesConveyor [HolesConveyor*]
    The candidate holes conveyor originated from the depth node
    @param[in][out] rgbHolesConveyor [HolesConveyor*]
    The candidate holes conveyor originated from the rgb node
    @return void
   **/
  void GenericFilters::assimilateBilaterally(
    HolesConveyor* depthHolesConveyor,
    HolesConveyor* rgbHolesConveyor)
  {
    //!< Assimilating holes has a meaning only if both nodes have published
    //!< candidate holes
    if (depthHolesConveyor->keyPoints.size() > 0 &&
      rgbHolesConveyor->keyPoints.size() > 0)
    {
      assimilateUnilaterally(*rgbHolesConveyor, depthHolesConveyor);

      assimilateUnilaterally(*depthHolesConveyor, rgbHolesConveyor);
    }

  }



  /**
    @brief Assimilates the fragmented holes of @param assimilable into the
    existing whole ones of @param assimilator. It checks whether each
    entry of the set of assimilable's outline points reside inside
    the assimilator's set of outlines. If so, the latter keypoint
    etc are kept unchanged and the former one is deleted.
    @param[in][out] assimilator [const HolesConveyor&]
    The candidate holes conveyor that will potentially assimilate the
    assimimable's holes
    @param[in][out] assimilable [HolesConveyor*]
    The candidate holes conveyor whose holes will potentially by assimilated
    by the assimilator
    @return void
   **/
  void GenericFilters::assimilateUnilaterally(
    const HolesConveyor& assimilator,
    HolesConveyor* assimilable)
  {
    //!< Validate assimilable's holes against the assimilator's ones
    for (int i = 0; i < assimilator.keyPoints.size(); i++)
    {
      for (int j = assimilable->keyPoints.size() - 1; j >= 0; j--)
      {
        //!< If all of assimilable's outline points reside inside the
        //!< assimilator's outline delete the keypoint along with its
        //!< associated conveyor entries
        if (isCapableOfAssimilating(i, assimilator, j, *assimilable))
        {
          assimilateOnce(j, assimilable);
        }
      }
    }
  }



  /**
    @brief Indicates whether a hole assigned the role of the assimilator
    is capable of assimilating another hole assigned the role of
    the assimilable. It checks whether the assimilable's outline
    points reside entirely inside the assimilator's outline.
    @param[in] assimilatorId [const int&] The index of the specific hole
    inside the assimilator HolesConveyor
    @paramp[in] assimilator [const HolesConveyor&] The HolesConveyor struct
    that acts as the assimilator
    @param[in] assimilableId [const int&] The index of the specific hole
    inside the assimilable HolesConveyor
    @paramp[in] assimilable [const HolesConveyor&] The HolesConveyor struct
    that acts as the assimilable
    @return [bool] True if all of the outline points of the assimilable
    hole are inside the outline of the assimilator
   **/
  bool GenericFilters::isCapableOfAssimilating(
    const int& assimilatorId,
    const HolesConveyor& assimilator,
    const int& assimilableId,
    const HolesConveyor& assimilable)
  {
    //!< Are all the outline points of assimilable inside the
    //!< assimilator's outline?
    bool allAssimilableOutlinePointsInAssimilator = true;
    for (int av = 0; av < assimilable.outlines[assimilableId].size(); av++)
    {
      if (cv::pointPolygonTest(assimilator.outlines[assimilatorId],
          assimilable.outlines[assimilableId][av], false) < 0)
      {
        allAssimilableOutlinePointsInAssimilator = false;
        break;
      }
    }
    return allAssimilableOutlinePointsInAssimilator;
  }



  /**
    @brief Intended to use after the check of the
    isCapableOfAssimilating function, this function carries the burden
    of having to delete a hole entry from its HolesConveyor struct,
    thus being its executor
    @param[in] keyPointId [const int&] The identifier of the hole inside
    the HolesConveyor struct
    @param[in][out] assimilable [HolesConveyor*] The holes conveyor
    from which the keypoint, outline and bounding rectangle entries
    will be deleted
    @return void
   **/
  void GenericFilters::assimilateOnce(const int& keyPointId,
    HolesConveyor* assimilable)
  {
    //!< Delete the keypoint from the conveyor
    assimilable->keyPoints.erase(
      assimilable->keyPoints.begin() + keyPointId);

    //!< Delete each outline point from its respective vector
    assimilable->outlines[keyPointId].erase(
      assimilable->outlines[keyPointId].begin(),
      assimilable->outlines[keyPointId].end());

    //!< Delete the outline vector entry alltogether
    assimilable->outlines.erase(assimilable->outlines.begin() + keyPointId);

    //!< Delete each rectangle point from its respective vector
    assimilable->rectangles[keyPointId].erase(
      assimilable->rectangles[keyPointId].begin(),
      assimilable->rectangles[keyPointId].end());

    //!< Delete the rectangle vector entry alltogether
    assimilable->rectangles.erase(assimilable->rectangles.begin() + keyPointId);
  }



  /**
    @brief Connects nearby holes. Holes' outlines do not intersect.
    @param[in][out] assimilator [HolesConveyor*] The holes conveyor
    that will act as the assimilator of holes
    @param[in][out] assimilable [HolesConveyor*] The holes conveyor
    that will act as the assimilable
    @param [in] pointCloud [const PointCloudXYZPtr&] The point cloud
    needed in order to specify which holes are going to be connected
    by criterion of distance
    @return void
   **/
  void GenericFilters::connectUnilaterally(HolesConveyor* assimilator,
    HolesConveyor* assimilable, const PointCloudXYZPtr& pointCloudXYZ)
  {

    //!< Validate the assimilable's holes against the assimilator's ones
    for (int i = 0; i < assimilator->keyPoints.size(); i++)
    {
      for (int j = assimilable->keyPoints.size() - 1; j >= 0; j--)
      {
        //!< Are all the assimilable's outline points outside
        //!< the assimilator's bounding box?
        int numAssimilableOutlinePointsInAssimilator = 0;

        //!< The real min distance (in meters) between two points of the
        //!< assimilator's and assimilable's outlines
        double minOutlinesDistance = 10000.0;

        for (int av = 0; av < assimilable->outlines[j].size(); av++)
        {
          if (cv::pointPolygonTest(assimilator->outlines[i],
              assimilable->outlines[j][av], false) > 0)
          {
            numAssimilableOutlinePointsInAssimilator++;
          }

          //!< The assimilable's current outline point x,y,z coordinates
          //!< measured by the depth sensor
          float assimilableOutlinePointX = pointCloudXYZ->points[
            static_cast<int>(assimilable->outlines[j][av].x)
            + pointCloudXYZ->width *
            static_cast<int>(assimilable->outlines[j][av].y)].x;

          float assimilableOutlinePointY = pointCloudXYZ->points[
            static_cast<int>(assimilable->outlines[j][av].x)
            + pointCloudXYZ->width *
            static_cast<int>(assimilable->outlines[j][av].y)].y;

          float assimilableOutlinePointZ = pointCloudXYZ->points[
            static_cast<int>(assimilable->outlines[j][av].x)
            + pointCloudXYZ->width *
            static_cast<int>(assimilable->outlines[j][av].y)].z;


          //!< The assimilator's current outline point x,y,z coordinates
          //!< measured by the depth sensor
          float assimilatorOutlinePointX = pointCloudXYZ->points[
            static_cast<int>(assimilator->outlines[j][av].x)
            + pointCloudXYZ->width *
            static_cast<int>(assimilator->outlines[j][av].y)].x;

          float assimilatorOutlinePointY = pointCloudXYZ->points[
            static_cast<int>(assimilator->outlines[j][av].x)
            + pointCloudXYZ->width *
            static_cast<int>(assimilator->outlines[j][av].y)].y;

          float assimilatorOutlinePointZ = pointCloudXYZ->points[
            static_cast<int>(assimilator->outlines[j][av].x)
            + pointCloudXYZ->width *
            static_cast<int>(assimilator->outlines[j][av].y)].z;


          //!< The current outline points distance
          float outlinePointsDistance = sqrt(
            pow(assimilableOutlinePointX - assimilatorOutlinePointX, 2) +
            pow(assimilableOutlinePointY - assimilatorOutlinePointY, 2) +
            pow(assimilableOutlinePointZ - assimilatorOutlinePointZ, 2));

          if (outlinePointsDistance < minOutlinesDistance)
          {
            minOutlinesDistance = outlinePointsDistance;
          }
        }

        //!< If not all of assimilable's outline points
        //!< are outside the assimilator's outline,
        //!< or the minimum distance between the assimilator's and assimilable's
        //!< outlines is greater than a distance thrshold,
        //!< this assimilable is not a candidate to be connected with
        //!< the assimilator
        if (numAssimilableOutlinePointsInAssimilator != 0 ||
          minOutlinesDistance > Parameters::connect_holes_min_distance)
        {
          continue;
        }


        //!< Calculate the assimilator's bounding box area
        float assimilatorRectangleWidthX = assimilator->rectangles[i][0].x
          - assimilator->rectangles[i][1].x;
        float assimilatorRectangleWidthY = assimilator->rectangles[i][0].y
          - assimilator->rectangles[i][1].y;
        float assimilatorRectangleHeightX = assimilator->rectangles[i][1].x
          - assimilator->rectangles[i][2].x;
        float assimilatorRectangleHeightY = assimilator->rectangles[i][1].y
          - assimilator->rectangles[i][2].y;

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


        //!< If the assimilable's area is smaller than the assimilator's,
        //!< connect the assimilator with the assimilable,
        //!< replacing the assimilator and delete the assimilable
        if (assimilableBoxArea < assimilatorBoxArea)
        {
          //!< Viewing the two outlines as sets,
          //!< the final outline should not have the intersection
          //!< of the two sets.
          std::vector<cv::Point> assimilatorOutline;
          std::vector<cv::Point> assimilableOutline;


          //!< The assimilator's outline will be the sum of the outline points
          //!< that are not located inside each other
          assimilator->outlines[i].insert(assimilator->outlines[i].end(),
            assimilable->outlines[j].begin(), assimilable->outlines[j].end());


          //!< The assimilable's new least area rotated bounding box will be the
          //!< one that encloses the new (merged) outline points
          cv::RotatedRect substituteRotatedRectangle =
            minAreaRect(assimilator->outlines[i]);

          //!< Obtain the four vertices of the new rotated rectangle
          cv::Point2f substituteVerticesArray[4];
          substituteRotatedRectangle.points(substituteVerticesArray);

          //!< Same as substituteVerticesArray array, but vector
          std::vector<cv::Point2f> substituteVerticesVector;

          //!< Store the four vertices to the substituteVerticesVector
          for(int v = 0; v < 4; v++)
          {
            substituteVerticesVector.push_back(substituteVerticesArray[v]);
          }

          //!< Replace the assimilator's vertices with the new vertices
          assimilator->rectangles[i] = substituteVerticesVector;


          //!< The overall candidate hole's keypoint
          assimilator->keyPoints[i].pt.x = (assimilator->keyPoints[i].pt.x +
            assimilable->keyPoints[j].pt.x) / 2;
          assimilator->keyPoints[i].pt.y = (assimilator->keyPoints[i].pt.y +
            assimilable->keyPoints[j].pt.y) / 2;

          //!< The assimilable has now been merged with the assimilator,
          //!< so delete it. So long, assimilable
          assimilable->keyPoints.erase(assimilable->keyPoints.begin() + j);
          assimilable->outlines[j].erase(assimilable->outlines[j].begin(),
            assimilable->outlines[j].end());
          assimilable->outlines.erase(assimilable->outlines.begin() + j);
          assimilable->rectangles[j].erase(assimilable->rectangles[j].begin(),
            assimilable->rectangles[j].end());
          assimilable->rectangles.erase(assimilable->rectangles.begin() + j);
        }
      }
    }
  }



  /**
    @brief Given the RGB and Depth HolesConveyor* structs,
    the purpose of this function is to identify blobs that are overlapping
    each other but none of them is entirely inside the other, and merge
    them in one candidate hole: the one whose bounding rectangle has
    the greater area.
    @param[in][out] depthHolesConveyor [HolesConveyor*]
    The candidate holes conveyor originated from the depth node
    @param[in][out] rgbHolesConveyor [HolesConveyor*]
    The candidate holes conveyor originated from the rgb node
    @return void
   **/
  void GenericFilters::amalgamateBilaterally(
    HolesConveyor* depthHolesConveyor,
    HolesConveyor* rgbHolesConveyor)
  {
    //!< Merging holes has a meaning only if both nodes have published
    //!< candidate holes
    if (depthHolesConveyor->keyPoints.size() > 0 &&
      rgbHolesConveyor->keyPoints.size() > 0)
    {
      amalgamateUnilaterally(rgbHolesConveyor, depthHolesConveyor);

      amalgamateUnilaterally(depthHolesConveyor, rgbHolesConveyor);
    }
  }



  /**
    @brief Given two HolesConveyor* structs, one with the
    potential of amalgamating the other (amalgamator) and the other with the
    potential of being amalgamated by the other (amalgamatable), the purpose
    of this function is to identify blobs that are overlapping each other
    but none of them is entirely inside the other, while the amalgamator's
    bounding box is greater than that of the amalgamatable's.
    If this is true for a given amalgamator and amalgamatable,
    the amalgamator will grow rectangle-wise and outline-wise
    by the size of the amalgamator, while the amalgamator's
    new keypoint will be the mean of the two keypoints. The amalgamatable
    then is rendered useless and deleted.
    @param[in][out] amalgamator [HolesConveyor*]
    The holes conveyor whose candidate holes will potentially
    assimilate candidate holes of @param amalgamatable
    @param[in][out] amalgamatable [HolesConveyor*]
    The holes conveyor whose candidate holes will potentially
    will be assimilated by candidate holes of @paramamalgamator
    @return void
   **/
  void GenericFilters::amalgamateUnilaterally(
    HolesConveyor* amalgamator,
    HolesConveyor* amalgamatable)
  {
    //!< Validate the amalgamatable's holes against the amalgamator's ones
    for (int i = 0; i < amalgamator->keyPoints.size(); i++)
    {
      for (int j = amalgamatable->keyPoints.size() - 1; j >= 0; j--)
      {
        //!< If the amalgamatable's area is smaller than the amalgamator's,
        //!< merge the amalgamator with the amalgamatable into theamalgamator
        //!< and delete the amalgamatable
        if (isCapableOfAmalgamating(i, *amalgamator, j, *amalgamatable))
        {
          amalgamateOnce(i, amalgamator, j, amalgamatable);
        }
      }
    }
  }



  /**
    @brief Indicates whether a hole assigned the role of the amalgamator
    is capable of amalgamating another hole assigned the role of
    the amalgamatable. The amalgamator is capable of amalgamating the
    amalgamatable if and only if the amalgatamable's outline
    points intersect with the amalgamator's outline, and the bounding
    rectangle of the amalgamator is larger in area than the bounding
    rectangle of the amalgamatable
    @param[in] amalgamatorId [const int&] The index of the specific hole
    that acts as the amalgamator inside the amalgamator HolesConveyor
    @param[in] amalgamator [const HolesConveyor&] The HolesConveyor that
    acts as the amalgamator struct
    @param[in] amalgamatableId [const int&] The index of the specific hole
    that acts as the amalgamatable inside the amalgamatable HolesConveyor
    @param[in] amalgamatable [const HolesConveyor&] The HolesConveyor that
    acts as the amalgamatavle struct
    @return [bool] True if the amalgamator is capable of amalgamating
    the amalgamatable
   **/
  bool GenericFilters::isCapableOfAmalgamating(
    const int& amalgamatorId,
    const HolesConveyor& amalgamator,
    const int& amalgamatableId,
    const HolesConveyor& amalgamatable)
  {
    //!< Are all the assimilable's outline points inside
    //!< the assimilator's bounding box? If not all but some, continue
    int numAmalgamatableOutlinePointsInAmalgamator= 0;
    for (int av = 0; av < amalgamatable.outlines[amalgamatableId].size(); av++)
    {
      if (cv::pointPolygonTest(amalgamator.outlines[amalgamatorId],
          amalgamatable.outlines[amalgamatableId][av], false) > 0)
      {
        numAmalgamatableOutlinePointsInAmalgamator++;
      }
    }

    //!< If zero or all of amalgamatable's outline points
    //!< are inside the amalgamator's outline, this amalgamatable is
    //!< not a candidate to be amalgamated by the amalgamator
    if (numAmalgamatableOutlinePointsInAmalgamator == 0 ||
      numAmalgamatableOutlinePointsInAmalgamator ==
      amalgamatable.outlines[amalgamatableId].size())
    {
      return false;
    }


    //!< Calculate the amalgamator's bounding box area
    float amalgamatorRectangleWidthX =
      amalgamator.rectangles[amalgamatorId][0].x
      - amalgamator.rectangles[amalgamatorId][1].x;
    float amalgamatorRectangleWidthY =
      amalgamator.rectangles[amalgamatorId][0].y
      - amalgamator.rectangles[amalgamatorId][1].y;

    float amalgamatorRectangleHeightX =
      amalgamator.rectangles[amalgamatorId][1].x
      - amalgamator.rectangles[amalgamatorId][2].x;
    float amalgamatorRectangleHeightY =
      amalgamator.rectangles[amalgamatorId][1].y
      - amalgamator.rectangles[amalgamatorId][2].y;

    float amalgamatorBoxArea = sqrt(pow(amalgamatorRectangleWidthX, 2)
      + pow(amalgamatorRectangleWidthY, 2)) *
      sqrt(pow(amalgamatorRectangleHeightX, 2)
        + pow(amalgamatorRectangleHeightY, 2));


    //!< Calculate the amalgamatable's bounding box area
    float amalgamatableRectangleWidthX =
      amalgamatable.rectangles[amalgamatableId][0].x
      - amalgamatable.rectangles[amalgamatableId][1].x;
    float amalgamatableRectangleWidthY =
      amalgamatable.rectangles[amalgamatableId][0].y
      - amalgamatable.rectangles[amalgamatableId][1].y;

    float amalgamatableRectangleHeightX =
      amalgamatable.rectangles[amalgamatableId][1].x
      - amalgamatable.rectangles[amalgamatableId][2].x;
    float amalgamatableRectangleHeightY =
      amalgamatable.rectangles[amalgamatableId][1].y
      - amalgamatable.rectangles[amalgamatableId][2].y;

    float amalgamatableBoxArea = sqrt(pow(amalgamatableRectangleWidthX, 2) +
      pow(amalgamatableRectangleWidthY, 2)) *
      sqrt(pow(amalgamatableRectangleHeightX, 2)
        + pow(amalgamatableRectangleHeightY, 2));


    //!< If the amalgatamable's area is smaller than the assimilator's,
    //!< this amalgamator is capable of amalgamating the amalgamatable
    return (amalgamatableBoxArea < amalgamatorBoxArea);
  }



  /**
    @brief Intended to use after the check of the
    isCapableOfAmalgamating function, this function carries the burden
    of having to delete a hole entry from its HolesConveyor amalgamatable
    struct, thus being its executor,
    and modifying the HolesConveyor amalgamator struct entry so that it
    it has absorbed the amalgamatable hole in terms of keypoint location,
    outline unification and bounding rectangle inclusion of the
    amalgamatable's outline
    @param[in] amalgamatorId [const int&] The identifier of the hole inside
    the HolesConveyor amalgamator struct
    @param[in][out] amalgamator [HolesConveyor*] The holes conveyor
    whose keypoint, outline and bounding rectangle entries
    will be modified
    @param[in] amalgamatableId [const int&] The identifier of the hole inside
    the HolesConveyor amalgamatable struct
    @param[in][out] amalgamatable [HolesConveyor*] The holes conveyor
    whose keypoint, outline and bounding rectangle entries
    will be deleted
    @return void
   **/
  void GenericFilters::amalgamateOnce(const int& amalgamatorId,
    HolesConveyor* amalgamator,
    const int& amalgamatableId,
    HolesConveyor* amalgamatable)
  {
    //!< Viewing the two outlines as sets,
    //!< the final outline should not have the intersection
    //!< of the two sets.
    std::vector<cv::Point> amalgamatorOutline;
    std::vector<cv::Point> amalgamatableOutline;

    //!< Add all the amalgamatable's outline points that are not located
    //!< inside the amalgamator's outline, to the amalgamatableOutline
    //!< vector
    for (int o = 0; o < amalgamatable->outlines[amalgamatableId].size(); o++)
    {
      if (pointPolygonTest(amalgamator->outlines[amalgamatorId],
          amalgamatable->outlines[amalgamatableId][o], false) <= 0)
      {
        amalgamatableOutline.push_back(
          amalgamatable->outlines[amalgamatableId][o]);
      }
    }

    //!< Add all the amalgamator's outline points that are not located
    //!< inside the amalgamatable's outline, to the amalgamatorOutline
    //!< vector
    for (int o = 0; o < amalgamator->outlines[amalgamatorId].size(); o++)
    {
      if (pointPolygonTest(amalgamatable->outlines[amalgamatableId],
          amalgamator->outlines[amalgamatorId][o], false) <= 0)
      {
        amalgamatorOutline.push_back(amalgamator->outlines[amalgamatorId][o]);
      }
    }

    //!< The amalgamator's outline will be the sum of the outline points
    //!< that are not located inside each other
    amalgamator->outlines[amalgamatorId] = amalgamatorOutline;
    amalgamator->outlines[amalgamatorId].insert(
      amalgamator->outlines[amalgamatorId].end(),
      amalgamatableOutline.begin(), amalgamatableOutline.end());


    //!< The amalgamatable's new least area rotated bounding box will be the
    //!< one that encloses the new (merged) outline points
    cv::RotatedRect substituteRotatedRectangle =
      minAreaRect(amalgamator->outlines[amalgamatorId]);

    //!< Obtain the four vertices of the new rotated rectangle
    cv::Point2f substituteVerticesArray[4];
    substituteRotatedRectangle.points(substituteVerticesArray);

    //!< Same as substituteVerticesArray array, but vector
    std::vector<cv::Point2f> substituteVerticesVector;

    //!< Store the four vertices to the substituteVerticesVector
    for(int v = 0; v < 4; v++)
    {
      substituteVerticesVector.push_back(substituteVerticesArray[v]);
    }

    //!< Replace the amalgamator's vertices with the new vertices
    amalgamator->rectangles[amalgamatorId] = substituteVerticesVector;


    //!< The overall candidate hole's keypoint
    amalgamator->keyPoints[amalgamatorId].pt.x =
      (amalgamator->keyPoints[amalgamatorId].pt.x +
       amalgamatable->keyPoints[amalgamatableId].pt.x) / 2;

    amalgamator->keyPoints[amalgamatorId].pt.y =
      (amalgamator->keyPoints[amalgamatorId].pt.y +
       amalgamatable->keyPoints[amalgamatableId].pt.y) / 2;

    //!< The amalgamatable has now been merged with the amalgamator,
    //!< so delete it. So long, amalgamatable

    //!< Delete the keypoint from the conveyor
    amalgamatable->keyPoints.erase(
      amalgamatable->keyPoints.begin() + amalgamatableId);

    //!< Delete each outline point from its respective vector
    amalgamatable->outlines[amalgamatableId].erase(
      amalgamatable->outlines[amalgamatableId].begin(),
      amalgamatable->outlines[amalgamatableId].end());

    //!< Delete the outline vector entry alltogether
    amalgamatable->outlines.erase(
      amalgamatable->outlines.begin() + amalgamatableId);

    //!< Delete each rectangle point from its respective vector
    amalgamatable->rectangles[amalgamatableId].erase(
      amalgamatable->rectangles[amalgamatableId].begin(),
      amalgamatable->rectangles[amalgamatableId].end());

    //!< Delete the rectangle vector entry alltogether
    amalgamatable->rectangles.erase(
      amalgamatable->rectangles.begin() + amalgamatableId);
  }

} // namespace pandora_vision
