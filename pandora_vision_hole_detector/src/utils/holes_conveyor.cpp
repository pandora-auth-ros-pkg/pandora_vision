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

#include "utils/holes_conveyor.h"

namespace pandora_vision
{
  /**
    @brief Copies one HolesConveyor struct to another
    @param[in] src [const HolesConveyor&] The source struct
    @param[out] dst [HolesConveyor*] The destination struct
    @return void
   **/
  void HolesConveyorUtils::copyTo(const HolesConveyor& src,
    HolesConveyor* dst)
  {
    //!< Insert the src HolesConveyor into the dst HolesConveyor
    for (int i = 0; i < src.keyPoints.size(); i++)
    {
      dst->keyPoints.push_back(src.keyPoints[i]);

      std::vector<cv::Point> tempOutline;
      for (int j = 0; j < src.outlines[i].size(); j++)
      {
        tempOutline.push_back(src.outlines[i][j]);
      }
      dst->outlines.push_back(tempOutline);

      std::vector<cv::Point2f> tempRectangle;
      for (int j = 0; j < src.rectangles[i].size(); j++)
      {
        tempRectangle.push_back(src.rectangles[i][j]);
      }
      dst->rectangles.push_back(tempRectangle);
    }
  }


  /**
    @brief Given two sources of struct HolesConveyor, this function
    merge them into one struct.
    @param[in] srcA [const HolesConveyor&] The first HolesConveyor source
    @param[in] srcB [const HolesConveyor&] The second HolesConveyor source
    @param[out] dst [HolesConveyor*] The final struct
   **/
  void HolesConveyorUtils::merge(const HolesConveyor& srcA,
    const HolesConveyor& srcB, HolesConveyor* dst)
  {
    //!< Insert the srcA HolesConveyor into the dst HolesConveyor
    for (int i = 0; i < srcA.keyPoints.size(); i++)
    {
      dst->keyPoints.push_back(srcA.keyPoints[i]);

      std::vector<cv::Point> tempOutline;
      for (int j = 0; j < srcA.outlines[i].size(); j++)
      {
        tempOutline.push_back(srcA.outlines[i][j]);
      }
      dst->outlines.push_back(tempOutline);

      std::vector<cv::Point2f> tempRectangle;
      for (int j = 0; j < srcA.rectangles[i].size(); j++)
      {
        tempRectangle.push_back(srcA.rectangles[i][j]);
      }
      dst->rectangles.push_back(tempRectangle);
    }

    //!< Insert the srcB HolesConveyor into the dst HolesConveyor
    for (int i = 0; i < srcB.keyPoints.size(); i++)
    {
      dst->keyPoints.push_back(srcB.keyPoints[i]);

      std::vector<cv::Point> tempOutline;
      for (int j = 0; j < srcB.outlines[i].size(); j++)
      {
        tempOutline.push_back(srcB.outlines[i][j]);
      }
      dst->outlines.push_back(tempOutline);

      std::vector<cv::Point2f> tempRectangle;
      for (int j = 0; j < srcB.rectangles[i].size(); j++)
      {
        tempRectangle.push_back(srcB.rectangles[i][j]);
      }
      dst->rectangles.push_back(tempRectangle);
    }
  }



  /**
    @brief Extracts the specified hole from a HolesConveyor into a new
    HolesConveyor struct that is returned
    @param[in] conveyor [const HolesConveyor&] The HolesConveyor struct
    @param[in] index [const int&] The index of the hole inside the conveyor
    @return A HolesConveyor struct that containes the index-th hole of the
    conveyor
   **/
  HolesConveyor HolesConveyorUtils::getHole(const HolesConveyor& conveyor,
    const int& index)
  {
    HolesConveyor dst;

    //!< Get the index-th hole's keypoint
    dst.keyPoints.push_back(conveyor.keyPoints[index]);

    //!< Get the index-th hole's outline
    std::vector<cv::Point> tempOutline;
    for (int j = 0; j < conveyor.outlines[index].size(); j++)
    {
      tempOutline.push_back(conveyor.outlines[index][j]);
    }
    dst.outlines.push_back(tempOutline);

    //!< Get the index-th hole's rectangle
    std::vector<cv::Point2f> tempRectangle;
    for (int j = 0; j < conveyor.rectangles[index].size(); j++)
    {
      tempRectangle.push_back(conveyor.rectangles[index][j]);
    }
    dst.rectangles.push_back(tempRectangle);

    return dst;
  }



  /**
    @brief Replaces a specified hole from a HolesConveyor dst struct
    with the hole of index srcIndex of the src HolesConveyor struct entry
    @param[in] src [const HolesConveyor&] The HolesConveyor source struct
    @param[in] srcIndex [const int&] The index of the hole inside the
    src conveyor that will be copied into the dst HolesConveyor struct,
    in the dstIndex position
    @param[out] dst [HolesConveyor*] The HolesConveyor destination struct
    @param[in] dstIndex [const int&] The index of the hole inside the
    dst conveyor that will be replaced by the srcIndex-th of the src
    HolesConveyor struct
    @return void
   **/
  void HolesConveyorUtils::replaceHole(const HolesConveyor& src,
    const int& srcIndex, HolesConveyor* dst, const int& dstIndex)
  {
    //!< Replace the dst's dstIndex-th hole's keypoint
    dst->keyPoints.at(dstIndex) = src.keyPoints.at(srcIndex);

    //!< Erase the outline points for entry dstIndex of the dst
    dst->outlines[dstIndex].erase(
      dst->outlines[dstIndex].begin(),
      dst->outlines[dstIndex].end());

    //!< Replace the dst's dstIndex-th hole's outline points
    dst->outlines.at(dstIndex) = src.outlines[srcIndex];

    //!< Erase the rectangle points for entry dstIndex of the dst
    dst->rectangles[dstIndex].erase(
      dst->rectangles[dstIndex].begin(),
      dst->rectangles[dstIndex].end());

    //!< Replace the dst's dstIndex-th hole's rectangle points
    dst->rectangles.at(dstIndex) = src.rectangles[srcIndex];
  }



  /**
    @brief Replaces an entire HolesConveyor struct with another
    @param[in] src [const HolesConveyor&] The source conveyor struct
    @param[out] dst [HolesConveyor*] The destination conveyor struct
    @return void
   **/
  void HolesConveyorUtils::replace(const HolesConveyor& src, HolesConveyor* dst)
  {
    //!< Clear the dst
    clear(dst);

    //!< Fill it with the src
    copyTo(src, dst);
  }



  /**
    @brief Hollows a HolesConveyor struct, deleting every entry in it
    @param[in][out] conveyor [HolesConveyor*] The conveyor struct that will
    be cleared
    @return void
   **/
  void HolesConveyorUtils::clear(HolesConveyor* conveyor)
  {
    //!< Delete the keypoints
    conveyor->keyPoints.erase(conveyor->keyPoints.begin(),
      conveyor->keyPoints.end());


    //!< Delete each outline point from its respective vector
    for (int i = 0; i < conveyor->outlines.size(); i++)
    {
      conveyor->outlines[i].erase(conveyor->outlines[i].begin(),
        conveyor->outlines[i].end());
    }

    //!< Delete the outline vector alltogether
    conveyor->outlines.erase(conveyor->outlines.begin(),
      conveyor->outlines.end());


    //!< Delete each outline point from its respective vector
    for (int i = 0; i < conveyor->rectangles.size(); i++)
    {
      conveyor->rectangles[i].erase(conveyor->rectangles[i].begin(),
        conveyor->rectangles[i].end());
    }

    //!< Delete the rectangles vector alltogether
    conveyor->rectangles.erase(conveyor->rectangles.begin(),
      conveyor->rectangles.end());
  }

} // namespace pandora_vision
