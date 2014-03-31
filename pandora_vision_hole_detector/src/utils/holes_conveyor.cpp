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
    fuses them into one struct.
    @param[in] srcA [const HolesConveyor&] The first HolesConveyor source
    @param[in] srcB [const HolesConveyor&] The second HolesConveyor source
    @param[out] dst [HolesConveyor*] The final struct
   **/
  void HolesConveyorUtils::fuse(const HolesConveyor& srcA,
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

} // namespace pandora_vision
