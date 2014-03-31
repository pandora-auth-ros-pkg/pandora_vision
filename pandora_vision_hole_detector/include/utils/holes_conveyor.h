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

#ifndef UTILS_HOLES_CONVEYOR_H
#define UTILS_HOLES_CONVEYOR_H

#include "utils/defines.h"

namespace pandora_vision
{
  /**
    @brief The structure that represents holes found.
    @param keyPoints [std::vector<cv::KeyPoint>] The vector of the
    holes' keypoints
    @param rectangles [std::vector< std::vector<cv::Point2f> >] The
    vector of the holes' rotated bounding boxes vertices
    @param outlines [std::vector<std::vector<cv::Point> >] The
    vector of the holes' outlines
   **/
  struct HolesConveyor
  {
    std::vector<cv::KeyPoint> keyPoints;
    std::vector< std::vector<cv::Point2f> > rectangles;
    std::vector<std::vector<cv::Point> > outlines;
  };


  /**
    @Class HoleConveyor
    @brief Provides methods pertinent to the HolesConveyor struct
   **/
  class HolesConveyorUtils
  {
    public:

      /**
        @brief Copies one HolesConveyor struct to another
        @param[in] src [const HolesConveyor&] The source struct
        @param[out] dst [HolesConveyor*] The destination struct
        @return void
       **/
      static void copyTo(const HolesConveyor& src, HolesConveyor* dst);

      /**
        @brief Given two sources of struct HolesConveyor, this function
        fuses them into one struct.
        @param[in] srcA [const HolesConveyor&] The first HolesConveyor source
        @param[in] srcB [const HolesConveyor&] The second HolesConveyor source
        @param[out] dst [HolesConveyor*] The final struct
       **/
      static void fuse(const HolesConveyor& srcA,
        const HolesConveyor& srcB, HolesConveyor* dst);
  };

}

#endif  // namespace pandora_vision
