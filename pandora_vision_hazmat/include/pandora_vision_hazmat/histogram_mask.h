/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
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
 * Authors: Choutas Vassilis 
 *********************************************************************/


#ifndef PANDORA_VISION_HAZMAT_HISTOGRAM_MASK_H
#define PANDORA_VISION_HAZMAT_HISTOGRAM_MASK_H

#include "pandora_vision_hazmat/hazmat_detector.h"

/** 
 @class HistogramMask
 @brief Class the implements the histogram back projection algorithm 
 **/
 

class HistogramMask  
{
  public : 


    // Function the calculates the backprojection of the given histogram
    // on the image to extract regions of interest.
    static void createBackProjectionMask(const cv::Mat &frame ,
        cv::Mat *mask , const cv::Mat hist );

    // Constructor
    HistogramMask();

  private :

    // 2D range of the histogram. 
    static float* ranges_[2];

    // Channels when creating the histogram.
    static int channels_[2];

    static float hueRange_[2];
    static float satRange_[2];

    // Resize scale for input image.
    static int scale_;

    // The threshold for the mask.
    static int thresh_;

    // Maximum value of the intensity of every pixel of the mask.
    static int maxValue_;

};


#endif  // PANDORA_VISION_HAZMAT_HISTOGRAM_MASK_H_
