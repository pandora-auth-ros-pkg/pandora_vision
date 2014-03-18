/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, P.A.N.D.O.R.A. Team.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*
*  are met:
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
* Author: Despoina Paschalidou
*********************************************************************/

#ifndef RGB_NODE_RGB_CONSTANTS_H 
#define RGB_NODE_RGB_CONSTANTS_H 

namespace pandora_vision
{
  struct RgbParameters
  {
    //! Frame characteristics (frameWidth,frameHeight,
    //! horizontal field of view, vertical field of view)
    static int frameWidth;
    static int frameHeight;
    static int hfov;
    static int vfov;
  
    //! Canny parameters
    static int canny_ratio;
    static int canny_kernel_size;
    static int canny_low_threshold;
    static int canny_blur_noise_kernel_size;
    
    //! Sobel parameters
    static int sobel_scale;
    static int sobel_delta;
 
    //! Blob detection parameters
    static int blob_min_threshold;
    static int blob_max_threshold;
    static int blob_threshold_step;
    static int blob_min_area;
    static int blob_max_area;
    static double blob_min_convexity;
    static double blob_max_convexity;
    static double blob_min_inertia_ratio;
    static double blob_max_circularity;
    static double blob_min_circularity;
    static bool blob_filter_by_color;
    static bool blob_filter_by_circularity;
    
    //! Debug flags
    static bool debug_enable;
  };
}// namespace pandora_vision
#endif  // RGB_NODE_RGB_CONSTANTS_H
