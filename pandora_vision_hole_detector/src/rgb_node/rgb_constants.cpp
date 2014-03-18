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

#include "rgb_node/rgb_constants.h"

namespace pandora_vision
{
  //!Frame characteristics (frameWidth,frameHeight,
  //!horizontal field of view, vertical field of view)
  int RgbParameters::frameWidth = 640;
  int RgbParameters::frameHeight = 480;
  int RgbParameters::hfov = 61.14;
  int RgbParameters::vfov = 48;
  
  //!< Canny parameters
  int RgbParameters::canny_ratio = 3;
  int RgbParameters::canny_kernel_size = 3;
  int RgbParameters::canny_low_threshold = 1000;
  int RgbParameters::canny_blur_noise_kernel_size = 3;
  
  //!< Sobel parameters
  int RgbParameters::sobel_scale = 1;
  int RgbParameters::sobel_delta = 0;
  
  //!< Blob detection parameters
  int RgbParameters::blob_min_threshold = 0;
  int RgbParameters::blob_max_threshold = 1255;
  int RgbParameters::blob_threshold_step = 5;
  int RgbParameters::blob_min_area = 550;
  int RgbParameters::blob_max_area = 8000;
  double RgbParameters::blob_min_convexity = 0.6;
  double RgbParameters::blob_max_convexity = 100;
  double RgbParameters::blob_min_inertia_ratio = 0;
  double RgbParameters::blob_max_circularity = 1.0;
  double RgbParameters::blob_min_circularity = 0.3;
  bool RgbParameters::blob_filter_by_color = false;
  bool RgbParameters::blob_filter_by_circularity = true;
}// namespace pandora_vision 
