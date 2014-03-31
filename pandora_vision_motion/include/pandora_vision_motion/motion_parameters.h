/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Despoina Paschalidou
*********************************************************************/

#ifndef PANDORA_VISION_MOTION_MOTION_PARAMETERS_H
#define PANDORA_VISION_MOTION_MOTION_PARAMETERS_H

namespace pandora_vision
{
  struct MotionParameters
  {
    /// Length of the history according to which we calculate background image
    static int history;
    /// Threshold of the sqaured Mahalanobis distance to decide whether
    /// it is well described by the background model
    static int varThreshold;
    /// Parameter defining whether shadow detection should be enabled
    static bool bShadowDetection;
    /// Maximum allowed number of mixture components
    static int nmixtures;
    
    /// Threshold between pixel (grayscale) values to be considered "different" between 2 frames
    static int diff_threshold;
    /// Evaluation threshold: higher value means a lot of movement
    static double motion_high_thres;
    /// Evaluation threshold: higher value means a little movement - less means no movement at all
    static double motion_low_thres;
    
  };

} // namespace pandora_vision
#endif  // PANDORA_VISION_MOTION_MOTION_PARAMETERS_H
