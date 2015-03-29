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


#ifndef PANDORA_VISION_HAZMAT_DETECTION_SURF_DETECTOR_H
#define PANDORA_VISION_HAZMAT_DETECTION_SURF_DETECTOR_H
 
#include "pandora_vision_hazmat/detection/feature_matching_detector.h"

class SurfDetector : public FeatureMatchingDetector 
{
  public:
    SurfDetector();

    ~SurfDetector()
    {
      delete[] matchers_;
    };

    // Functions used to change the SURF algorithm parameters.

    // Function for changing the threshold used for the hessian
    // keypoint detector.
    void setHessianThreshold(const double &t);

    // Function that changes the number of pyramid octaves the keypoint
    // detector uses.
    void setOctavesNumber(const double &ocNum);

    // Setter for the the number of octave layers in each octave.
    void setOctaveLayersNumber(const double &ocLay);

    // Setter for the flag that decides whether to use or not an
    // extended descriptor vector( 128 element ) .
    void setExtendedDescrFlag(const double &extFlag);

    // Set the flag that will decide if the orientation of the features
    // will be computed. 
    void setOrientationFlag(const double &orFlag);

    // Calculates the keypoints of the image and its descriptors.
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
        , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints );

  private:

    // SURF detector 
    cv::SURF s_;

};

#endif  // PANDORA_VISION_HAZMAT_DETECTION_SURF_DETECTOR_H_
