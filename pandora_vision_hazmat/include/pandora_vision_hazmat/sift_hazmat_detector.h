<<<<<<< HEAD
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


#ifndef PANDORA_VISION_HAZMAT_SIFT_HAZMAT_DETECTOR_H
#define PANDORA_VISION_HAZMAT_SIFT_HAZMAT_DETECTOR_H
=======
#ifndef SIFT_HAZMAT_DETECTOR_H
#define SIFT_HAZMAT_DETECTOR_H
>>>>>>> Added new Hazmat detector classes.

#include "pandora_vision_hazmat/simple_hazmat_detector.h"

/**
  @class SiftHazmatDetector
  @brief Implements a detector that uses SIFT ( Scale Invariant 
         Features Transformation) features to detect the different 
         signs.
**/

class SiftHazmatDetector : public SimpleHazmatDetector 
{
  public:
<<<<<<< HEAD
    SiftHazmatDetector();
    // SiftHazmatDetector object destructor.
    ~SiftHazmatDetector() {};
=======
    
    // SIFT hazmat detector constructor
    SiftHazmatDetector() : 
      SimpleHazmatDetector("SIFT") 
    {
      // Initiliaze  the Flann Based Matcher .
      matcher_ = cv::FlannBasedMatcher();
      s_ = cv::SIFT() ; 
    }


    // SiftHazmatDetector object destructor.
    ~SiftHazmatDetector() {} ;
>>>>>>> Added new Hazmat detector classes.
    
    // Function that returns the type of the feature detector used.
    //~ const virtual TrainerType getType( void ) ;
    
    // Calculates the keypoints of the image and its descriptors.
<<<<<<< HEAD
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints );
     
=======
    void virtual getFeatures(const cv::Mat &frame , 
      cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) ;
      
>>>>>>> Added new Hazmat detector classes.
          
  private:
  
    // SIFT detector 
<<<<<<< HEAD
    cv::SIFT s_;
    
};

#endif  // PANDORA_VISION_HAZMAT_SIFT_HAZMAT_DETECTOR_H
=======
    cv::SIFT s_ ;
    
  };

#endif
>>>>>>> Added new Hazmat detector classes.
