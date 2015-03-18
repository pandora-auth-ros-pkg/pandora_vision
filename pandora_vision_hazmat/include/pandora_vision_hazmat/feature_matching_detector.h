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


#ifndef PANDORA_VISION_HAZMAT_FEATURE_MATCHING_DETECTOR_H
#define PANDORA_VISION_HAZMAT_FEATURE_MATCHING_DETECTOR_H

#include "pandora_vision_hazmat/planar_object_detector.h"

/**
 @class FeatureMatchingDetector
 @brief An interface for all the detectors that extract features
        from an image and compare them with those from the training set
        to find a possible match in the current frame.
**/

 
class FeatureMatchingDetector : public PlanarObjectDetector
{
  public:
  
    bool virtual readData(void);    
    
    // Function that returns the detected keypoints and features of the 
    // the image .
    
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) 
      = 0; 
    
    // Find the matches between the pattern and the query frame.
    bool virtual findKeypointMatches(const cv::Mat &frameDescriptors ,
      const cv::Mat &patternDescriptors , 
      const std::vector<cv::Point2f> patternKeyPoints ,
      const std::vector<cv::KeyPoint> sceneKeyPoints ,
      std::vector<cv::Point2f> *matchedPatternKeyPoints , 
      std::vector<cv::Point2f> *matchedSceneKeyPoints ,
      const int &patternID = 0 );

           
    // Find the matches between the pattern and the query frame.
    bool virtual findKeypointMatches(const cv::Mat &frameDescriptors ,
      const cv::Mat &patternDescriptors , 
      const std::vector<cv::Point2f> patternKeyPoints ,
      const std::vector<cv::KeyPoint> sceneKeyPoints ,
      std::vector<cv::Point2f> *matchedPatternKeyPoints , 
      std::vector<cv::Point2f> *matchedSceneKeyPoints  ) ;
    
    // Returns the type of the features used. 
    //~ const TrainerType virtual getType( void ) = 0 ;
    
    /* virtual const cv::FlannBasedMatcher& getMatcher(void)
    {
      return matcher_;
    }*/
    
    // Constructor
    explicit FeatureMatchingDetector(const std::string &featureName);
    
    // Destructor 
    virtual ~FeatureMatchingDetector() {};
  
  protected:
  
    /* Array of descriptors matchers used to match the keypoints
     * found in the input frame and those of the training set
    */
    cv::Ptr<cv::DescriptorMatcher> *matchers_;
    
   
    
  
};

#endif  // PANDORA_VISION_HAZMAT_FEATURE_MATCHING_DETECTOR_H
