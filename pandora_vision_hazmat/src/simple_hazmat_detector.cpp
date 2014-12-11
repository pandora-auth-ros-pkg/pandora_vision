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


#include "pandora_vision_hazmat/simple_hazmat_detector.h"

   

SimpleHazmatDetector::SimpleHazmatDetector(
  const std::string &featureName) : HazmatDetector(featureName)
  {
    
    }
    
/**
  @brief Function used to detect matches between a pattern and the 
         input frame.
  @param frameDescriptors [const cv::Mat & ] : Descriptors of the frame.
  @param patternDescriptors [const cv::Mat &] : Descriptors of the 
         pattern.
  @param patternKeyPoints [std::vector<cv::KeyPoint> *] : Vector of 
         detected keypoints in the pattern.
  @param sceneKeyPoints [std::vector<cv::KeyPoint> *] : Vector of 
         detected keypoints in the frame.
**/
          
    
bool SimpleHazmatDetector::findKeypointMatches(
      const cv::Mat &frameDescriptors ,
      const cv::Mat &patternDescriptors , 
      const std::vector<cv::Point2f> patternKeyPoints ,
      const std::vector<cv::KeyPoint> sceneKeyPoints ,
      std::vector<cv::Point2f> *matchedPatternKeyPoints , 
      std::vector<cv::Point2f> *matchedSceneKeyPoints , 
      const int &patternID  ) 
{
  // Clear the vectors containing the matched keypoints.
  matchedSceneKeyPoints->clear();
  matchedPatternKeyPoints->clear();

  // Define the vector of vectors that contains the matches.
  // Each element is a vector that contains the first,the second
  // up to the n-th best match.
  std::vector< std::vector<cv::DMatch> > matches;
  
  // Perfom the matching using the matcher of the patternID-th 
  // pattern and find the top 2 correspondences. 
  matchers_[patternID]->knnMatch( frameDescriptors , matches , 2 );

  // The vector containing the best matches
  std::vector< cv::DMatch > goodMatches;

  if ( matches.size() > 0 )
  {

    // We filter that matches by keeping only those
    // whose distance ration between the first and the second
    // best match is below a certain threshold.
    // TO DO : READ THE THRESHOLD FROM FILE.
    
    float ratio = 0.6;
    
    for( int i = 0; i <  matches.size() ; i++ )
    { 
        if( matches[i][0].distance < ratio*matches[i][1].distance )
        { 
           goodMatches.push_back( matches[i][0]); 
        }
    }
    
  }
  // No matches found.
  else
    return false;
  
  // Add the keypoints of the matches found to the corresponding
  // vectors for the pattern and the scene.
  for( int i = 0; i < goodMatches.size(); i++ )
  { 
    // Pattern key points .
    matchedPatternKeyPoints->push_back( 
      patternKeyPoints[ goodMatches[i].trainIdx ] );
      
    // Scene key points .
    matchedSceneKeyPoints->push_back( 
      sceneKeyPoints[ goodMatches[i].queryIdx ].pt );
  }
  
  // If we have less than 4 matches then we cannot find the Homography
  // and this is an invalid pattern.
  
  if ( goodMatches.size() < 4 )
    return false;
    

  goodMatches.clear();
  matches.clear();

  return true;
  
  }
