#include "pandora_vision_hazmat/simple_hazmat_detector.h"

   
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
  std::vector< cv::DMatch > goodMatches ;
  
  // If we have found any matches.
  if ( matches.size() > 0 )
  {

    // We filter that matches by keeping only those
    // whose distance ration between the first and the second
    // best match is below a certain threshold.
    // TO DO : READ THE THRESHOLD FROM FILE.
    
    float ratio = 0.6 ;
    
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
  
  // Matches have been successfully found and keypoints correctly 
  // assigned.
  return true;  
  
  }
