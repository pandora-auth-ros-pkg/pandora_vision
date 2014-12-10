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
      std::vector<cv::Point2f> *matchedSceneKeyPoints  ) 
{
  // Define the vector that will contain the matches.
  std::vector< cv::DMatch > matches;
  
  // Perfom the matching using the matcher defined by the object.
  matcher_.match(patternDescriptors,frameDescriptors , matches );
  
  // Vector that containes the optimum matches.
  std::vector< cv::DMatch > goodMatches;
  
  // Minimum distance between matches.
  double minDist =  std::numeric_limits<double>::max() ;
  // Maximum distance between matches.

  // If matches have been found
  if ( matches.size() > 0 )
  {

      // Quick calculation of max and min distances between keypoints
      for( int i = 0; i < patternDescriptors.rows; i++ )
      { 
        double dist = matches[i].distance;
        if( dist < minDist ) minDist = dist;
      }


      // Keep only the matches that are below a certain distance 
      // threshold
      // TO DO : READ THE THRESHOLD FROM FILE.
    
      for( int i = 0; i < patternDescriptors.rows; i++ )
      { 
        if( matches[i].distance < 2*minDist )
          goodMatches.push_back( matches[i]); 
      }
      
      
      

  }
  else
  {
    return false;
  }
  
  
  // Find the keypoints that correspond to the best matches.

  
  for( int i = 0; i < goodMatches.size(); i++ )
  {
    
    // Pattern key points .
    matchedPatternKeyPoints->push_back( 
      patternKeyPoints[ goodMatches[i].queryIdx ] );
      
    // Scene key points .
    matchedSceneKeyPoints->push_back( 
      sceneKeyPoints[ goodMatches[i].trainIdx ].pt );
      
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
