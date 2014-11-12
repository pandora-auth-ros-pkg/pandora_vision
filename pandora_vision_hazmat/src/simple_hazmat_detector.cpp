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
          
    
bool virtual findKeypointMatches(const cv::Mat &frameDescriptors ,
      const cv::Mat pattern & patternDescriptors ,  
      std::vector<cv::KeyPoint> *patternKeyPoints , 
      std::vector<cv::KeyPoint> *sceneKeyPoints  ) 
{
  // Define the vector that will contain the matches.
  std::vector< cv::DMatch > matches;
  
  // Perfom the matching using the matcher defined by the object.
  matcher_->match(patternDescriptors,frameDescriptors , matches );
  
  // Vector that containes the optimum matches.
  std::vector< cv::DMatch > goodMatches;
  
  // Minimum distance between matches.
  double minDist ;
  // Maximum distance between matches.
  double maxDist ;
  
  // If matches have been found
  if ( matches.size() > 0 )
  {
      // Quick calculation of max and min distances between keypoints
      for( int i = 0; i < descriptorsObject.rows; i++ )
      { 
        double dist = matches[i].distance;
        if( dist < minDist ) minDist = dist;
        if( dist > maxDist ) maxDist = dist;
      }
      
      // Keep only the matches that are below a certain distance 
      // threshold
      // TO DO : READ THE THRESHOLD FROM FILE.
    
      for( int i = 0; i < patternDescriptors.rows; i++ )
      { if( matches[i].distance < 3*minDist )
         { goodMatches.push_back( matches[i]); }
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
    patternKeyPoints->push_back( 
      patterns_[bestMatchIndex].keyPoints[ goodMatches[i].queryIdx ] );
      
    // Scene key points .
    sceneKeyPoints->push_back( 
      frameKeyPoints[ goodMatches[i].trainIdx ].pt );
  }
  
  
  goodMatches.clear();
  matches.clear();
  
  // Matches have been successfully found and keypoints correctly 
  // assigned.
  return true;
  
  }
