#ifndef MULTIPLE_FLANN_MATCHER
#define MULTIPLE_FLANN_MATCHER

#include "pandora_vision_hazmat/hazmat_detector.h"

class MultipleFlannMatcher : public HazmatDetector
{
  public :
  
    
    // Function that takes as input the scene descriptors, matches
    // them to the pattern descriptors using one pretrained FLANN
    // matcher for every pattern and returns the corresponding
    // keypoints.
    bool virtual findKeypointMatches(const cv::Mat &frameDescriptors ,
      const cv::Mat &patternDescriptors , 
      const std::vector<cv::Point2f> patternKeyPoints ,
      const std::vector<cv::KeyPoint> sceneKeyPoints ,
      std::vector<cv::Point2f> *matchedPatternKeyPoints , 
      std::vector<cv::Point2f> *matchedSceneKeyPoints  , 
      const int &patternID = 0 ); 
    
    
    // Function that returns the features 
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) 
    {
      detector->getFeatures(frame, mask , descriptors , keyPoints);
    }
    
    // Decorator Constructor.
    MultipleFlannMatcher(HazmatDetector* detectorPtr);
    
    // Default Constructor
    // Will be implemented to be independent of the decoration process
    // so as to be called by an outside module.
    MultipleFlannMatcher()
    {} ;
    
  private : 
    HazmatDetector* detector ;
    
    cv::DescriptorMatcher **matchers;
  };

#endif
