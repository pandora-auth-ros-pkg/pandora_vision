#ifndef SIMPLE_HAZMAT_DETECTOR_H
#define SIMPLE_HAZMAT_DETECTOR_H

#include "pandora_vision_hazmat/hazmat_detector.h"

/**
 @class SimpleHazmatDetector
 @brief A Simple detector interface that is used to produce the 
        different kind of objects that represent the types of features
        used.
**/

class SimpleHazmatDetector : public HazmatDetector
{
  public:
  
    // Function used to select the input file for the pattern names.
    
    static void setFileName( const std::string &file ) ;
    
    // Function that returns the detected keypoints and features of the 
    // the image .
    
    void virtual getFeatures( const cv::Mat &frame , 
    cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) = 0; 
    
    // Find the matches between the pattern and the query frame.
    bool virtual findKeypointMatches(const cv::Mat &frameDescriptors ,
      const cv::Mat pattern & patternDescriptors , 
      std::vector<cv::KeyPoint> *patternKeyPoints , 
      std::vector<cv::KeyPoint> *SceneKeyPoints  ) ;
    
    // Returns the type of the features used. 
    //~ const TrainerType virtual getType( void ) = 0 ;
    
    // Constructor
    SimpleHazmatDetector(const std::string &featureName);
    
    // Destructor 
    virtual ~SimpleHazmatDetector() {} ;
  
  protected:
  
    // Flann Matcher .
    // It is initialised by every feature detector separately so as to
    // set the correct parameters.
    cv::FlannBasedMatcher matcher_ ;
    
    
  
};

#endif
