#ifndef SIFT_HAZMAT_DETECTOR_H
#define SIFT_HAZMAT_DETECTOR_H

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
    
    // Function that returns the type of the feature detector used.
    //~ const virtual TrainerType getType( void ) ;
    
    // Calculates the keypoints of the image and its descriptors.
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) ;
     
          
  private:
  
    // SIFT detector 
    cv::SIFT s_ ;
    
  };

#endif
