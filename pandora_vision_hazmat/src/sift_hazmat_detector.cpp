#include "pandora_vision_hazmat/sift_hazmat_detector.h"

/** 
 * Sift detector 
**/

/**
  
 **/
  
void SiftHazmatDetector::getFeatures( const cv::Mat &frame , 
  const cv::Mat &mask , cv::Mat *descriptors , 
  std::vector<cv::KeyPoint> *keyPoints ) 
{
   s_( frame , mask , *keyPoints , *descriptors );
  }


