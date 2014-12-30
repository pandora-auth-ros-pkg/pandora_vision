#include "pandora_vision_hazmat/surf_hazmat_detector.h"

/** 
 * SURF-feature based detector 
**/
  
void SurfHazmatDetector::getFeatures( const cv::Mat &frame , 
  const cv::Mat &mask , cv::Mat *descriptors , 
  std::vector<cv::KeyPoint> *keyPoints ) 
{
   s_( frame , mask , *keyPoints , *descriptors );
  }


