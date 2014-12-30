#include "pandora_vision_hazmat/orb_hazmat_detector.h"

/** 
 *  ORB detector 
**/
  
void OrbHazmatDetector::getFeatures( const cv::Mat &frame , 
  const cv::Mat &mask , cv::Mat *descriptors , 
  std::vector<cv::KeyPoint> *keyPoints ) 
{
   s_( frame , mask , *keyPoints , *descriptors );
  }


