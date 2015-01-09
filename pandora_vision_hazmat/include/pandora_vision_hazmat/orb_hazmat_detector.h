#ifndef ORB_HAZMAT_DETECTOR_H
#define ORB_HAZMAT_DETECTOR_H
 
#include "pandora_vision_hazmat/simple_hazmat_detector.h"

class OrbHazmatDetector : public SimpleHazmatDetector 
{
   public:

     // Default Constructor
     OrbHazmatDetector();
   
   
    // OrbHazmatDetector object destructor.
    ~OrbHazmatDetector() {} ;
    
    // Function that returns the type of the feature detector used.
    //~ const virtual TrainerType getType( void ) ;
    
    // Calculates the keypoints of the image and its descriptors.
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) ;
     
   private:

   // ORB detector 
   cv::ORB s_;

   };

#endif
