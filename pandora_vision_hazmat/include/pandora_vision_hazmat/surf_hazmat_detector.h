#ifndef SURF_HAZMAT_DETECTOR_H
#define SURF_HAZMAT_DETECTOR_H
 
#include "pandora_vision_hazmat/simple_hazmat_detector.h"

class SurfHazmatDetector : public SimpleHazmatDetector 
{
   public:
     SurfHazmatDetector();

     ~SurfHazmatDetector()
     {} ;

     // Functions used to change the SURF algorithm parameters.
     
     // Function for changing the threshold used for the hessian
     // keypoint detector.
     void setHessianThreshold(const double &t);
     
     // Function that changes the number of pyramid octaves the keypoint
     // detector uses.
     void setOctavesNumber(const double &ocNum);
     
     // Setter for the the number of octave layers in each octave.
     void setOctaveLayersNumber(const double &ocLay);
     
     // Setter for the flag that decides whether to use or not an
     // extended descriptor vector( 128 element ) .
     void setExtendedDescrFlag(const double &extFlag);
     
     // Set the flag that will decide if the orientation of the features
     // will be computed. 
     void setOrientationFlag(const double &orFlag);
      
    // Calculates the keypoints of the image and its descriptors.
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) ;
 
     
   private:

   // SURF detector 
   cv::SURF s_;

   };

#endif
