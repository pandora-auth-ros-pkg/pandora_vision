#ifndef HAZMAT_DETECTOR_H
#define HAZMAT_DETECTOR_H

#include "pandora_vision_hazmat/detector_interface.h"




class SimpleHazmatDetector : public Detector
{
  public:
  
    // Function that detects the Hazmat patterns.
    
    bool detect(const cv::Mat &frame , float *x , float *y ) ;
    
    // Function used for reading the training data from an xml file.
    
    void readData( void ) ;
    
    // Function used to select the input file for the pattern names.
    
    static void setFileName( const std::string &file ) ;
    
    
    // Function used to get the best feature matches between a frame
    // and a number of patterns.
    
    int getBestMatches( const cv::Mat &frame ,
     const cv::Mat &features , double *minDist , double *maxDist  ) ;
    
    // Function that returns the detected keypoints and features of the 
    // the image .
    
    void virtual getFeatures( const cv::Mat &frame , 
    cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) = 0; 
    
    void createMask(const cv::Mat &frame , cv::Mat *mask , 
      cv::Mat &data);
    
    // Returns the type of the features used. 
    //~ const TrainerType virtual getType( void ) = 0 ;
    
    // Return the name of the features used.
    const std::string getFeaturesName(void)  ;
    
    // Constructor
    HazmatDetector(const std::string &featureName);
    
    // Destructor 
    virtual ~HazmatDetector();
  
  
  protected:
  
    // Name of the file from which the training data is read.
    static std::string fileName_ ;
    
    // Vector of the patterns we wish to detect.
    std::vector<Pattern> patterns_ ;
    
    // Type of detector used .
    //~ const TrainerType type_ ;
    
    // Name of the detector .
    const std::string featuresName_;    
    
    // Flann Matcher .
    cv::FlannBasedMatcher matcher_ ;
    
    // Vector of matches.
    std::vector< cv::DMatch > bestMatches_ ;
    

};


class SiftHazmatDetector : public SimpleHazmatDetector 
{
  public:
  
    // SIFT Hazmat detector constructor .
    SiftHazmatDetector();  
    
    // SIFT Hazmat detector destructor .
    virtual ~SiftHazmatDetector();
    
    // Function that returns the type of the feature detector used.
    //~ const virtual TrainerType getType( void ) ;
    
    // Calculates the keypoints of the image and its descriptors.
    void virtual getFeatures(const cv::Mat &frame , 
      cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) ;
      
    //~ bool virtual detect(const cv::Mat &frame , 
      //~ float *x , float *y);
      
    //~ int SiftHazmatDetector::getBestMatches( const cv::Mat &frame ,
     //~ const cv::Mat &features , double *minDist , double *maxDist  );
  
    //~ void virtual createMask(cv::Mat &frame , cv::Mat *mask , 
      //~ cv::Mat &data) {}
  
    //~ void virtual SiftHazmatDetector::readData();
      
    //~ const std::string virtual getFeaturesName(void) ;
          
  private:
  
    // SIFT detector 
    cv::SIFT s_ ;
    
  };
  
/** 
 Future implementation.
  **/
  
//~ class SurfHazmatDetector : public HazmatDetector 
//~ {
  //~ public:
    //~ SurfHazmatDetector();
    //~ 
    //~ // Functions used to change the SURF algorithm parameters.
    //~ 
    //~ // Function for changing the threshold used for the hessian
    //~ // keypoint detector.
    //~ void setHessianThreshold(const double &t);
    //~ 
    //~ // Function that changes the number of pyramid octaves the keypoint
    //~ // detector uses.
    //~ void setOctavesNumber(const double &ocNum);
    //~ 
    //~ // Setter for the the number of octave layers in each octave.
    //~ void setOctaveLayersNumber(const double &ocLay);
    //~ 
    //~ // Setter for the flag that decides whether to use or not an
    //~ // extended descriptor vector( 128 element ) .
    //~ void setExtendedDescrFlag(const double &extFlag);
    //~ 
    //~ // Set the flag that will decide if the orientation of the features
    //~ // will be computed. 
    //~ void setOrientationFlag(const double &orFlag);
  //~ 
  //~ private:
  //~ 
    //~ // SIFT detector 
    //~ SURF s;
    //~ 
  //~ };
  
#endif
