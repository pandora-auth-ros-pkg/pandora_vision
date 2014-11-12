#ifndef HAZMAT_DETECTOR_H
#define HAZMAT_DETECTOR_H

#include "pandora_vision_hazmat/detector_interface.h"




class HazmatDetector : public Detector
{
  public:
  
    // Function that detects the Hazmat patterns.
    
    bool virtual detect(const cv::Mat &frame , float *x , float *y ) ;
    
    // Function used for reading the training data from an xml file.
    
    void virtual readData( void ) ;
    
    
    // Function used to get the best feature matches between a frame
    // and a number of patterns.
    
    int getBestMatches( const cv::Mat &frame ,
     const cv::Mat &features , double *minDist , double *maxDist  ) ;
    
    // Function that returns the detected keypoints and features of the 
    // the image .
    
    void virtual getFeatures( const cv::Mat &frame , 
    cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) 
    {
      descriptors->data = NULL ;
      } 
    
    void virtual createMask(const cv::Mat &frame , cv::Mat *mask , 
      const cv::Mat &data = cv::Mat() );
    
    // Returns the type of the features used. 
    //~ const TrainerType virtual getType( void ) = 0 ;
    
    // Function that takes as input the scene descriptors, matches
    // them to the pattern descriptors and returns the corresponding
    // keypoints.
    bool virtual findKeypointMatches(const cv::Mat &frameDescriptors ,
      const cv::Mat pattern & patternDescriptors , 
      std::vector<cv::KeyPoint> *patternKeyPoints , 
      std::vector<cv::KeyPoint> *sceneKeyPoints  ) = 0 ;
      
    bool virtual findBoundingBox(std::vector<cv::KeyPoint> &patternKeyPoints , 
      std::vector<cv::KeyPoint> &sceneKeyPoints , 
      std::vector<cv::Point2f> *patternBB) ;

    // Function that sets the file where the pattern names are stored.
    static void setFileName( const std::string &file ) 
    {
      fileName_ = file ;       
    }
    
    // Return the name of the features used.
    const std::string getFeaturesName(void)
    {
      return this->featuresName_ ;
    }
    
    // Default Hazmat Detector Constructor.
    // Used when the decorators are initiliazed.
    HazmatDetector() {}    
   
    // Hazmat Detector class constructor.
    explicit HazmatDetector(const std::string &featureName) 
      : featuresName_(featureName)
    {
      // Initialize the Flann matcher.
      readData();
    }
    
    // HazmatDetector Destructor.
    ~HazmatDetector()
    {
      patterns_.clear() ;
      bestMatches_.clear() ;
}
  
  
  protected:
  
    // Name of the file from which the training data is read.
    static std::string fileName_ ;
    
    // Vector of the patterns we wish to detect.
    std::vector<Pattern> patterns_ ;
    
    // Type of detector used .
    //~ const TrainerType type_ ;
    
    // Name of the detector .
    const std::string featuresName_;    
    
    

};




  
#endif
