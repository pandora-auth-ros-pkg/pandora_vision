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
    

    
    // Function that returns the detected keypoints and features of the 
    // the image .
    
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) 
    = 0 ;
    

    
    // Returns the type of the features used. 
    //~ const TrainerType virtual getType( void ) = 0 ;
    
    // Function that takes as input the scene descriptors, matches
    // them to the pattern descriptors and returns the corresponding
    // keypoints.
    bool virtual findKeypointMatches(const cv::Mat &frameDescriptors ,
      const cv::Mat &patternDescriptors , 
      const std::vector<cv::Point2f> patternKeyPoints ,
      const std::vector<cv::KeyPoint> sceneKeyPoints ,
      std::vector<cv::Point2f> *matchedPatternKeyPoints , 
      std::vector<cv::Point2f> *matchedSceneKeyPoints  , 
      const int &patternID = 0 ) = 0 ;
      
    /**
    * @brief Find the homography between the scene and the pattern keypoints
    * , check if it is valid and return the bounding box of the detected
    * pattern .
    * @param patternKeyPoints [std::vector<cv::KeyPoint> &] : Input 
    * keypoints from detected descriptor matches on the pattern.
    * @param sceneKeyPoints [std::vector<cv::KeyPoint> &] : Input 
    * keypoints from detected descriptor matches in the scene.
    * @param patternBB [std::vector<cv::Point2f *] : Vector of 2D float
    * Points that containes the bounding box and the center of the 
    * pattern.
    
    **/
    bool virtual findBoundingBox(const std::vector<cv::Point2f> 
      &patternKeyPoints , 
      const std::vector<cv::Point2f> &sceneKeyPoints , 
      const std::vector<cv::Point2f> &patternBB , 
      std::vector<cv::Point2f> *sceneBB) ;

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
    
    static void setDims(const cv::Mat & frame)
    {
      width = frame.cols ;
      height = frame.rows;
    }
    
    // Default Hazmat Detector Constructor.
    // Used when the decorators are initiliazed.
    HazmatDetector() {}    
   
    // Hazmat Detector class constructor.
    explicit HazmatDetector(const std::string &featureName) 
      : featuresName_(featureName) , patterns_(new std::vector<Pattern>) 
    {

      readData();
    }
    
    // HazmatDetector Destructor.
    virtual ~HazmatDetector()
    {
    }
    
    // Return a pointer to the patterns. 
    boost::shared_ptr< std::vector<Pattern> > getPatternsPtr()
    {
      return patterns_ ;
    } 
    
    // Returns the total number of patterns.
    int getPatternsNumber(void)
    {
      return patterns_->size() ;
    }
    
 
  protected:
     // Vector of the patterns we wish to detect.
    boost::shared_ptr< std::vector<Pattern> > patterns_ ;
    
    // Structs used for finding the execution time. 
    #if defined(CHRONO) || defined(FEATURES_CHRONO) 
    struct timeval startwtime, endwtime;
    #endif 
  private : 
    // Name of the file from which the training data is read.
    static std::string fileName_ ;        

    // Type of detector used .
    //~ const TrainerType type_ ;
    
    // Name of the detector .
    const std::string featuresName_;    
    
    // Width of the input frame.
    static int width ; 
    
    // Height of the input frame.
    static int height ;
    
    
    
    

};




  
#endif
