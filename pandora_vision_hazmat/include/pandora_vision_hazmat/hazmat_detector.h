<<<<<<< HEAD
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, P.A.N.D.O.R.A. Team.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Choutas Vassilis 
 *********************************************************************/


#ifndef PANDORA_VISION_HAZMAT_HAZMAT_DETECTOR_H
#define PANDORA_VISION_HAZMAT_HAZMAT_DETECTOR_H
=======
#ifndef HAZMAT_DETECTOR_H
#define HAZMAT_DETECTOR_H
>>>>>>> Added new Hazmat detector classes.

#include "pandora_vision_hazmat/detector_interface.h"




class HazmatDetector : public Detector
{
  public:
  
    // Function that detects the Hazmat patterns.
<<<<<<< HEAD
    
    bool virtual detect(const cv::Mat &frame , float *x , float *y );
    
    // Function used for reading the training data from an xml file.
    
    bool virtual readData( void );
  
    // Function that returns the detected keypoints and features of the 
    // the image .
    
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) 
    = 0;
    
    // Function that takes as input the scene descriptors, matches
    // them to the pattern descriptors and returns the corresponding
    // keypoints.
    bool virtual findKeypointMatches(const cv::Mat &frameDescriptors ,
      const cv::Mat &patternDescriptors , 
      const std::vector<cv::Point2f> patternKeyPoints ,
      const std::vector<cv::KeyPoint> sceneKeyPoints ,
      std::vector<cv::Point2f> *matchedPatternKeyPoints , 
      std::vector<cv::Point2f> *matchedSceneKeyPoints  , 
      const int &patternID = 0 ) = 0;
      
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
      std::vector<cv::Point2f> *sceneBB);

    // Function that sets the file where the pattern names are stored.
    static void setFileName( const std::string &file ) 
    {
      fileName_ = file;       
    }
    
    // Return the name of the features used.
    const std::string getFeaturesName(void)
    {
      return this->featuresName_;
    }
    
    static void setDims(const cv::Mat & frame)
    {
      width = frame.cols;
      height = frame.rows;
    }
    
    // Default Hazmat Detector Constructor.
    // Used when the decorators are initiliazed.
    HazmatDetector() {}    
   
    // Hazmat Detector class constructor.
    explicit HazmatDetector(const std::string &featureName); 
    
    // HazmatDetector Destructor.
    virtual ~HazmatDetector()
    {}
    
    // Return a pointer to the patterns. 
    boost::shared_ptr< std::vector<Pattern> > getPatternsPtr()
    {
      return patterns_;
    } 
    
    // Returns the total number of patterns.
    int getPatternsNumber(void)
    {
      return patterns_->size();
    }
    
 
  protected:
     // Vector of the patterns we wish to detect.
    boost::shared_ptr< std::vector<Pattern> > patterns_;
    
    // Structs used for finding the execution time. 
    #if defined(CHRONO) || defined(FEATURES_CHRONO) 
    struct timeval startwtime, endwtime;
    #endif 
  private : 
    // Name of the file from which the training data is read.
    static std::string fileName_;        

    // Type of detector used .
    //~ const TrainerType type_ ;
    
    // Name of the detector .
    const std::string featuresName_;    
    
    // Width of the input frame.
    static int width; 
    
    // Height of the input frame.
    static int height;
   
};
  
#endif  // PANDORA_VISION_HAZMAT_HAZMAT_DETECTOR_H
=======
    
    bool virtual detect(const cv::Mat &frame , float *x , float *y ) ;
    
    // Function used for reading the training data from an xml file.
    
    void virtual readData( void ) ;
    

    
    // Function that returns the detected keypoints and features of the 
    // the image .
    
    void virtual getFeatures( const cv::Mat &frame , const cv::Mat &mask
     , cv::Mat *descriptors , std::vector<cv::KeyPoint> *keyPoints ) 
    = 0 ;
    
    void virtual createMask(const cv::Mat &frame , cv::Mat *mask , 
      const cv::Mat &data = cv::Mat() );
    
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
      std::vector<cv::Point2f> *matchedSceneKeyPoints  ) = 0 ;
      
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
    bool virtual findBoundingBox(const std::vector<cv::Point2f> &patternKeyPoints , 
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
      : featuresName_(featureName)
    {
      readData();
    }
    
    // HazmatDetector Destructor.
    ~HazmatDetector()
    {
      patterns_.clear() ;
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
    
    // Width of the input frame.
    static int width ; 
    
    // Height of the input frame.
    static int height ;
    
    // Structs used for finding the execution time. 
    #ifdef CHRONO
    struct timeval startwtime, endwtime;
    #endif
    

};




  
#endif
>>>>>>> Added new Hazmat detector classes.
