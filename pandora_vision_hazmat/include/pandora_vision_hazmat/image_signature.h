#ifndef IMAGE_SIGNATURE_H
#define IMAGE_SIGNATURE_H

#include "pandora_vision_hazmat/hazmat_detector.h"

/**
 @class ImageSignature 
 @brief Class that implements the image signature saliency map.
**/

class ImageSignature : public HazmatDetector
{
  public : 
    
    // Function that calculates the image signature.
    static void calculateSignature(const cv::Mat &image , 
      cv::Mat *imgSign);
    
    // Function the creates the mask that will be applied to the 
    // incoming frame based on the saliency map produced by the 
    // signature of the image.
    void virtual createMask(const cv::Mat &frame , cv::Mat *mask , 
      const cv::Mat &data = cv::Mat() );
    
    // Return the array that containts the signs of an arbitrary
    // matrix.
    static void signFunction(const cv::Mat &array , cv::Mat *signs );
    
    // The class who is being decorated is called to detect the 
    // pattern we wish to find.
    
    //~ bool virtual detect(const cv::Mat &frame , 
      //~ float *x , float *y )
      //~ 
    //~ {
      //~ return detector_->detect(frame,x,y);
    //~ }
    
    // Function used for reading the training data from an xml file.
    // The necessary data has to be read by the object point by the
    // detector_ pointer.
    //~ void virtual readData( void ) 
    //~ {
      //~ return ;
      //~ }
    
    // Function used to get the best feature matches between a frame
    // and a number of patterns.
    // In this decorator class it has no need to be implemented
    // since it's purpose is to create the correct filter.
    //~ int virtual getBestMatches( const cv::Mat &frame ,
     //~ const cv::Mat &features , double *minDist , double *maxDist  ) 
     //~ {
       //~ return 0;
     //~ }
     
    // Constructor
    ImageSignature(HazmatDetector *baseDetector);
    
    virtual ~ImageSignature() 
    {
    } 
    
  private :
  
    // Pointer the base class that will be decorated.
    HazmatDetector *detector_ ;
  };

#endif
