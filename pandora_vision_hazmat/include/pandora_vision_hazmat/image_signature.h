#ifndef IMAGE_SIGNATURE_H
#define IMAGE_SIGNATURE_H

#include "pandora_vision_hazmat/hazmat_detector.h"

/**
 @class ImageSignature 
 @brief Class that implements the image signature saliency map.
**/

class ImageSignature
{
  public : 
    
    
    // Function that calculates the image signature.
    static void calculateSignature(const cv::Mat &image , 
      cv::Mat *imgSign);
    
    // Function the creates the mask that will be applied to the 
    // incoming frame based on the saliency map produced by the 
    // signature of the image.
    static void createSaliencyMapMask(const cv::Mat &frame , 
      cv::Mat *mask );
       
    // Return the array that containts the signs of an arbitrary
    // matrix.
    static void signFunction(const cv::Mat &array , cv::Mat *signs );
     
    // Constructor
    ImageSignature()
    { } ;
    
    virtual ~ImageSignature() 
    {
    } 
    
  private :

  };

#endif
