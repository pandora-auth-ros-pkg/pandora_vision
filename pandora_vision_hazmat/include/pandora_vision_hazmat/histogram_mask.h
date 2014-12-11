#ifndef HISTOGRAM_MASK_H
#define HISTOGRAM_MASK_H

#include "pandora_vision_hazmat/hazmat_detector.h"

/** 
 @class HistogramMask
 @brief Class the implements the histogram back projection algorithm 
 **/
 

class HistogramMask  
{
  public : 
    
    
    // Function the calculates the backprojection of the given histogram
    // on the image to extract regions of interest.
    void static createBackProjectionMask(const cv::Mat &frame , 
      cv::Mat *mask , const cv::Mat hist );
  
        
    // Constructor
    HistogramMask();
    

    
  private :

    
    // 2D range of the histogram. 
    static float* ranges_[2] ;
    
    // Channels when creating the histogram.
    static int channels_[2] ;
    
    static float hueRange_[2] ;
    static float satRange_[2] ;
    
    // Resize scale for input image.
    static int scale_ ;
    
    // The threshold for the mask.
    static int thresh_ ;
    
    // Maximum value of the intensity of every pixel of the mask.
    static int maxValue_ ;

  };
  

  


#endif
