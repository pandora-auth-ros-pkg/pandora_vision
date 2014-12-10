#ifndef HISTOGRAM_MASK_H
#define HISTOGRAM_MASK_H

#include "pandora_vision_hazmat/hazmat_detector.h"

/** 
 @class HistogramMask
 @brief Class the implements the histogram back projection algorithm 
 **/
 

class HistogramMask : 
{
  public : 
    
    
    // Function the calculates the backprojection of the given histogram
    // on the image to extract regions of interest.
    void virtual createBackProjectionMask(const cv::Mat &frame , 
      cv::Mat *mask , const cv::Mat hist );
  
        
    // Constructor
    HistogramMask();
    
    virtual ~HistogramMask() 
    {
    } 
    
  private :
    // Pointer the base class that will be decorated.
    HazmatDetector *detector_ ;
      
    // A Matrix containing the resulting normalized image.
    static cv::Mat normImage ;
    
    // The histogram that will be backprojected to the frame
    // in order to find the regions of interest.
    static cv::Mat histogram_ ;
    
    // A flag that tests whether we will use the filter that implements
    // histogram backprojection.
    static bool histogramEnabled_ ;

  };
  


#endif
