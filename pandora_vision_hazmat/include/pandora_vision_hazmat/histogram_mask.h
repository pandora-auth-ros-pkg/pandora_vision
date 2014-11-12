#ifndef HISTOGRAM_MASK_H
#define HISTOGRAM_MASK_H

#include "pandora_vision_hazmat/hazmat_detector.h"

/** 
 @class HistogramMask
 @brief Class the implements the histogram back projection algorithm 
 **/
 

class HistogramMask : public HazmatDetector
{
  public : 
    
    // Function that calculates the normalized histogram
    static void calcNormYUVHist(const cv::Mat &image , cv::Mat *hist);
    
    // Function the calculates the backprojection 
    void virtual createMask(const cv::Mat &frame , cv::Mat *mask , 
      const cv::Mat &data = cv::Mat() );
    
    // Normalize an image by dividing it with the channel ch.
    static void normalizeImage(cv::Mat &image , int ch );

     
    // Constructor
    HistogramMask(HazmatDetector *baseDetector);
    
    virtual ~HistogramMask() 
    {
    } 
    
  private :
    // Pointer the base class that will be decorated.
    HazmatDetector *detector_ ;
      
    // A Matrix containing the resulting normalized image.
    static cv::Mat normImage ;

  };
  


#endif
