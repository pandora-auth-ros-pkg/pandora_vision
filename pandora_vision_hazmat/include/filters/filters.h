#ifndef FILTERS_H
#define FILTERS_H

#include "pandora_vision_hazmat/detector_interface.h"

/** 
 @class HistogramMask
 @brief Class the implements the histogram back projection algorithm 
 **/
 

class HistogramMask : public Detector
{
  public : 
    
    // Function that calculates the normalized histogram
    static void calcNormYUVHist(const cv::Mat &image , cv::Mat *hist);
    
    // Function the calculates the backprojection 
    void createMask( cv::Mat frame , cv::Mat mask , cv::Mat data );
    
    // Normalize an image by dividing it with the channel ch.
    static void normalizeImage(cv::Mat &image , cv::Mat *normImage , 
      int ch );
    
    HistogramMask(Detector *baseDetector);
    
    virtual ~HistogramMask() 
    {
      
    } 
    
  private :
    // Pointer the base class that will be decorated.
    Detector *detector_ ;
  };

#endif
