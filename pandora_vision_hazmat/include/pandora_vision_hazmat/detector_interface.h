#ifndef DETECTOR_INTERFACE_H
#define DETECTOR_INTERFACE_H

#include "pandora_vision_hazmat/Utilities.h"


// Abstract Interface 

class Detector
{
  public : 
  
    // Function for detection of the pattern on the current frame .   
     
    bool virtual detect(const cv::Mat &frame , 
      float *x , float *y ) = 0 ;
    
    // Function used for reading the training data from an xml file.
    
    void virtual readData( void ) = 0 ;
        
    
    // Function used to get the best feature matches between a frame
    // and a number of patterns.
    //~ int virtual getBestMatches( const cv::Mat &frame ,
     //~ const cv::Mat &features , double *minDist , double *maxDist  ) = 0;
     
    
    Detector() {}
    
    virtual ~Detector() {} ;
  
};

#endif
