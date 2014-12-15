#ifndef UTILITIES_H
#define UTILITIES_H

/**
  Header file used for initializing useful data structures and 
  including the necessary libraries.
**/

#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Filter Libraries.
#include "pandora_vision_hazmat/histogram_mask.h"
#include "pandora_vision_hazmat/image_signature.h"



//~ #define CHRONO 
#define DEBUG 
#define HUE_RANGE {0,180}
#define SAT_RANGE {0,255}
#define DEFAULT_HIST_CHANNELS {0,1}
#define HIST_RANGE { HUE_RANGE , SAT_RANGE }

#ifdef CHRONO 
#include "sys/time.h"
#endif




/** 
 @enum TrainerType
 @brief Type of keypoint detector used .
 **/

enum TrainerType { SIFT , SURF };


struct Pattern
{
  public : 
  
    // Name of the pattern.
    std::string name ; 
    
    // A vector of 2D points that contains the bounding box
    // and the center of the pattern.
    std::vector<cv::Point2f> boundingBox ;
    
    // Vector of detected keypoints in the pattern.
    std::vector<cv::Point2f> keyPoints ;
    
    // Matrix of image descriptors .
    cv::Mat descriptors;
    
    // Color histogram of the pattern.
    cv::Mat histogram;
    
  };
  
#endif
